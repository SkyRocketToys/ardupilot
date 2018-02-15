/*
  driver for Beken_2425 radio
 */
#include <AP_HAL/AP_HAL.h>

#pragma GCC optimize("O0")

#if defined(HAL_RCINPUT_WITH_AP_RADIO) && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412

#include <AP_Math/AP_Math.h>
#include "AP_Radio_bk2425.h"
#include <utility>
#include <stdio.h>
#include <StorageManager/StorageManager.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

static THD_WORKING_AREA(_irq_handler_wa, 2048);
#define TIMEOUT_PRIORITY 250	//Right above timer thread
#define EVT_TIMEOUT EVENT_MASK(0) // Event in the irq handler thread triggered by a timeout interrupt
#define EVT_IRQ EVENT_MASK(1) // Event in the irq handler thread triggered by a radio IRQ (Tx finished, Rx finished, MaxRetries limit)
#define EVT_BIND EVENT_MASK(2) // (not used yet) The user has clicked on the "start bind" button in the web interface (or equivalent).

extern const AP_HAL::HAL& hal;

// Output debug information on the UART, wrapped in MavLink packets
#define Debug(level, fmt, args...)   do { if ((level) <= get_debug_level()) { hal.console->printf(fmt, ##args); }} while (0)
// Output fast debug information on the UART, in raw format. MavLink should be disabled if you want to understand these messages.
// This is for debugging issues with frequency hopping and synchronisation.
#define DebugPrintf(level, fmt, args...)   do { if (radio_instance && ((level) <= radio_instance->get_debug_level())) { printf(fmt, ##args); }} while (0)


// object instance for trampoline
AP_Radio_beken *AP_Radio_beken::radio_instance;
thread_t *AP_Radio_beken::_irq_handler_ctx;
virtual_timer_t AP_Radio_beken::timeout_vt;
// See variable definitions in AP_Radio_bk2425.h for comments
uint32_t AP_Radio_beken::irq_time_us;
uint32_t AP_Radio_beken::irq_when_us;
uint32_t AP_Radio_beken::last_timeout_us;
uint32_t AP_Radio_beken::next_timeout_us;
uint32_t AP_Radio_beken::delta_timeout_us = 1000;
uint32_t AP_Radio_beken::next_switch_us;

// -----------------------------------------------------------------------------
// We have re
void SyncTiming::Rx(uint32_t when)
{
	uint32_t ld = delta_rx_time_us;
	uint32_t d = when - rx_time_us;
	if ((d > ld - DIFF_DELTA_RX) && (d < ld + DIFF_DELTA_RX)) // Two deltas are similar to each other
	{
		if ((d > TARGET_DELTA_RX-SLOP_DELTA_RX) && (d < TARGET_DELTA_RX+SLOP_DELTA_RX)) // delta is within range of single packet distance
		{
			// Use filter to change the estimate of the time in microseconds between the transmitters packet (according to OUR clock)
			sync_time_us = ((sync_time_us * (256-16)) + (d * 16)) / 256;
		}
	}
	rx_time_us = when;
	delta_rx_time_us = d;
	last_delta_rx_time_us = ld;
}


// -----------------------------------------------------------------------------

/*
  constructor
 */
AP_Radio_beken::AP_Radio_beken(AP_Radio &_radio) :
    AP_Radio_backend(_radio),
    beken(hal.spi->get_device("beken")) // trace this later - its on libraries/AP_HAL_ChibiOS/SPIDevice.cpp:92
{
    // link to instance for irq_trampoline
    
    // (temporary) go into test mode
    radio_instance = this;
    beken.fcc.fcc_mode = 0;
    beken.fcc.channel = 23;
    beken.fcc.power = 7;
}

/*
  initialise radio
 */
bool AP_Radio_beken::init(void)
{
    if(_irq_handler_ctx != nullptr) {
        AP_HAL::panic("AP_Radio_beken: double instantiation of irq_handler\n");
    }
    chVTObjectInit(&timeout_vt);
    _irq_handler_ctx = chThdCreateStatic(_irq_handler_wa,
                     sizeof(_irq_handler_wa),
                     TIMEOUT_PRIORITY,          /* Initial priority.    */
                     irq_handler_thd,           /* Thread function.     */
                     NULL);                     /* Thread parameter.    */
    sem = hal.util->new_semaphore();    
    
    return reset();
}

/*
  reset radio
 */
bool AP_Radio_beken::reset(void)
{
    if (!beken.lock_bus()) {
        return false;
    }

    radio_init();
    beken.unlock_bus();

    return true;
}

/*
  return statistics structure from radio
 */
const AP_Radio::stats &AP_Radio_beken::get_stats(void)
{
    return stats;
}

/*
  read one pwm channel from radio
 */
uint16_t AP_Radio_beken::read(uint8_t chan)
{
    if (chan >= BEKEN_MAX_CHANNELS) {
        return 0;
    }
    return pwm_channels[chan];
}

/*
  update status - called from main thread
 */
void AP_Radio_beken::update(void)
{
}
    

/*
  return number of active channels, and updates the data
 */
uint8_t AP_Radio_beken::num_channels(void)
{
    uint32_t now = AP_HAL::millis();
    uint8_t chan = get_rssi_chan();
    if ((chan > 0) && ((chan-1) < BEKEN_MAX_CHANNELS)) {
        pwm_channels[chan-1] = 50; // Fixed value that will not update
        chan_count = MAX(chan_count, chan);
    }

    chan = get_pps_chan();
    if ((chan > 0) && ((chan-1) < BEKEN_MAX_CHANNELS)) {
        pwm_channels[chan-1] = t_status.pps; // How many packets received per second
        chan_count = MAX(chan_count, chan);
    }

    chan = get_tx_rssi_chan();
    if ((chan > 0) && ((chan-1) < BEKEN_MAX_CHANNELS)) {
        pwm_channels[chan-1] = 50; // Fixed value that will not update
        chan_count = MAX(chan_count, chan);
    }

    chan = get_tx_pps_chan();
    if ((chan > 0) && ((chan-1) < BEKEN_MAX_CHANNELS)) {
        pwm_channels[chan-1] = tx_pps;
        chan_count = MAX(chan_count, chan);
    }
    
    // Every second, update the statistics
    if (now - last_pps_ms > 1000) {
        last_pps_ms = now;
        t_status.pps = stats.recv_packets - last_stats.recv_packets;
        last_stats = stats;
        if (lost != 0 || timeouts != 0) {
            Debug(3,"lost=%lu timeouts=%lu\n", lost, timeouts);
        }
        lost=0;
        timeouts=0;
    }
    return chan_count;
}

/*
  return time of last receive in microseconds
 */
uint32_t AP_Radio_beken::last_recv_us(void)
{
    return synctm.packet_timer;
}

/*
  send len bytes as a single packet
 */
bool AP_Radio_beken::send(const uint8_t *pkt, uint16_t len)
{
    // disabled for now
    return false;
}

static const uint16_t CRCTable[] = {
        0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,
        0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
        0x1081,0x0108,0x3393,0x221a,0x56a5,0x472c,0x75b7,0x643e,
        0x9cc9,0x8d40,0xbfdb,0xae52,0xdaed,0xcb64,0xf9ff,0xe876,
        0x2102,0x308b,0x0210,0x1399,0x6726,0x76af,0x4434,0x55bd,
        0xad4a,0xbcc3,0x8e58,0x9fd1,0xeb6e,0xfae7,0xc87c,0xd9f5,
        0x3183,0x200a,0x1291,0x0318,0x77a7,0x662e,0x54b5,0x453c,
        0xbdcb,0xac42,0x9ed9,0x8f50,0xfbef,0xea66,0xd8fd,0xc974,
        0x4204,0x538d,0x6116,0x709f,0x0420,0x15a9,0x2732,0x36bb,
        0xce4c,0xdfc5,0xed5e,0xfcd7,0x8868,0x99e1,0xab7a,0xbaf3,
        0x5285,0x430c,0x7197,0x601e,0x14a1,0x0528,0x37b3,0x263a,
        0xdecd,0xcf44,0xfddf,0xec56,0x98e9,0x8960,0xbbfb,0xaa72,
        0x6306,0x728f,0x4014,0x519d,0x2522,0x34ab,0x0630,0x17b9,
        0xef4e,0xfec7,0xcc5c,0xddd5,0xa96a,0xb8e3,0x8a78,0x9bf1,
        0x7387,0x620e,0x5095,0x411c,0x35a3,0x242a,0x16b1,0x0738,
        0xffcf,0xee46,0xdcdd,0xcd54,0xb9eb,0xa862,0x9af9,0x8b70,
        0x8408,0x9581,0xa71a,0xb693,0xc22c,0xd3a5,0xe13e,0xf0b7,
        0x0840,0x19c9,0x2b52,0x3adb,0x4e64,0x5fed,0x6d76,0x7cff,
        0x9489,0x8500,0xb79b,0xa612,0xd2ad,0xc324,0xf1bf,0xe036,
        0x18c1,0x0948,0x3bd3,0x2a5a,0x5ee5,0x4f6c,0x7df7,0x6c7e,
        0xa50a,0xb483,0x8618,0x9791,0xe32e,0xf2a7,0xc03c,0xd1b5,
        0x2942,0x38cb,0x0a50,0x1bd9,0x6f66,0x7eef,0x4c74,0x5dfd,
        0xb58b,0xa402,0x9699,0x8710,0xf3af,0xe226,0xd0bd,0xc134,
        0x39c3,0x284a,0x1ad1,0x0b58,0x7fe7,0x6e6e,0x5cf5,0x4d7c,
        0xc60c,0xd785,0xe51e,0xf497,0x8028,0x91a1,0xa33a,0xb2b3,
        0x4a44,0x5bcd,0x6956,0x78df,0x0c60,0x1de9,0x2f72,0x3efb,
        0xd68d,0xc704,0xf59f,0xe416,0x90a9,0x8120,0xb3bb,0xa232,
        0x5ac5,0x4b4c,0x79d7,0x685e,0x1ce1,0x0d68,0x3ff3,0x2e7a,
        0xe70e,0xf687,0xc41c,0xd595,0xa12a,0xb0a3,0x8238,0x93b1,
        0x6b46,0x7acf,0x4854,0x59dd,0x2d62,0x3ceb,0x0e70,0x1ff9,
        0xf78f,0xe606,0xd49d,0xc514,0xb1ab,0xa022,0x92b9,0x8330,
        0x7bc7,0x6a4e,0x58d5,0x495c,0x3de3,0x2c6a,0x1ef1,0x0f78
};

/*
  initialise the radio
 */
void AP_Radio_beken::radio_init(void)
{
	DebugPrintf(1, "radio_init\r\n");
    beken.SetRBank(1);
    uint8_t id = beken.ReadReg(BK2425_R1_WHOAMI); // id is now 99
    beken.SetRBank(0); // Reset to default register bank.

    if (id != BK_CHIP_ID_BK2425) {
        
        Debug(1, "bk2425: radio not found\n"); // We have to keep trying  because it takes time to initialise
        return; // Failure
    }

    Debug(1, "beken: radio_init starting\n");

    beken.bkReady = 0;
    spd = beken.gTxSpeed;
	beken.SwitchToIdleMode();
    hal.scheduler->delay(100); // delay more than 50ms.

    // Initialise Beken registers
    beken.SetRBank(0);
    beken.InitBank0Registers(beken.gTxSpeed);
    beken.SetRBank(1);
    beken.InitBank1Registers(beken.gTxSpeed);
    hal.scheduler->delay(100); // delay more than 50ms.
    beken.SetRBank(0);
    
    beken.SwitchToRxMode(); // switch to RX mode
    beken.bkReady = 1;
    hal.scheduler->delay_microseconds(10*1000); // 10ms seconds delay
    
    // setup handler for rising edge of IRQ pin
    hal.gpio->attach_interrupt(HAL_GPIO_RADIO_IRQ, trigger_irq_radio_event, HAL_GPIO_INTERRUPT_FALLING);

    if (load_bind_info()) { // what happens here? 
        Debug(3,"Loaded bind info\n");
        nextChannel(1);
    }

    chVTSet(&timeout_vt, MS2ST(10), trigger_timeout_event, nullptr); // Initial timeout?
    if (3 <= get_debug_level())
		beken.DumpRegisters();
}

void AP_Radio_beken::trigger_irq_radio_event()
{
    //we are called from ISR context
    chSysLockFromISR();
    irq_time_us = AP_HAL::micros();
    chEvtSignalI(_irq_handler_ctx, EVT_IRQ);
    chSysUnlockFromISR();
}

void AP_Radio_beken::trigger_timeout_event(void *arg)
{
    (void)arg;
    //we are called from ISR context
	next_timeout_us += delta_timeout_us;
	last_timeout_us = AP_HAL::micros();
	if (int32_t(next_timeout_us - last_timeout_us) < 500) // Too late for this one
		next_timeout_us = last_timeout_us + delta_timeout_us;
	uint32_t delta = US2ST(next_timeout_us - last_timeout_us);

    chSysLockFromISR();
    chVTSetI(&timeout_vt, delta, trigger_timeout_event, nullptr); // Timeout every 1 ms
    chEvtSignalI(_irq_handler_ctx, EVT_TIMEOUT);
    chSysUnlockFromISR();
}

// The user has clicked on the "Start Bind" button on the web interface
void AP_Radio_beken::start_recv_bind(void)
{
    chan_count = 0;
    synctm.packet_timer = AP_HAL::micros();
    chEvtSignal(_irq_handler_ctx, EVT_BIND);
    Debug(1,"Starting bind\n");
}

// handle a data96 mavlink packet for fw upload
void AP_Radio_beken::handle_data_packet(mavlink_channel_t chan, const mavlink_data96_t &m)
{
}


// ----------------------------------------------------------------------------
// Update the telemetry status variable; can be called in irq thread
// since the functions it calls are lightweight
void AP_Radio_beken::update_SRT_telemetry(void)
{
    t_status.flags = 0;
    t_status.flags |= AP_Notify::flags.gps_status >= 3?TELEM_FLAG_GPS_OK:0;
    t_status.flags |= AP_Notify::flags.pre_arm_check?TELEM_FLAG_ARM_OK:0;
    t_status.flags |= AP_Notify::flags.failsafe_battery?0:TELEM_FLAG_BATT_OK;
    t_status.flags |= hal.util->get_soft_armed()?TELEM_FLAG_ARMED:0;
    t_status.flags |= AP_Notify::flags.have_pos_abs?TELEM_FLAG_POS_OK:0;
    t_status.flags |= AP_Notify::flags.video_recording?TELEM_FLAG_VIDEO:0;
    t_status.flight_mode = AP_Notify::flags.flight_mode;
    t_status.tx_max = get_tx_max_power();
    t_status.note_adjust = get_tx_buzzer_adjust();
}

// ----------------------------------------------------------------------------
// Update a radio control packet
// Called from IRQ context
void AP_Radio_beken::UpdateTxData(void)
{
	packetFormatTx* tx = &beken.pktDataTx;

	// Base values for this packet type
	update_SRT_telemetry();
	tx->packetType = BK_PKT_TYPE_TELEMETRY; ///< The packet type
//	tx->channel;
	tx->pps = t_status.pps;
	tx->flags = t_status.flags;
	tx->droneid[0] = myDroneId[0];
	tx->droneid[1] = myDroneId[1];
	tx->droneid[2] = myDroneId[2];
	tx->droneid[3] = myDroneId[3];
	tx->flight_mode = t_status.flight_mode;
	tx->wifi = t_status.wifi_chan + (24 * t_status.tx_max);
	tx->note_adjust = t_status.note_adjust;
}


// ----------------------------------------------------------------------------
/* support all 4 rc input modes by swapping channels. */
void AP_Radio_beken::map_stick_mode(void)
{
    switch (get_stick_mode()) {
    case 1: {
        // mode1 = swap throttle and pitch
        uint16_t tmp = pwm_channels[1];
        pwm_channels[1] = pwm_channels[2];
        pwm_channels[2] = tmp;
        break;
    }

    case 3: {
        // mode3 = swap throttle and pitch, swap roll and yaw
        uint16_t tmp = pwm_channels[1];
        pwm_channels[1] = pwm_channels[2];
        pwm_channels[2] = tmp;
        tmp = pwm_channels[0];
        pwm_channels[0] = pwm_channels[3];
        pwm_channels[3] = tmp;
        break;
    }

    case 4: {
        // mode4 = swap roll and yaw
        uint16_t tmp = pwm_channels[0];
        pwm_channels[0] = pwm_channels[3];
        pwm_channels[3] = tmp;
        break;
    }
        
    case 2:
    default:
        // nothing to do, transmitter is natively mode2
        break;
    }
}

// ----------------------------------------------------------------------------
// Handle receiving a packet (we are still in an interrupt!)
void AP_Radio_beken::ProcessPacket(const uint8_t* packet, uint8_t rxaddr)
{
    switch (packet[0]) {
	case BK_PKT_TYPE_CTRL_FOUND:
	case BK_PKT_TYPE_CTRL_LOST:
		// We haz data
		if (rxaddr == 0)
		{
			syncch.SetChannel(packet[1]);
		    synctm.packet_timer = AP_HAL::micros(); // This is essential for letting the channels update
			// Put the data into the control values
			pwm_channels[0] = 1000 + packet[2] + (uint16_t(packet[6] & 0xC0) << 2); // Roll
			pwm_channels[1] = 1000 + packet[3] + (uint16_t(packet[6] & 0x30) << 4); // Pitch
			pwm_channels[2] = 1000 + packet[4] + (uint16_t(packet[6] & 0x0C) << 6); // Throttle
			pwm_channels[3] = 1000 + packet[5] + (uint16_t(packet[6] & 0x03) << 8); // Yaw
			pwm_channels[4] = 1000 + ((packet[7] & 0x07) >> 0) * 100; // SW1, SW2, SW3
			pwm_channels[5] = 1000 + ((packet[7] & 0x38) >> 3) * 100; // SW4, SW5, SW6
			// cope with mode1/mode2/mode3/mode4
			map_stick_mode();
			chan_count = MAX(chan_count, 6);
			switch (packet[9]) {
			case BK_INFO_FW_VER: break;
			case BK_INFO_DFU_RX: break;
			case BK_INFO_FW_CRC_LO: break;
			case BK_INFO_FW_CRC_HI: break;
			case BK_INFO_FW_YM: break;
			case BK_INFO_FW_DAY: break;
			case BK_INFO_MODEL: break;
			case BK_INFO_PPS:
				tx_pps = packet[10]; // Remember pps from tx
				break;
			case BK_INFO_BATTERY:
				// "voltage from TX is in 0.025 volt units". Convert to 0.01 volt units for easier display
				// The CC2500 code (and this) actually assumes it is in 0.04 volt units, hence the tx scaling by 23/156 (38/256) instead of 60/256)
				// Which means a maximum value is 152 units representing 6.0v rather than 240 units representing 6.0v
		        pwm_channels[6] = packet[10] * 4;
				break;
			case BK_INFO_COUNTDOWN:
				if (packet[10])
				{
					syncch.SetCountdown(packet[10]+1, packet[11]);
					DebugPrintf(2, "%d ", packet[10]);
				}
				break;
			default:
				break;
			};
		}
		break;
	case BK_PKT_TYPE_BIND:
		if (rxaddr == 1)
		{
			// Set the address on which we are receiving the control data
			syncch.SetChannel(packet[1]);
			beken.SetAddresses(&packet[2]);
			Debug(3, " Bound to %x %x %x %x %x\r\n", packet[2], packet[3], packet[4], packet[5], packet[6]);
		}
		break;
	case BK_PKT_TYPE_TELEMETRY:
	case BK_PKT_TYPE_DFU:
	default:
		// This is one of our packets! Ignore it.
		break;
	}
}

// ----------------------------------------------------------------------------
// Prepare to send a FCC packet
void AP_Radio_beken::UpdateFccScan(void)
{
	// Support scan mode
    if (beken.fcc.scan_mode) {
        beken.fcc.scan_count++;
        if (beken.fcc.scan_count >= 200) {
            beken.fcc.scan_count = 0;
            beken.fcc.channel += 2; // Go up by 2Mhz
            if (beken.fcc.channel >= CHANNEL_FCC_HIGH) {
                beken.fcc.channel = CHANNEL_FCC_LOW;
            }
        }
    }
}

// ----------------------------------------------------------------------------
// main IRQ handler
void AP_Radio_beken::irq_handler(uint32_t when)
{
    if (beken.fcc.fcc_mode) {
        // don't process interrupts in FCCTEST mode
		beken.WriteReg(BK_WRITE_REG | BK_STATUS,
			(BK_STATUS_RX_DR | BK_STATUS_TX_DS | BK_STATUS_MAX_RT)); // clear RX_DR or TX_DS or MAX_RT interrupt flag
        return;
    }
    
	// Determine which state fired the interrupt
	uint8_t bk_sta = beken.ReadStatus();
	if (bk_sta & BK_STATUS_TX_DS)
	{
		// Packet was sent towards the Tx board
		synctm.tx_time_us = when;
//		stats.sentPacketCount++;
		beken.SwitchToRxMode(); // Prepare to receive next packet (on the next channel)
		nextChannel(1);
		DebugPrintf(2, "T");
	}
	if (bk_sta & BK_STATUS_MAX_RT)
	{
		// We have had a "max retries" error
	}
	bool bReply = false;
	if (bk_sta & BK_STATUS_RX_DR)
	{
		// We have received a packet
		uint8_t rxstd = 0;
//		DebugPrintf(2, "R%ld,%ld\r\n", when, synctm.sync_time_us);
		DebugPrintf(2, "R");
		// Which pipe (address) have we received this packet on?
		if ((bk_sta & BK_STATUS_RX_MASK) == BK_STATUS_RX_P_0)
		{
			rxstd = 0;
		}
		else if ((bk_sta & BK_STATUS_RX_MASK) == BK_STATUS_RX_P_1)
		{
			rxstd = 1;
		}
		else
		{
			stats.recv_errors++;
		}
		
		uint8_t len, fifo_sta;
		uint8_t packet[32];
		do
		{
            stats.recv_packets++;
			len = beken.ReadReg(BK_R_RX_PL_WID_CMD);	// read received packet length in bytes

			if (len <= PACKET_LENGTH_RX_MAX)
			{
				bReply = true;
				synctm.Rx(when);
				next_switch_us = when + synctm.sync_time_us + 3000; // Switch channels if we miss the next packet
				// This includes short packets (e.g. where no telemetry was sent)
				beken.ReadRegisterMulti(BK_RD_RX_PLOAD, packet, len); // read receive payload from RX_FIFO buffer
//				DebugPrintf(3, "Packet %d(%d) %d %d %d %d %d %d %d %d ...\r\n", rxstd, len,
//					packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7]);
			}
			else // Packet was too long
			{
				beken.ReadRegisterMulti(BK_RD_RX_PLOAD, packet, 32); // read receive payload from RX_FIFO buffer
				beken.Strobe(BK_FLUSH_RX); // flush Rx
			}
			fifo_sta = beken.ReadReg(BK_FIFO_STATUS);	// read register FIFO_STATUS's value
		} while (!(fifo_sta & BK_FIFO_STATUS_RX_EMPTY)); // while not empty
		beken.WriteReg(BK_WRITE_REG | BK_STATUS,
			(BK_STATUS_RX_DR | BK_STATUS_TX_DS | BK_STATUS_MAX_RT)); // clear RX_DR or TX_DS or MAX_RT interrupt flag
		ProcessPacket(packet, rxstd);
	}

	// Clear the bits
	beken.WriteReg((BK_WRITE_REG|BK_STATUS), (BK_STATUS_MAX_RT | BK_STATUS_TX_DS | BK_STATUS_RX_DR));
	if (bReply)
	{
		hal.scheduler->delay_microseconds(100); // delay to give the tx a chance to switch to receive mode
//		if (AP_HAL::micros() >= next_switch_us) // The delay was so long we need to swap channels
//		{
//			nextChannel(1);
//			DebugPrintf(2, "x");
//		}

		// Send the telemetry reply to the controller
		beken.Strobe(BK_FLUSH_TX); // flush Tx
		beken.ClearAckOverflow();
		beken.SwitchToTxMode();
		UpdateTxData();
		beken.pktDataTx.channel = syncch.channel;
		beken.SendPacket(BK_WR_TX_PLOAD, (uint8_t *)&beken.pktDataTx, PACKET_LENGTH_TX_TELEMETRY);
	}
}	

// ----------------------------------------------------------------------------
// handle timeout IRQ (called every ms)
void AP_Radio_beken::irq_timeout(void)
{
	if (!beken.bkReady) // We are reinitialising the chip in the main thread
	{
		return;
	}

	// Set the transmission power
	uint8_t pwr = get_transmit_power();
	if (pwr != beken.fcc.power + 1)
	{
		if ((pwr > 0) && (pwr <= 8))
		{
			beken.SetPower(pwr-1);
		}
	}
	
	// Do we need to change our fcc test mode status?
	uint8_t fcc = get_fcc_test();
	if (fcc != beken.fcc.fcc_mode)
	{
		beken.Strobe(BK_FLUSH_TX);
		if (fcc == 0) // Turn off fcc test mode
		{
			if (beken.fcc.CW_mode)
			{
				beken.SwitchToIdleMode();
				beken.SetCwMode(false);
			}
		}
		else
		{
			if (fcc > 3)
			{
				if (!beken.fcc.CW_mode)
				{
					beken.SwitchToIdleMode();
					beken.SetCwMode(true);
					beken.DumpRegisters();
				}
			}
			else
			{
				if (beken.fcc.CW_mode)
				{
					beken.SwitchToIdleMode();
					beken.SetCwMode(false);
				}
			}
			switch (fcc) {
			case 1: case 4:
			default:
				beken.fcc.channel = CHANNEL_FCC_LOW;
				break;
			case 2: case 5:
				beken.fcc.channel = CHANNEL_FCC_MID;
				break;
			case 3: case 6:
				beken.fcc.channel = CHANNEL_FCC_HIGH;
				break;
			};
		}
		beken.fcc.fcc_mode = fcc;
		DebugPrintf(1, "\r\nFCC mode %d\r\n", fcc);
	}

	// For fcc mode, just send packets on timeouts
	if (beken.fcc.fcc_mode)
	{
		static uint8_t tt = 0;
		if (++tt >= 5) // Space out to every 5 ms
		{
			tt = 0;
		}
		else
		{
			return;
		}

		beken.SwitchToTxMode();
		beken.ClearAckOverflow();
		UpdateFccScan();
		beken.SetChannel(beken.fcc.channel);
		UpdateTxData();
		beken.pktDataTx.channel = 0;
		if (!beken.fcc.CW_mode)
		{
			beken.SendPacket(BK_WR_TX_PLOAD, (uint8_t *)&beken.pktDataTx, PACKET_LENGTH_TX_TELEMETRY);
//			DebugPrintf(3, "*");
		}
		return;
	}

	// Normal modes - we have timed out for channel hopping
	if (last_timeout_us >= next_switch_us) // We can swap channels now
	{
		int32_t d = synctm.sync_time_us; // Time between packets, e.g. 5100 us
		uint32_t dt = last_timeout_us - synctm.rx_time_us;
		if (dt > 50*d) // We have lost sync (missed 50 packets) so slow down the channel hopping until we resync
		{
			d *= 4;
			DebugPrintf(2, "C");
		}
		else
		{
			DebugPrintf(2, "c");
		}
		{
			uint8_t fifo_sta = radio_instance->beken.ReadReg(BK_FIFO_STATUS);	// read register FIFO_STATUS's value
			if (!(fifo_sta & BK_FIFO_STATUS_RX_EMPTY)) // while not empty
			{
				DebugPrintf(2, "#");
				radio_instance->irq_handler(AP_HAL::micros());
			}
			else
			{
				next_switch_us += d; // Switch channels if we miss the next packet
			}
		}
		int32_t ss = int32_t(next_switch_us - last_timeout_us);
		if (ss < 1000) // Not enough time
		{
			next_switch_us = last_timeout_us + d; // Switch channels if we miss the next packet
			DebugPrintf(2, "j");
		}
		if (beken.WasTxMode())
			beken.SwitchToRxMode();
		nextChannel(1);
		beken.ClearAckOverflow();
	}
}

// ----------------------------------------------------------------------------
// Thread that supports Beken Radio work triggered by interrupts
// This is the only thread that should access the Beken radio chip via SPI.
void AP_Radio_beken::irq_handler_thd(void *arg)
{
    (void) arg;
    while(true) {
        eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
        if (_irq_handler_ctx != nullptr) // Sanity check
			_irq_handler_ctx->name = "RadioBeken"; // Only useful to be done once but here is done often

        radio_instance->beken.lock_bus();
        irq_when_us = irq_time_us; // Get the time of the event
        switch(evt) {
        case EVT_IRQ:
            if (radio_instance->beken.fcc.fcc_mode != 0) {
                DebugPrintf(3, "IRQ FCC\n");
            }
            radio_instance->irq_handler(irq_when_us);
            break;
        case EVT_TIMEOUT:
			radio_instance->irq_timeout();
            break;
        case EVT_BIND: // The user has clicked on the "Start Bind" button on the web interface
			DebugPrintf(2, "\r\nBtnStartBind\r\n");
            break;
        default:
            break;
        }
        radio_instance->beken.unlock_bus();
    }
}

void AP_Radio_beken::setChannel(uint8_t channel)
{
	beken.SetChannel(channel);
}

enum {
	CHANNEL_COUNT_LOGICAL = 16, ///< The maximum number of entries in each frequency table
	CHANNEL_BASE_TABLE = 0, ///< The table used for non wifi boards
	CHANNEL_SAFE_TABLE = 3, ///< A table that will receive packets even if wrong
	CHANNEL_NUM_TABLES = 6, ///< The number of tables
};
const uint8_t bindHopData[CHANNEL_NUM_TABLES*CHANNEL_COUNT_LOGICAL] = {
#if 0
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
#else
	46,41,31,52,36,13,72,69, 21,56,16,26,61,66,10,45, // Normal
	57,62,67,72,58,63,68,59, 64,69,60,65,70,61,66,71, // Wifi channel 1,2,3,4,5
	62,10,67,72,63,68,11,64, 69,60,65,70,12,61,66,71, // Wifi channel 6
	10,67,11,72,12,68,13,69, 14,65,15,70,16,66,17,71, // Wifi channel 7
	10,70,15,20,11,71,16,21, 12,17,22,72,13,18,14,19, // Wifi channel 8
	10,15,20,25,11,16,21,12, 17,22,13,18,23,14,19,24, // Wifi channel 9,10,11
#endif
};

void AP_Radio_beken::nextChannel(uint8_t skip)
{
    if (skip)
		syncch.NextChannel();
    setChannel(bindHopData[syncch.channel]);
}

uint16_t AP_Radio_beken::calc_crc(uint8_t *data, uint8_t len)
{
    uint16_t crc = 0;
    for(uint8_t i=0; i < len; i++) {
        crc = (crc<<8) ^ (CRCTable[((uint8_t)(crc>>8) ^ *data++) & 0xFF]);
    }
    return crc;
}

bool AP_Radio_beken::check_crc(uint8_t ccLen, uint8_t *packet)
{
    uint16_t lcrc = calc_crc(&packet[3],(ccLen-7));
    return ((lcrc >>8)==packet[ccLen-4] && (lcrc&0x00FF)==packet[ccLen-3]);
}

/*
  save bind info
 */
void AP_Radio_beken::save_bind_info(void)
{
    // access to storage for bind information
    StorageAccess bind_storage(StorageManager::StorageBindInfo);
    struct bind_info info;
    
    info.magic = bind_magic;
    info.bindTxId[0] = beken.TX_Address[0];
    info.bindTxId[1] = beken.TX_Address[1];
    info.bindTxId[2] = beken.TX_Address[2];
    info.bindTxId[3] = beken.TX_Address[3];
    info.bindTxId[4] = beken.TX_Address[4];
    bind_storage.write_block(0, &info, sizeof(info));
}

/*
  load bind info
 */
bool AP_Radio_beken::load_bind_info(void)
{
    // access to storage for bind information
    StorageAccess bind_storage(StorageManager::StorageBindInfo);
    struct bind_info info;

    if (!bind_storage.read_block(&info, 0, sizeof(info)) || info.magic != bind_magic) {
        return false;
    }

//    beken.TX_Address[0] = info.bindTxId[0];
//    beken.TX_Address[1] = info.bindTxId[1];
//    beken.TX_Address[2] = info.bindTxId[2];
//    beken.TX_Address[3] = info.bindTxId[3];
//    beken.TX_Address[4] = info.bindTxId[4];

    return true;
}

/*
  send a telemetry packet
 */
void AP_Radio_beken::send_telemetry(void)
{
    uint8_t frame[15];

    memset(frame, 0, sizeof(frame));
    frame[0] = BK_PKT_TYPE_TELEMETRY;
}

// Step through the channels
void SyncChannel::NextChannel(void)
{
	if (countdown != countdown_invalid)
	{
		if (--countdown == 0)
		{
			channel = countdown_chan;
			countdown = countdown_invalid;
//			DebugPrintf(2, "{%d} ", countdown_chan); // radio_instance not accessible
			return;
		}
	}
	uint8_t table = channel / CHANNEL_COUNT_LOGICAL;
	channel = (channel + 1) % CHANNEL_COUNT_LOGICAL;
	channel += table * CHANNEL_COUNT_LOGICAL;
}

// If we have not received any packets for ages, try a WiFi table that covers all frequencies
void SyncChannel::SafeTable(void)
{
	uint8_t table = channel / CHANNEL_COUNT_LOGICAL;
	if ((table != CHANNEL_BASE_TABLE) && (table != CHANNEL_SAFE_TABLE)) // Are we using a table that is high end or low end only?
	{
		channel %= CHANNEL_COUNT_LOGICAL;
		channel += CHANNEL_SAFE_TABLE * CHANNEL_COUNT_LOGICAL;
	}
}

#endif // HAL_RCINPUT_WITH_AP_RADIO

