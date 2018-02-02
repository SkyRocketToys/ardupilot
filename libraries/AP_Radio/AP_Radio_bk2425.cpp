/*
  driver for Beken_2425 radio
 */
#include <AP_HAL/AP_HAL.h>

#pragma GCC optimize("O0")

#ifdef HAL_RCINPUT_WITH_AP_RADIO

#include <AP_Math/AP_Math.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <board_config.h>
#endif
#include "AP_Radio_bk2425.h"
#include <utility>
#include <stdio.h>
#include <StorageManager/StorageManager.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
static THD_WORKING_AREA(_irq_handler_wa, 512);
#define TIMEOUT_PRIORITY 250	//Right above timer thread
#define EVT_TIMEOUT EVENT_MASK(0)
#define EVT_IRQ EVENT_MASK(1)
#define EVT_BIND EVENT_MASK(2)
#endif

extern const AP_HAL::HAL& hal;

#define Debug(level, fmt, args...)   do { if ((level) <= get_debug_level()) { hal.console->printf(fmt, ##args); }} while (0)

#define LP_FIFO_SIZE  16      // Physical data FIFO lengths in Radio

// object instance for trampoline
AP_Radio_beken *AP_Radio_beken::radio_instance;
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
thread_t *AP_Radio_beken::_irq_handler_ctx;
virtual_timer_t AP_Radio_beken::timeout_vt;
uint32_t AP_Radio_beken::irq_time_us;
#endif

#define USER_ADDRESS_COMPONENT 0x59
#define MODEL_ADDRESS_COMPONENT 0x31
	
uint8_t TX_Address[]={0x80,USER_ADDRESS_COMPONENT,MODEL_ADDRESS_COMPONENT,0xC6,0x2C}; // non-const (Unbound tx)
uint8_t RX0_Address[]={0x01,USER_ADDRESS_COMPONENT,MODEL_ADDRESS_COMPONENT,0xC6,0x2C}; // non-const The data address
uint8_t RX1_Address[]={0x00,USER_ADDRESS_COMPONENT,MODEL_ADDRESS_COMPONENT,0xC6,0x2C}; // non-const The pairing address

/*
  constructor
 */
AP_Radio_beken::AP_Radio_beken(AP_Radio &_radio) :
    AP_Radio_backend(_radio),
    beken(hal.spi->get_device("beken")) // trace this later - its on libraries/AP_HAL_ChibiOS/SPIDevice.cpp:92
{
    // link to instance for irq_trampoline
    radio_instance = this;
}

/*
  initialise radio
 */
bool AP_Radio_beken::init(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    if(_irq_handler_ctx != nullptr) {
        AP_HAL::panic("AP_Radio_beken: double instantiation of irq_handler\n");
    }
    chVTObjectInit(&timeout_vt);
    _irq_handler_ctx = chThdCreateStatic(_irq_handler_wa,
                     sizeof(_irq_handler_wa),
                     TIMEOUT_PRIORITY,          /* Initial priority.    */
                     irq_handler_thd,           /* Thread function.     */
                     NULL);                     /* Thread parameter.    */
#endif
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
  return number of active channels
 */
uint8_t AP_Radio_beken::num_channels(void)
{
    uint32_t now = AP_HAL::millis();
    uint8_t chan = get_rssi_chan();
    if (chan > 0) {
        pwm_channels[chan-1] = t_status.rssi;
        chan_count = MAX(chan_count, chan);
    }

    chan = get_pps_chan();
    if (chan > 0) {
        pwm_channels[chan-1] = t_status.pps;
        chan_count = MAX(chan_count, chan);
    }

#if 0
    ch = get_tx_rssi_chan();
    if (ch > 0) {
        dsm.pwm_channels[ch-1] = dsm.tx_rssi;
        dsm.num_channels = MAX(dsm.num_channels, ch);
    }

    ch = get_tx_pps_chan();
    if (ch > 0) {
        dsm.pwm_channels[ch-1] = dsm.tx_pps;
        dsm.num_channels = MAX(dsm.num_channels, ch);
    }
#endif
    
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
    return packet_timer;
}

/*
  send len bytes as a single packet
 */
bool AP_Radio_beken::send(const uint8_t *pkt, uint16_t len)
{
    // disabled for now
    return false;
}

const uint16_t CRCTable[] = {
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
    beken.SetRBank(1);
    uint8_t id = beken.ReadReg(BK2425_R1_WHOAMI); // id is now 99
    beken.SetRBank(0); // carl - why do we access bank 0 after accessing bank 1?

    if (id != BK_CHIP_ID_BK2425) {
        
        Debug(1, "bk2425: radio not found\n"); // We have to keep trying  because it takes time to initialise
        return; // Failure
    }

    Debug(1, "beken: radio_init starting\n");

    int8_t i;
    beken.bkReady = 0;
    spd = beken.gTxSpeed;
    hal.scheduler->delay(100);//delay more than 50ms.
    beken.SetRBank(0);

    //********************Write Bank0 register******************
    for (i=20; i >= 0; i--) // From BK_FIFO_STATUS back to beginning of table
    {
        uint8_t idx = Bank0_Reg[i][0];
        uint8_t value = Bank0_Reg[i][1];
        if (idx == BK_RF_SETUP) // Adjust for speed
            value = Bank0_Reg6[spd][1];
        beken.WriteReg((BK_WRITE_REG|idx), value);
    }

    // Set the various 5 byte addresses
    beken.WriteRegisterMulti((BK_WRITE_REG|BK_RX_ADDR_P0),RX0_Address,5); // reg 10 - Rx0 addr
    beken.WriteRegisterMulti((BK_WRITE_REG|BK_RX_ADDR_P1),RX1_Address,5); // REG 11 - Rx1 addr
    beken.WriteRegisterMulti((BK_WRITE_REG|BK_TX_ADDR),TX_Address,5); // REG 16 - TX addr

#if RADIO_BEKEN
    i = beken.ReadReg(BK_FEATURE);
    if (i == 0) // i!=0 showed that chip has been actived.so do not active again.
        beken.WriteReg(BK_ACTIVATE_CMD,0x73);// Active
#endif
    for (i = 22; i >= 21; i--)
        beken.WriteReg((BK_WRITE_REG|Bank0_Reg[i][0]),Bank0_Reg[i][1]);

#if RADIO_BEKEN
    //********************Write Bank1 register******************
    beken.SetRBank(1);
    for (i = IREG1_4; i <= IREG1_13; i++)
    {
        const uint8_t* p = &Bank1_RegTable[spd][i][0];
        uint8_t idx = *p++;
        beken.WriteRegisterMulti((BK_WRITE_REG|idx), p, 4);
    }
    beken.WriteRegisterMulti((BK_WRITE_REG|BK2425_R1_14),&(Bank1_Reg14[0]),11);

//toggle REG4<25,26>
    {
        const uint8_t* p = &Bank1_RegTable[spd][IREG1_4A][0];
        uint8_t idx = *p++;
        beken.WriteRegisterMulti((BK_WRITE_REG|idx), p, 4);
    }
    {
        const uint8_t* p = &Bank1_RegTable[spd][IREG1_4][0];
        uint8_t idx = *p++;
        beken.WriteRegisterMulti((BK_WRITE_REG|idx), p, 4);
    }

    hal.scheduler->delay(100);//delay more than 50ms.

    /********************switch back to Bank0 register access******************/
    beken.SetRBank(0);
#else
    hal.scheduler->delay_microseconds(1000*100);
#endif
    beken.SwitchToRxMode(); // switch to RX mode
    beken.bkReady = 1;
    
    //beken.Reset();
    //for (uint8_t i=0; i<ARRAY_SIZE(radio_config); i++) {
    //    cc2500.WriteReg(radio_config[i].reg, radio_config[i].value);
    //}
    //cc2500.Strobe(CC2500_SIDLE);	// Go to idle...

    //for (uint8_t c=0;c<0xFF;c++) {
    //    //calibrate all channels
    //    cc2500.Strobe(CC2500_SIDLE);
    //    cc2500.WriteReg(CC2500_0A_CHANNR, c);
    //    cc2500.Strobe(CC2500_SCAL);
    //    hal.scheduler->delay_microseconds(900);
    //    calData[c][0] = cc2500.ReadReg(CC2500_23_FSCAL3);
    //    calData[c][1] = cc2500.ReadReg(CC2500_24_FSCAL2);
    //    calData[c][2] = cc2500.ReadReg(CC2500_25_FSCAL1);
    //}

    hal.scheduler->delay_microseconds(10*1000); // why do we need to have a 10 seconds delay?
    
    // setup handler for rising edge of IRQ pin
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    stm32_gpiosetevent(CYRF_IRQ_INPUT, true, false, false, irq_radio_trampoline);
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    hal.gpio->attach_interrupt(HAL_GPIO_RADIO_IRQ, trigger_irq_radio_event, HAL_GPIO_INTERRUPT_RISING);
#endif

    if (load_bind_info()) { // what happens here? 
        Debug(3,"Loaded bind info\n");
        listLength = 47;
        initialiseData(0);
        protocolState = STATE_SEARCH;
        chanskip = 1;
        nextChannel(1);
    } else {
        initTuneRx(); // 
        protocolState = STATE_BIND_TUNING;
    }

    chVTSet(&timeout_vt, MS2ST(10), trigger_timeout_event, nullptr);
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
    chSysLockFromISR();
    chVTSetI(&timeout_vt, MS2ST(10), trigger_timeout_event, nullptr);
    chEvtSignalI(_irq_handler_ctx, EVT_TIMEOUT);
    chSysUnlockFromISR();
}

void AP_Radio_beken::start_recv_bind(void)
{
    protocolState = STATE_BIND_TUNING;
    chan_count = 0;
    packet_timer = AP_HAL::micros();
    chEvtSignal(_irq_handler_ctx, EVT_BIND);
    Debug(1,"Starting bind\n");
}

// handle a data96 mavlink packet for fw upload
void AP_Radio_beken::handle_data_packet(mavlink_channel_t chan, const mavlink_data96_t &m)
{
}

// main IRQ handler
void AP_Radio_beken::irq_handler(void)
{
    uint8_t ccLen = 32;
    //bool matched = false;
    //do {
    //    ccLen = cc2500.ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST);
    //    hal.scheduler->delay_microseconds(20);
    //    uint8_t ccLen2 = cc2500.ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST);
    //    matched = (ccLen == ccLen2);
    //} while (!matched);

    //if (ccLen & 0x80) {
    //    Debug(3,"Fifo overflow %02x\n", ccLen);
    //    // RX FIFO overflow
    //    cc2500.Strobe(CC2500_SFRX);
    //    cc2500.Strobe(CC2500_SRX);
    //    return;
    //}

    uint8_t packet[ccLen];
    beken.ReadFifo(packet, ccLen);

    if (get_fcc_test() != 0) {
        // don't process interrupts in FCCTEST mode
        return;
    }
    
#if 0
    if ((packet[ccLen-1] & 0x80) == 0) {
        // bad radio CRC
        return;
    }
#endif
    
#if 0
    static uint8_t counter;
    if (counter++ == 50) {
        Debug(2, "BEKEN IRQ state=%u\n", unsigned(protocolState));
        Debug(3,"len=%u\n", ccLen);
        for (uint8_t i=0; i<ccLen; i++) {
            Debug(4, "%02x:%02x ", i, packet[i]);
            if ((i+1) % 16 == 0) {
                Debug(4, "\n");
            }
        }
        if (ccLen % 16 != 0) {
            Debug(4, "\n");
        }
        counter = 0;
    }
#endif

    switch (protocolState) {
    case STATE_BIND_TUNING:
        if (tuneRx(ccLen, packet)) {
            Debug(2,"got BIND_TUNING\n");
            initGetBind();
            initialiseData(1);
            protocolState = STATE_BIND_BINDING1;
        }
        break;

    case STATE_BIND_BINDING1:
        if (getBind1(ccLen, packet)) {
            Debug(2,"got BIND1\n");
            protocolState = STATE_BIND_BINDING2;            
        }
        break;

    case STATE_BIND_BINDING2:
        if (getBind2(ccLen, packet)) {
            Debug(2,"got BIND2\n");
            protocolState = STATE_BIND_COMPLETE;
        }
        break;

    case STATE_BIND_COMPLETE:
        protocolState = STATE_STARTING;
        save_bind_info();
        Debug(3,"listLength=%u\n", listLength);
        Debug(3,"Saved bind info\n");
        break;

    case STATE_STARTING:
        listLength = 47;
        initialiseData(0);
        protocolState = STATE_SEARCH;
        chanskip = 1;
        nextChannel(1);
        break;

    case STATE_SEARCH:
        protocolState = STATE_DATA;
        // fallthrough

    case STATE_DATA: {
        if (packet[0] != 0x1D || ccLen != 32) {
            break;
        }
        if (!check_crc(ccLen, packet)) {
            Debug(3, "bad CRC\n");
            break;
        }
        if (packet[1] != bindTxId[0] ||
            packet[2] != bindTxId[1]) {
            Debug(3, "p1=%02x p2=%02x p6=%02x\n", packet[1], packet[2], packet[6]);
            // not for us
            break;
        }
        //if (packet[7] == 0x00 ||
        //    packet[7] == 0x20 ||
        //    packet[7] == 0x10 ||
        //    packet[7] == 0x12 ||
        //    packet[7] == 0x14 ||
        //    packet[7] == 0x16 ||
        //    packet[7] == 0x18 ||
        //    packet[7] == 0x1A ||
        //    packet[7] == 0x1C ||
        //    packet[7] == 0x1E) {
        //    // channel packet or range check packet
        //    parse_frSkyX(packet);
        //
        //    // get RSSI value from status byte
        //    uint8_t rssi_raw = packet[ccLen-2];
        //    float rssi_dbm;
        //    if (rssi_raw >= 128) {
        //        rssi_dbm = (rssi_raw - 256.0)/2;
        //    } else {
        //        rssi_dbm = rssi_raw*0.5;
        //    }
        //    rssi_filtered = 0.95 * rssi_filtered + 0.05 * rssi_dbm;
        //    t_status.rssi = uint8_t(rssi_filtered);
        //    
        //    stats.recv_packets++;
        //    uint8_t hop_chan = packet[4] & 0x3F;
        //    uint8_t skip = (packet[4]>>6) | (packet[5]<<2);
        //    if (channr != hop_chan) {
        //        Debug(4, "channr=%u hop_chan=%u\n", channr, hop_chan);
        //    }
        //    channr = hop_chan;
        //    if (chanskip != skip) {
        //        Debug(4, "chanskip=%u skip=%u\n", chanskip, skip);
        //    }
        //    chanskip = skip;
        //    packet_timer = irq_time_us;
        //    chVTSet(&timeout_vt, MS2ST(10), trigger_timeout_event, nullptr);
        //    
        //    packet3 = packet[3];
        //
        //    cc2500.Strobe(CC2500_SIDLE);
        //    cc2500.SetPower(get_transmit_power());
        //    send_telemetry();
        //
        //    // we can safely sleep here as we have a dedicated thread for radio processing.
        //    cc2500.unlock_bus();
        //    hal.scheduler->delay_microseconds(2800);
        //    cc2500.lock_bus();
        //
        //    nextChannel(chanskip);
        //    
        //} else {
        //    Debug(3, "p7=%02x\n", packet[7]);
        //}
        break;
    }

    case STATE_FCCTEST:
        // nothing to do, all done in timeout code
        Debug(3,"IRQ in FCCTEST state\n");
        break;

    default:
        Debug(2,"state %u\n", (unsigned)protocolState);
        break;
    }
}

// handle timeout IRQ
void AP_Radio_beken::irq_timeout(void)
{
    if (get_fcc_test() != 0 && protocolState != STATE_FCCTEST) {
        protocolState = STATE_FCCTEST;
        Debug(1,"Starting FCCTEST %u\n", get_fcc_test());
        setChannel(labs(get_fcc_test()) * 10);
        send_telemetry();
    }
    
    switch (protocolState) {
    case STATE_BIND_TUNING: {
        if (bindOffset >= 126) {
            bindOffset = -126;
        }
        uint32_t now = AP_HAL::millis();    
        if (now - timeTunedMs > 50) {
            timeTunedMs = now;
            bindOffset += 5;
            Debug(6,"bindOffset=%d\n", int(bindOffset));
            //cc2500.Strobe(CC2500_SIDLE);
            //cc2500.WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset);
            //cc2500.Strobe(CC2500_SFRX);
            //cc2500.Strobe(CC2500_SRX);
        }
        break;
    }
        
    case STATE_DATA: {
//        uint32_t now = AP_HAL::micros();
        
        //if (now - packet_timer > 50*sync_time_us) {
        //    Debug(3,"searching %u\n", now - packet_timer);
        //    cc2500.Strobe(CC2500_SIDLE);
        //    cc2500.Strobe(CC2500_SFRX);
        //    nextChannel(1);
        //    cc2500.Strobe(CC2500_SRX);
        //    timeouts++;
        //    protocolState = STATE_SEARCH;
        //}
        break;
    }

    case STATE_SEARCH:
        // shift by one channel at a time when searching
        nextChannel(1);
        break;
            
    case STATE_FCCTEST: {
        if (get_fcc_test() == 0) {
            protocolState = STATE_DATA;
            Debug(1,"Ending FCCTEST\n");
        }
        setChannel(labs(get_fcc_test()) * 10);
        send_telemetry();
        chVTSet(&timeout_vt, MS2ST(10), trigger_timeout_event, nullptr);
        break;
    }

    default:
        break;
    }
}

void AP_Radio_beken::irq_handler_thd(void *arg)
{
    (void)arg;
    while(true) {
        eventmask_t evt = chEvtWaitAny(ALL_EVENTS);

        radio_instance->beken.lock_bus();
        
        switch(evt) {
        case EVT_IRQ:
            if (radio_instance->protocolState == STATE_FCCTEST) {
                hal.console->printf("IRQ FCC\n");
            }
            radio_instance->irq_handler();
            break;
        case EVT_TIMEOUT:
            //if (radio_instance->beken.ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST)) {
            //    irq_time_us = AP_HAL::micros();
            //    radio_instance->irq_handler();
            //} else {
            //    radio_instance->irq_timeout();
            //}
            break;
        case EVT_BIND:
            radio_instance->initTuneRx();
            break;
        default:
            break;
        }

        radio_instance->beken.unlock_bus();
    }
}

void AP_Radio_beken::initTuneRx(void) // what do we do on this?
{
    //cc2500.WriteReg(CC2500_19_FOCCFG, 0x14);
    //timeTunedMs = AP_HAL::millis();
    //bindOffset = -126;
    //cc2500.WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset);
    //cc2500.WriteReg(CC2500_07_PKTCTRL1, 0x0C);
    //cc2500.WriteReg(CC2500_18_MCSM0, 0x8);
    //
    //cc2500.Strobe(CC2500_SIDLE);
    //cc2500.WriteReg(CC2500_23_FSCAL3, calData[0][0]);
    //cc2500.WriteReg(CC2500_24_FSCAL2, calData[0][1]);
    //cc2500.WriteReg(CC2500_25_FSCAL1, calData[0][2]);
    //cc2500.WriteReg(CC2500_0A_CHANNR, 0);
    //cc2500.Strobe(CC2500_SFRX);
    //cc2500.Strobe(CC2500_SRX);
}

void AP_Radio_beken::initialiseData(uint8_t adr)
{
    //cc2500.WriteReg(CC2500_0C_FSCTRL0, bindOffset);
    //cc2500.WriteReg(CC2500_18_MCSM0, 0x8);
    //cc2500.WriteReg(CC2500_09_ADDR, adr ? 0x03 : bindTxId[0]);
    //cc2500.WriteReg(CC2500_07_PKTCTRL1, 0x0D); // address check, no broadcast, autoflush, status enable
    //cc2500.WriteReg(CC2500_19_FOCCFG, 0x16);
    hal.scheduler->delay_microseconds(10*1000);
}

void AP_Radio_beken::initGetBind(void)
{
    //cc2500.Strobe(CC2500_SIDLE);
    //cc2500.WriteReg(CC2500_23_FSCAL3, calData[0][0]);
    //cc2500.WriteReg(CC2500_24_FSCAL2, calData[0][1]);
    //cc2500.WriteReg(CC2500_25_FSCAL1, calData[0][2]);
    //cc2500.WriteReg(CC2500_0A_CHANNR, 0);
    //cc2500.Strobe(CC2500_SFRX);
    hal.scheduler->delay_microseconds(20); // waiting flush FIFO

    //cc2500.Strobe(CC2500_SRX);
    listLength = 0;
    bindIdx = 0x05;
}

/*
  check if we have received a packet with sufficiently good link quality
 */
bool AP_Radio_beken::tuneRx(uint8_t ccLen, uint8_t *packet)
{
    if (bindOffset >= 126) {
        bindOffset = -126;
    }
    if ((packet[ccLen - 1] & 0x80) && packet[2] == 0x01) {
        uint8_t Lqi = packet[ccLen - 1] & 0x7F;
        //bool crc_ok = (packet[ccLen - 1] & 0x80) != 0;
        if (Lqi < 50) {
            return true;
        }
    }
    return false;
}

/*
  get a block of hopping data
 */
bool AP_Radio_beken::getBind1(uint8_t ccLen, uint8_t *packet)
{
    // len|bind |tx
    // id|03|01|idx|h0|h1|h2|h3|h4|00|00|00|00|00|00|00|00|00|00|00|00|00|00|00|CHK1|CHK2|RSSI|LQI/CRC|
    // Start by getting bind packet 0 and the txid
    if ((packet[ccLen - 1] & 0x80) && packet[2] == 0x01 && packet[5] == 0x00) {
        bindTxId[0] = packet[3];
        bindTxId[1] = packet[4];
        for (uint8_t n = 0; n < 5; n++) {
            uint8_t c = packet[5] + n;
            if (c < sizeof(bindHopData)) {
                bindHopData[c] = packet[6 + n];
            }
        }
        return true;
    }
    return false;
}

/*
  get a 2nd stage of hopping data
 */
bool AP_Radio_beken::getBind2(uint8_t ccLen, uint8_t *packet)
{
    if (bindIdx > 120) {
        return true;
    }
    if ((packet[ccLen - 1] & 0x80) &&
        packet[2] == 0x01 &&
        packet[3] == bindTxId[0] &&
        packet[4] == bindTxId[1] &&
        packet[5] == bindIdx) {
        for (uint8_t n = 0; n < 5; n++) {
            if (packet[6 + n] == packet[ccLen - 3] || (packet[6 + n] == 0)) {
                if (bindIdx >= 0x2D) {
                    listLength = packet[5] + n;
                    return true;
                }
            }
            uint8_t c = packet[5] + n;
            if (c < sizeof(bindHopData)) {
                bindHopData[c] = packet[6 + n];
            }
        }
        bindIdx = bindIdx + 5;
        return false;
    }
    return false;
}

void AP_Radio_beken::setChannel(uint8_t channel)
{
    //cc2500.Strobe(CC2500_SIDLE);
    //cc2500.WriteReg(CC2500_23_FSCAL3, calData[channel][0]);
    //cc2500.WriteReg(CC2500_24_FSCAL2, calData[channel][1]);
    //cc2500.WriteReg(CC2500_25_FSCAL1, calData[channel][2]);
    //cc2500.WriteReg(CC2500_0A_CHANNR, channel);
    //cc2500.Strobe(CC2500_SRX);
}

void AP_Radio_beken::nextChannel(uint8_t skip)
{
    channr = (channr + skip) % listLength;
    setChannel(bindHopData[channr]);
}

void AP_Radio_beken::parse_frSkyX(const uint8_t *packet)
{
    uint16_t c[8];

    c[0] = (uint16_t)((packet[10] <<8)& 0xF00) | packet[9];
    c[1] = (uint16_t)((packet[11]<<4)&0xFF0) | (packet[10]>>4);
    c[2] = (uint16_t)((packet[13] <<8)& 0xF00) | packet[12];
    c[3] = (uint16_t)((packet[14]<<4)&0xFF0) | (packet[13]>>4);
    c[4] = (uint16_t)((packet[16] <<8)& 0xF00) | packet[15];
    c[5] = (uint16_t)((packet[17]<<4)&0xFF0) | (packet[16]>>4);
    c[6] = (uint16_t)((packet[19] <<8)& 0xF00) | packet[18];
    c[7] = (uint16_t)((packet[20]<<4)&0xFF0) | (packet[19]>>4);

    uint8_t j;
    for (uint8_t i=0;i<8;i++) {
        if(c[i] > 2047)  {
            j = 8;
            c[i] = c[i] - 2048;
        } else {
            j = 0;
        }
        if (c[i] == 0) {
            continue;
        }
        uint16_t word_temp = (((c[i]-64)<<1)/3+860);
        if ((word_temp > 800) && (word_temp < 2200)) {
            uint8_t chan = i+j;
            if (chan < BEKEN_MAX_CHANNELS) {
                pwm_channels[chan] = word_temp;
                if (chan >= chan_count) {
                    chan_count = chan+1;
                }
            }
        }
    }
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
    info.bindTxId[0] = bindTxId[0];
    info.bindTxId[1] = bindTxId[1];
    info.bindOffset = bindOffset;
    info.listLength = listLength;
    memcpy(info.bindHopData, bindHopData, sizeof(info.bindHopData));
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
    
    bindTxId[0] = info.bindTxId[0];
    bindTxId[1] = info.bindTxId[1];
    bindOffset = info.bindOffset;
    listLength = info.listLength;
    memcpy(bindHopData, info.bindHopData, sizeof(bindHopData));

    return true;
}

/*
  send a telemetry packet
 */
void AP_Radio_beken::send_telemetry(void)
{
    uint8_t frame[15];

    memset(frame, 0, sizeof(frame));
    
    frame[0] = sizeof(frame)-1;
    frame[1] = bindTxId[0];
    frame[2] = bindTxId[1];
    frame[3] = packet3;
    if (telem_send_rssi) {
        frame[4] = (t_status.rssi*2) | 0x80;
    } else {
        frame[4] = uint8_t(hal.analogin->board_voltage() * 10) & 0x7F;
    }
    telem_send_rssi = !telem_send_rssi;
    
    uint16_t lcrc = calc_crc(&frame[3], 10);
    frame[13] = lcrc>>8;
    frame[14] = lcrc;

    //cc2500.Strobe(CC2500_SIDLE);
    //cc2500.Strobe(CC2500_SFTX);
    if (get_fcc_test() >= 0) {
        // in negative FCC test modes we don't write to the FIFO, which gives
        // continuous transmission
        //cc2500.WriteFifo(frame, sizeof(frame));
    }
    //cc2500.Strobe(CC2500_STX);
}

#endif // HAL_RCINPUT_WITH_AP_RADIO

