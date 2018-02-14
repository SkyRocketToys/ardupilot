/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

/*
  AP_Radio implementation for CC2500 2.4GHz radio. 

  With thanks to cleanflight and betaflight projects
 */

#include "AP_Radio_backend.h"

#if defined(HAL_RCINPUT_WITH_AP_RADIO) && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412
#include "hal.h"
#include "telem_structure.h"
#include "driver_bk2425.h"

#define BEKEN_MAX_CHANNELS 16

// This helper struct estimates the times (in microseconds) between packets,
// according to the STM32 clock which may well be 2% different from the STM8 clock.
// For instance it may be 5108 instead of the nominal 5000 microseconds.
struct SyncTiming {
	enum { TARGET_DELTA_RX = 5000,               // Nominal 5ms between packets is expected
		SLOP_DELTA_RX = TARGET_DELTA_RX / 10,    // +/- 500us i.e. 10% skew each way is accepted.
		DIFF_DELTA_RX = TARGET_DELTA_RX / 100 }; // Two consequetive deltas must be very close together (50us)
    uint32_t packet_timer; // Time we last received a valid control packet
    uint32_t rx_time_us; // Time we last received a packet
    uint32_t tx_time_us; // Time we last finished transmitting a packet
    uint32_t delta_rx_time_us; // Time between last rx packets
    uint32_t last_delta_rx_time_us; // previous version of the delta
    uint32_t sync_time_us; // Estimate of base time in microseconds between packets. 5000 +/- 500
    SyncTiming() : // Constructor to setup sensible initial conditions
		delta_rx_time_us(TARGET_DELTA_RX),
		last_delta_rx_time_us(TARGET_DELTA_RX),
		sync_time_us(TARGET_DELTA_RX)
		{}
	void Rx(uint32_t when); // Adjust the timing based on a new packet
};

// Helper struct for synchronising channels when we change hopping table (e.g. learn of a WiFi channel change).
struct SyncChannel {
	enum { countdown_invalid = 0 }; // When countdown is this value, no change is pending
    uint8_t channel; // Index within the channel hopping sequence. Corresponds to txChannel on the button board
    uint8_t countdown; // How many packet slots until a pending table change occurs?
    uint8_t countdown_chan; // Which channel do we jump to when the table change happens?
    SyncChannel() : // Constructor to setup sensible initial conditions
		channel(0),
		countdown(countdown_invalid),
		countdown_chan(0)
		{}
    void SetChannel(uint8_t chan) { channel = chan; }
    void SetCountdown(uint8_t cnt, uint8_t nextCh) { countdown = cnt; countdown_chan = nextCh; }
    void NextChannel(void); // Step through the channels
    void SafeTable(void); // Give up on this WiFi table as packets have not been received
};

// Main class for receiving (and replying) to Beken radio packets
class AP_Radio_beken : public AP_Radio_backend
{
public:
	// Override base class functions
    AP_Radio_beken(AP_Radio &radio);
    bool init(void) override; // initialise the radio
    bool reset(void) override; // reset the radio
    bool send(const uint8_t *pkt, uint16_t len) override; // send a packet
    void start_recv_bind(void) override; // start bind process as a receiver
    uint32_t last_recv_us(void) override; // return time in microseconds of last received R/C packet
    uint8_t num_channels(void) override; // return number of input channels
    uint16_t read(uint8_t chan) override; // return current "PWM" (value) of a channel
    void handle_data_packet(mavlink_channel_t chan, const mavlink_data96_t &m) override; // handle a data96 mavlink packet for fw upload
    void update(void) override; // update status
    uint32_t get_tx_version(void) override { return 0; } // get TX fw version
    const AP_Radio::stats &get_stats(void) override; // get radio statistics structure

	// Extra public functions
    void set_wifi_channel(uint8_t channel) { t_status.wifi_chan = channel; } // set the 2.4GHz wifi channel used by companion computer, so it can be avoided
    
private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    
    // Static data, for interrupt support
    static AP_Radio_beken *radio_instance;
    static thread_t *_irq_handler_ctx;
    static virtual_timer_t timeout_vt;
    static uint32_t irq_time_us; // Time the Beken IRQ was last triggered, in the handler interrupts
    static uint32_t irq_when_us; // Time the Beken IRQ was last triggered, in the handler thread (copied from irq_time_us)
    static uint32_t last_timeout_us; // Time the timeout was last triggered (copied from irq_time_us via irq_when_us)
    static uint32_t next_timeout_us; // Time the next timeout is due to be triggered
    static uint32_t delta_timeout_us; // Desired delta between timeouts (1000us). Faster than the actual timing, since at the time we retrigger a timer interrupt we have insufficient information.
    static uint32_t next_switch_us; // Time when we next want to switch radio channels

	// Static functions, for interrupt support
    static void irq_handler_thd(void* arg);
    static void trigger_irq_radio_event(void);
    static void trigger_timeout_event(void *arg);

	//  Private functions
    void radio_init(void);
	void ProcessPacket(const uint8_t* packet, uint8_t rxaddr);
    void setChannel(uint8_t channel);
    void nextChannel(uint8_t skip);
    uint16_t calc_crc(uint8_t *data, uint8_t len);
    bool check_crc(uint8_t ccLen, uint8_t *packet);
    void send_telemetry(void);
    void irq_handler(uint32_t when);
    void irq_timeout(void);
    void save_bind_info(void);
    bool load_bind_info(void);
	void UpdateFccScan(void);
	void UpdateTxData(void);
	void map_stick_mode(void); // Support mode1,2,3,4 for stick mapping
	void update_SRT_telemetry(void);

    // semaphore between ISR and main thread
    AP_HAL::Semaphore *sem;    

    AP_Radio::stats stats;
    AP_Radio::stats last_stats;

    uint16_t pwm_channels[BEKEN_MAX_CHANNELS]; // Channel data
    uint8_t chan_count; // Number of valid channels

    Radio_Beken beken;

	SyncChannel syncch; // Index within the channel hopping sequence. Corresponds to txChannel on the button board
    uint32_t lost; // Number of packets we should have received but didnt?
    uint32_t timeouts;
    SyncTiming synctm; // Timing between packets, according to the local clock (not the tx clock).

    // bind structure saved to storage
    static const uint16_t bind_magic = 0x120a;
    struct PACKED bind_info {
        uint16_t magic;
        uint8_t bindTxId[5]; // The transmission address I last used
    };

    struct telem_status t_status; // Keep track of certain data that can be sent as telemetry to the tx.
    uint32_t last_pps_ms; // Timestamp of the last PPS (packets per second) calculation, in milliseconds.
    
    ITX_SPEED spd; // Speed of radio modulation.
    uint8_t myDroneId[4]; // CRC of the flight boards UUID, to inform the tx
};

#endif // HAL_RCINPUT_WITH_AP_RADIO
