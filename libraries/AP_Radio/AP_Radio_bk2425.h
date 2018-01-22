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
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <nuttx/arch.h>
#include <systemlib/systemlib.h>
#include <drivers/drv_hrt.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "hal.h"
#endif
#include "telem_structure.h"
#include "driver_bk2425.h"

#define BEKEN_MAX_CHANNELS 8

class AP_Radio_beken : public AP_Radio_backend
{
public:
    
    AP_Radio_beken(AP_Radio &radio);
    
    // init - initialise radio
    bool init(void) override;

    // rest radio
    bool reset(void) override;
    
    // send a packet
    bool send(const uint8_t *pkt, uint16_t len) override;

    // start bind process as a receiver
    void start_recv_bind(void) override;

    // return time in microseconds of last received R/C packet
    uint32_t last_recv_us(void) override;

    // return number of input channels
    uint8_t num_channels(void) override;

    // return current PWM of a channel
    uint16_t read(uint8_t chan) override;

    // handle a data96 mavlink packet for fw upload
    void handle_data_packet(mavlink_channel_t chan, const mavlink_data96_t &m) override;

    // update status
    void update(void) override;

    // get TX fw version
    uint32_t get_tx_version(void) override {
        return 0;
    }
    
    // get radio statistics structure
    const AP_Radio::stats &get_stats(void) override;

    // set the 2.4GHz wifi channel used by companion computer, so it can be avoided
    void set_wifi_channel(uint8_t channel) {
        // t_status.wifi_chan = channel;
    }
    

    
private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    static AP_Radio_beken *radio_instance;
    static thread_t *_irq_handler_ctx;
    static virtual_timer_t timeout_vt;

    static void irq_handler_thd(void* arg);
    static void trigger_irq_radio_event(void);
    static void trigger_timeout_event(void *arg);

    void radio_init(void);
	void ProcessPacket(const uint8_t* packet, uint8_t rxaddr);

    // semaphore between ISR and main thread
    AP_HAL::Semaphore *sem;    

    AP_Radio::stats stats;
    AP_Radio::stats last_stats;

    uint16_t pwm_channels[BEKEN_MAX_CHANNELS];

    Radio_Beken beken;

    uint8_t rxNum;
    uint8_t channr;
    uint8_t chanskip;
    uint32_t packet_timer;
    static uint32_t irq_time_us;
    const uint32_t sync_time_us = 9000;
    uint8_t chan_count;
    uint32_t lost;
    uint32_t timeouts;
    bool have_bind_info;
    uint8_t packet3;
    bool telem_send_rssi;
    float rssi_filtered;

    uint32_t timeTunedMs;

    void setChannel(uint8_t channel);
    void nextChannel(uint8_t skip);

    void parse_frSkyX(const uint8_t *packet);
    uint16_t calc_crc(uint8_t *data, uint8_t len);
    bool check_crc(uint8_t ccLen, uint8_t *packet);

    void send_telemetry(void);

    void irq_handler(void);
    void irq_timeout(void);

    // bind structure saved to storage
    static const uint16_t bind_magic = 0x120a;
    struct PACKED bind_info {
        uint16_t magic;
        uint8_t bindTxId[5];
    };
    
    void save_bind_info(void);
    bool load_bind_info(void);
    
    enum {
        STATE_INIT = 0,
        STATE_BIND,
        STATE_DATA,
        STATE_DFU,
        STATE_FCCTEST,
    } protocolState;

    struct config {
        uint8_t reg;
        uint8_t value;
    };
    static const config radio_config[];

    struct telem_status t_status;
    uint32_t last_pps_ms;
    
    ITX_SPEED spd;
};

// ----------------------------------------------------------------------------
// Packet format definition
// ----------------------------------------------------------------------------

/** The type of packets being sent between controller and drone */
enum BK_PKT_TYPE_E {
	BK_PKT_TYPE_INVALID      = 0,    ///< Invalid packet from empty packets or bad CRC
	BK_PKT_TYPE_CTRL_FOUND   = 0x10, ///< (Tx->Drone) User control - known receiver
	BK_PKT_TYPE_CTRL_LOST    = 0x11, ///< (Tx->Drone) User control - unknown receiver
	BK_PKT_TYPE_BIND         = 0x12, ///< (Tx->Drone) Tell drones this tx is broadcasting
	BK_PKT_TYPE_TELEMETRY    = 0x13, ///< (Drone->Tx) Send telemetry to tx
	BK_PKT_TYPE_DFU          = 0x14, ///< (Drone->Tx) Send new firmware to tx
};
typedef uint8_t BK_PKT_TYPE;


/** Data for packets that are not droneid packets
	Onair order = little-endian */
typedef struct packetDataDeviceCtrl_s {
	uint8_t throttle; ///< High 8 bits of the throttle joystick
	uint8_t roll; ///< High 8 bits of the roll joystick
	uint8_t pitch; ///< High 8 bits of the pitch joystick
	uint8_t yaw; ///< High 8 bits of the yaw joystick
	uint8_t lsb; ///< Low 2 bits of throttle, roll, pitch, yaw
	uint8_t buttons_held; ///< The buttons
	uint8_t buttons_toggled; ///< The buttons
	uint8_t data_type; ///< Type of extra data being sent
	uint8_t data_value_lo; ///< Value of extra data being sent
	uint8_t data_value_hi; ///< Value of extra data being sent
} packetDataDeviceCtrl;

enum { SZ_ADDRESS = 5 }; ///< Size of address for transmission packets (40 bits)
enum { SZ_CRC_GUID = 4 }; ///< Size of UUID for drone (32 bits)
enum { SZ_DFU = 16 }; ///< Size of DFU packets

/** Data for packets that are binding packets
	Onair order = little-endian */
typedef struct packetDataDeviceBind_s {
	uint8_t bind_address[SZ_ADDRESS]; ///< The address being used by control packets
	uint8_t hopping; ///< The hopping table in use for this connection
} packetDataDeviceBind;

/** Data structure for data packet transmitted from device (controller) to host (drone) */
typedef struct packetDataDevice_s {
	BK_PKT_TYPE packetType; ///< The packet type
	uint8_t channel; ///< Next channel I will broadcast on
	union packetDataDevice_u ///< The variant part of the packets
	{
		packetDataDeviceCtrl ctrl; ///< Control packets
		packetDataDeviceBind bind; ///< Binding packets
	} u;
} packetFormatTx;

/** Data structure for data packet transmitted from host (drone) to device (controller) */
typedef struct packetDataDrone_s {
	BK_PKT_TYPE packetType; ///< 0: The packet type
	uint8_t channel; ///< 1: Next channel I will broadcast on
	uint8_t wifi; ///< 2:
	uint8_t rssi; ///< 3:
	uint8_t droneid[SZ_CRC_GUID]; ///< 4...7:
	uint8_t mode; ///< 8:
	// Telemetry data (unspecified so far)
} packetFormatRx;

typedef struct packetDataDfu_s {
	BK_PKT_TYPE packetType; ///< 0: The packet type
	uint8_t channel; ///< 1: Next channel I will broadcast on
	uint8_t address_lo; ///< 2:
	uint8_t address_hi; ///< 3:
	uint8_t data[SZ_DFU]; ///< 4...19:
} packetFormatDfu;

