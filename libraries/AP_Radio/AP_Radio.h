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
 * base class for direct attached radios
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_Radio_backend;

class AP_Radio
{
public:
    friend class AP_Radio_backend;
    
    // constructor
    AP_Radio(void);
    
    // init - initialise radio
    bool init(void);

    // reset the radio
    bool reset(void);
    
    // send a packet
    bool send(const uint8_t *pkt, uint16_t len);

    // start bind process as a receiver
    void start_recv_bind(void);

    // return time in microseconds of last received R/C packet
    uint32_t last_recv_us(void);

    // return number of input channels
    uint8_t num_channels(void);

    // return current PWM of a channel
    uint16_t read(uint8_t chan);

    struct stats {
        uint32_t bad_packets;
        uint32_t recv_errors;
        uint32_t recv_packets;
        uint32_t lost_packets;
        uint32_t timeouts;
    };

    enum ap_radio_type {
        RADIO_TYPE_NONE=0,
        RADIO_TYPE_CYRF6936=1,
    };
    
    enum ap_radio_protocol {
        PROTOCOL_AUTO=0,
        PROTOCOL_DSM2=1,
        PROTOCOL_DSMX=2,
    };
    
    // get packet statistics
    const struct stats &get_stats(void);

    static const struct AP_Param::GroupInfo var_info[];

    // get singleton instance
    static AP_Radio *instance(void) {
        return _instance;
    }
    
private:
    AP_Radio_backend *driver;

    AP_Int8 radio_type;
    AP_Int8 protocol;
    AP_Int8 debug_level;
    AP_Int8 disable_crc;
    AP_Int8 rssi_chan;
    AP_Int8 rate_chan;
    AP_Int8 telem_enable;
    
    static AP_Radio *_instance;
};
