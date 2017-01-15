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

class AP_Radio_backend;

class AP_Radio
{
public:
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
        uint32_t recv_packets;
        uint32_t lost_packets;
        uint32_t timeouts;
    };

    // get packet statistics
    const struct stats &get_stats(void);
    
private:
    AP_Radio_backend *driver;
};
