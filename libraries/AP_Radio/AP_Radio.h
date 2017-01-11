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

    // receive a packet
    uint8_t recv(uint8_t *pkt, uint16_t maxlen, uint32_t timeout_usec=0);

    // go to next channel
    void next_channel(void);
    
private:
    AP_Radio_backend *driver;
};
