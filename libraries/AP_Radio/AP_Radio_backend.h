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
 * backend class for direct attached radios
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Radio.h"

class AP_Radio_backend
{
public:
    AP_Radio_backend(AP_Radio &radio);
    virtual ~AP_Radio_backend();
    
    // init - initialise radio
    virtual bool init(void) = 0;

    // init - reset radio
    virtual bool reset(void) = 0;
    
    // send a packet
    virtual bool send(const uint8_t *pkt, uint16_t len) = 0;

    // receive a packet
    virtual uint8_t recv(uint8_t *pkt, uint16_t len, uint32_t timeout_usec) = 0;

    // go to next channel
    virtual void next_channel(void) = 0;

    // start bind procedure
    virtual void start_bind(void) = 0;
    
private:
    AP_Radio &radio;
};
