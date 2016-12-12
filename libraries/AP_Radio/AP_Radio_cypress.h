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
 * AP_Radio implementation for Cypress 2.4GHz radio
 */

#include "AP_Radio_backend.h"
#include <nuttx/arch.h>
#include <systemlib/systemlib.h>

class AP_Radio_cypress : public AP_Radio_backend
{
public:
    AP_Radio_cypress(AP_Radio &radio);
    
    // init - initialise radio
    bool init(void) override;

    // rest radio
    bool reset(void) override;
    
    // send a packet
    bool send(const uint8_t *pkt, uint16_t len) override;

    // receive a packet
    uint8_t recv(uint8_t *pkt, uint16_t len) override;

private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    static AP_Radio_cypress *radio_instance;

    void radio_init(void);
    
    void dump_registers(uint8_t n);

    void force_initial_state(void);
    void set_channel(uint8_t channel);
    uint8_t read_status_debounced(uint8_t adr);
    
    uint8_t read_register(uint8_t reg);
    void write_register(uint8_t reg, uint8_t value);
    void write_multiple(uint8_t reg, uint8_t n, const uint8_t *data);

    void wait_irq(void);
    
    struct config {
        uint8_t reg;
        uint8_t value;
    };
    static const struct config radio_config[];

    static const uint8_t PnCode[];

    sem_t irq_sem;

    /*
      transmit a packet of length bytes, blocking until it is complete
     */
    bool streaming_transmit(const uint8_t *data, uint8_t length);    

    // receive a packet
    uint8_t streaming_receive(uint8_t *pkt, uint8_t len);
    
    void irq_handler(void);
    static int irq_trampoline(int irq, void *context);
};

