/*
 *  DSM receiver example code for Cypress radio
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Radio/AP_Radio.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Radio radio;

void setup()
{
    printf("RADIO init\n");
    hal.scheduler->delay(1000);
    radio.init();
}

void loop()
{
    static uint32_t counter;
    static uint32_t last_print_ms;
    uint8_t pkt[16];
    
    uint8_t len = radio.recv(pkt, sizeof(pkt));
    if (len != sizeof(pkt)) {
        printf("len=%u size=%u\n", len, sizeof(pkt));
    } else {
        counter++;
    }
    uint32_t now = AP_HAL::millis();
    if (now - last_print_ms > 1000) {
        if (last_print_ms != 0) {
            printf("pps:%u counter=%u\n", (1000 * counter) / (last_print_ms - now), counter);
        }
        last_print_ms = now;
        counter = 0;
    }

    radio.next_channel();
}

AP_HAL_MAIN();
