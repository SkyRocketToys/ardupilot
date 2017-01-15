/*
 *  DSM receiver example code for Cypress radio
 *
 * With thanks to the SuperBitRF project and their excellent cyrf6936 dongle:
 *  https://1bitsquared.com/products/superbit-usbrf-dongle
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Radio/AP_Radio.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Radio radio;

static bool do_bind = true;

void setup()
{
    hal.console->printf("RADIO init\n");
    hal.scheduler->delay(1000);
    hal.uartA->begin(115200);
    radio.init();
    if (do_bind) {
        radio.start_recv_bind();
    }
}

void loop()
{
    static AP_Radio::stats stats;
    hal.scheduler->delay(1000);

    AP_Radio::stats new_stats = radio.get_stats();
    printf("recv:%3u bad:%3u to:%3u re:%u N:%2u %4u %4u %4u %4u\n",
           new_stats.recv_packets - stats.recv_packets,
           new_stats.bad_packets - stats.bad_packets,
           new_stats.timeouts - stats.timeouts,
           new_stats.recv_errors - stats.recv_errors,
           radio.num_channels(),
           radio.read(0), radio.read(1), radio.read(2), radio.read(3));
    stats = new_stats;
}

AP_HAL_MAIN();
