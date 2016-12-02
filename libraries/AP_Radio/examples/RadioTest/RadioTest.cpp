/*
 *  Example code for AP_Radio
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Radio/AP_Radio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Radio radio;

void setup()
{
    hal.console->printf("Radio test\n");

    radio.init();
}

void loop()
{
    radio.send((const uint8_t *)"hello", 5);
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
