#include <AP_HAL/AP_HAL.h>
#include "AP_Radio_cypress.h"
#include <utility>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

AP_Radio_cypress::AP_Radio_cypress(AP_Radio &_radio) :
    AP_Radio_backend(_radio)
{
}

bool AP_Radio_cypress::init(void)
{
    dev = std::move(hal.spi->get_device("external0m3"));
    return true;
}

bool AP_Radio_cypress::send(const uint8_t *pkt, uint16_t len)
{
    printf("Sending '%*.*s'\n", (int)len, (int)len, (const char *)pkt);
    return true;
}
