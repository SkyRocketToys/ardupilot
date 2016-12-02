#include "AP_Radio.h"
#include <utility>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

void AP_Radio::init(void)
{
    dev = std::move(hal.spi->get_device("external0m3"));    
}

bool AP_Radio::send(const uint8_t *pkt, uint16_t len)
{
    printf("Sending '%*.*s'\n", (int)len, (int)len, (const char *)pkt);
    return true;
}
