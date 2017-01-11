#include <AP_HAL/AP_HAL.h>
#include "AP_Radio.h"
#include "AP_Radio_backend.h"
#include "AP_Radio_cypress.h"

extern const AP_HAL::HAL& hal;

bool AP_Radio::init(void)
{
    driver = new AP_Radio_cypress(*this);
    return driver->init();
}

bool AP_Radio::reset(void)
{
    return driver->reset();
}

bool AP_Radio::send(const uint8_t *pkt, uint16_t len)
{
    return driver->send(pkt, len);
}

uint8_t AP_Radio::recv(uint8_t *pkt, uint16_t len, uint32_t timeout_usec)
{
    return driver->recv(pkt, len, timeout_usec);
}

void AP_Radio::next_channel(void)
{
    return driver->next_channel();
}
