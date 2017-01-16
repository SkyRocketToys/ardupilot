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

void AP_Radio::start_recv_bind(void)
{
    return driver->start_recv_bind();
}

const AP_Radio::stats &AP_Radio::get_stats(void)
{
    return driver->get_stats();
}

uint8_t AP_Radio::num_channels(void)
{
    return driver->num_channels();
}

uint16_t AP_Radio::read(uint8_t chan)
{
    return driver->read(chan);
}
