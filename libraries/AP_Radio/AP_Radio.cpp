#include <AP_HAL/AP_HAL.h>
#include "AP_Radio.h"
#include "AP_Radio_backend.h"
#include "AP_Radio_cypress.h"

extern const AP_HAL::HAL& hal;

bool AP_Radio::init(void)
{
    driver = new AP_Radio_cypress(*this);
    if (!driver) {
        return false;
    }
    return driver->init();
}

bool AP_Radio::reset(void)
{
    if (!driver) {
        return false;
    }
    return driver->reset();
}

bool AP_Radio::send(const uint8_t *pkt, uint16_t len)
{
    if (!driver) {
        return false;
    }
    return driver->send(pkt, len);
}

void AP_Radio::start_recv_bind(void)
{
    if (!driver) {
        return;
    }
    return driver->start_recv_bind();
}

const AP_Radio::stats &AP_Radio::get_stats(void)
{
    return driver->get_stats();
}

uint8_t AP_Radio::num_channels(void)
{
    if (!driver) {
        return 0;
    }
    return driver->num_channels();
}

uint16_t AP_Radio::read(uint8_t chan)
{
    if (!driver) {
        return 0;
    }
    return driver->read(chan);
}

uint32_t AP_Radio::last_recv_us(void)
{
    if (!driver) {
        return 0;
    }
    return driver->last_recv_us();
}
