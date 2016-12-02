#include <AP_HAL/AP_HAL.h>
#include "AP_Radio.h"
#include "AP_Radio_backend.h"
#include "AP_Radio_cypress.h"

extern const AP_HAL::HAL& hal;

void AP_Radio::init(void)
{
    driver = new AP_Radio_cypress(*this);
    driver->init();
}

bool AP_Radio::send(const uint8_t *pkt, uint16_t len)
{
    return driver->send(pkt, len);
}
