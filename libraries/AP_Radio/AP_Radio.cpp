#include <AP_HAL/AP_HAL.h>

#ifdef HAL_RCINPUT_WITH_AP_RADIO

#include "AP_Radio.h"
#include "AP_Radio_backend.h"
#include "AP_Radio_cypress.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Radio::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Set type of direct attached radio
    // @Description: This enables support for direct attached radio receivers
    // @Values: 0:None,1:CYRF6936
    // @User: Advanced
    AP_GROUPINFO("_TYPE",  1, AP_Radio, radio_type, RADIO_TYPE_NONE),

    // @Param: _PROT
    // @DisplayName: protocol
    // @Description: Select air protocol
    // @Values: 0:Auto,1:DSM2,2:DSMX
    // @User: Advanced
    AP_GROUPINFO("_PROT",  2, AP_Radio, protocol, PROTOCOL_AUTO),

    // @Param: _DEBUG
    // @DisplayName: debug level
    // @Description: radio debug level
    // @Range: 0 4
    // @User: Advanced
    AP_GROUPINFO("_DEBUG",  3, AP_Radio, debug_level, 0),

    // @Param: _DISCRC
    // @DisplayName: disable receive CRC
    // @Description: disable receive CRC (for debug)
    // @Values: 0:NotDisabled,1:Disabled
    // @User: Advanced
    AP_GROUPINFO("_DISCRC",  4, AP_Radio, disable_crc, 0),

    // @Param: _RSSICH
    // @DisplayName: RSSI value channel
    // @Description: Channel to show RSSI value, or zero for disabled
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("_RSSICH",  5, AP_Radio, rssi_chan, 0),

    // @Param: _RATECH
    // @DisplayName: Packet rate channel
    // @Description: Channel to show received packet rate, or zero for disabled
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("_RATECH",  6, AP_Radio, rate_chan, 0),

    // @Param: _TELEM
    // @DisplayName: Enable telemetry
    // @Description: If this is non-zero then telemetry packets will be sent over DSM
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("_TELEM",  7, AP_Radio, telem_enable, 0),

    // @Param: _TXPOW
    // @DisplayName: Transmit power
    // @Description: Set transmit power
    // @Range: 0 7
    // @User: Advanced
    AP_GROUPINFO("_TXPOW",  8, AP_Radio, transmit_power, 7),
    
    AP_GROUPEND
};

AP_Radio *AP_Radio::_instance;

// constructor
AP_Radio::AP_Radio(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_instance != nullptr) {
        AP_HAL::panic("Multiple AP_Radio declarations");
    }
    _instance = this;
}

bool AP_Radio::init(void)
{
    switch (radio_type) {
    case RADIO_TYPE_CYRF6936:
        driver = new AP_Radio_cypress(*this);
        break;
    default:
        break;
    }
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
#endif // HAL_RCINPUT_WITH_AP_RADIO

