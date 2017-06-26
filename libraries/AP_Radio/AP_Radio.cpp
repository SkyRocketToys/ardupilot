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

<<<<<<< HEAD
    // @Param: _SIGCH
    // @DisplayName: RSSI signal strength
    // @Description: Channel to show receive RSSI signal strength, or zero for disabled
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("_SIGCH",  5, AP_Radio, rssi_chan, 0),

    // @Param: _PPSCH
    // @DisplayName: Packet rate channel
    // @Description: Channel to show received packet-per-second rate, or zero for disabled
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("_PPSCH",  6, AP_Radio, pps_chan, 0),
=======
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
>>>>>>> 096e4bfdf23cd06c3908a3e9cdcef1cc00ae1caa

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

    // @Param: _FCCTST
    // @DisplayName: Put radio into FCC test mode
    // @Description: If this is enabled then the radio will continuously transmit as required for FCC testing. The transmit channel is set by the value of the parameter. The radio will not work for RC input while this is enabled
    // @Values: 0:Disabled,1:MinChannel,2:MidChannel,3:MaxChannel,4:MinChannelCW,5:MidChannelCW,6:MaxChannelCW
    // @User: Advanced
    AP_GROUPINFO("_FCCTST", 9, AP_Radio, fcc_test, 0),

    // @Param: _STKMD
    // @DisplayName: Stick input mode
    // @Description: This selects between different stick input modes. The default is mode2, which has throttle on the left stick and pitch on the right stick. You can instead set mode1, which has throttle on the right stick and pitch on the left stick.
    // @Values: 1:Mode1,2:Mode2
    // @User: Advanced
    AP_GROUPINFO("_STKMD", 10, AP_Radio, stick_mode, 2),

    // @Param: _TESTCH
    // @DisplayName: Set radio to factory test channel
    // @Description: This sets the radio to a fixed test channel for factory testing. Using a fixed channel avoids the need for binding in factory testing.
    // @Values: 0:Disabled,1:TestChan1,2:TestChan2,3:TestChan2,4:TestChan2
    // @User: Advanced
    AP_GROUPINFO("_TESTCH", 11, AP_Radio, factory_test, 0),    
<<<<<<< HEAD

    // @Param: _TSIGCH
    // @DisplayName: RSSI value channel for telemetry data on transmitter
    // @Description: Channel to show telemetry RSSI value as received by TX
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("_TSIGCH", 12, AP_Radio, tx_rssi_chan, 0),

    // @Param: _TPPSCH
    // @DisplayName: Telemetry PPS channel
    // @Description: Channel to show telemetry packets-per-second value, as received at TX
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("_TPPSCH", 13, AP_Radio, tx_pps_chan, 0),
=======
>>>>>>> 096e4bfdf23cd06c3908a3e9cdcef1cc00ae1caa
    
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

// handle a data96 mavlink packet for fw upload
void AP_Radio::handle_data_packet(mavlink_channel_t chan, const mavlink_data96_t &m)
{
    if (driver) {
        driver->handle_data_packet(chan, m);
    }
}

// update status, should be called from main thread
void AP_Radio::update(void)
{
    if (driver) {
        driver->update();
    }
}

// get transmitter firmware version
uint32_t AP_Radio::get_tx_version(void)
{
    if (driver) {
        return driver->get_tx_version();
    }
    return 0;
}

#endif // HAL_RCINPUT_WITH_AP_RADIO

