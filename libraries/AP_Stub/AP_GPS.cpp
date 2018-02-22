#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>

#ifdef STUB_OUT_AP_GPS

const AP_Param::GroupInfo AP_GPS::var_info[] = {
    AP_GROUPEND
};

AP_GPS *AP_GPS::_singleton;

AP_GPS::AP_GPS()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_GPS must be singleton");
    }
    _singleton = this;
}

void AP_GPS::init(const AP_SerialManager& serial_manager)
{
}

uint8_t AP_GPS::num_sensors(void) const
{
    return 0;
}

void AP_GPS::Write_DataFlash_Log_Startup_messages()
{
}

void AP_GPS::update(void)
{
}

bool AP_GPS::speed_accuracy(uint8_t instance, float &sacc) const
{
    return false;
}

bool AP_GPS::is_healthy(uint8_t instance) const
{
    return false;
}

uint64_t AP_GPS::time_epoch_usec(uint8_t instance) const
{
    return 0;
}

bool AP_GPS::horizontal_accuracy(uint8_t instance, float &hacc) const
{
    return false;
}

bool AP_GPS::vertical_accuracy(uint8_t instance, float &vacc) const
{
    return false;
}

void AP_GPS::handle_msg(const mavlink_message_t *msg)
{
}

void AP_GPS::send_mavlink_gps_raw(mavlink_channel_t chan)
{
}

void AP_GPS::send_mavlink_gps2_raw(mavlink_channel_t chan)
{
}

void AP_GPS::send_mavlink_gps_rtk(mavlink_channel_t chan, uint8_t inst)
{
}

void AP_GPS::lock_port(uint8_t instance, bool lock)
{
}

bool AP_GPS::all_consistent(float &distance) const
{
    return true;
}

bool AP_GPS::blend_health_check() const
{
    return true;
}

uint8_t AP_GPS::first_unconfigured_gps(void) const
{
    return GPS_ALL_CONFIGURED;
}

void AP_GPS::broadcast_first_configuration_failure_reason(void) const
{
    gcs().send_text(MAV_SEVERITY_INFO, "GPS has been stubbed out");
}

bool AP_GPS::prepare_for_arming(void)
{
    return true;
}

Vector3f bogus_antenna_offset;
const Vector3f &AP_GPS::get_antenna_offset(uint8_t instance) const
{
    return bogus_antenna_offset;
}

bool AP_GPS::get_lag(uint8_t instance, float &lag_sec) const
{
    return false;
}

namespace AP {

AP_GPS &gps()
{
    return AP_GPS::gps();
}

};

#endif
