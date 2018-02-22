#include <AP_Beacon/AP_Beacon.h>

#include <GCS_MAVLink/GCS.h>

#ifdef STUB_OUT_AP_Beacon

const AP_Param::GroupInfo AP_Beacon::var_info[] = {
    AP_GROUPEND
};

AP_Beacon::AP_Beacon(AP_SerialManager &_serial_manager) :
    serial_manager(_serial_manager)
{
}

uint8_t AP_Beacon::count() const
{
    return 0;
}

void AP_Beacon::update(void)
{
}

bool AP_Beacon::healthy(void)
{
    return false;
}

bool AP_Beacon::enabled(void)
{
    return false;
}

void AP_Beacon::init(void)
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "AP_Beacon has been stubbed out");
}

bool AP_Beacon::get_beacon_data(uint8_t beacon_instance, struct BeaconState& state) const
{
    return false;
}

bool AP_Beacon::beacon_healthy(uint8_t beacon_instance) const
{
    return false;
}


uint32_t AP_Beacon::beacon_last_update_ms(uint8_t beacon_instance) const
{
    return 0;
}

Vector3f AP_Beacon::beacon_position(uint8_t beacon_instance) const
{
    return {0,0,0};
}

float AP_Beacon::beacon_distance(uint8_t beacon_instance) const
{
    return 0.0f;
}

bool AP_Beacon::get_vehicle_position_ned(Vector3f &position, float& accuracy_estimate) const
{
    return false;
}

bool AP_Beacon::get_origin(Location &origin_loc) const
{
    return false;
}

const Vector2f* AP_Beacon::get_boundary_points(uint16_t& num_points) const
{
    return nullptr;
}

#endif
