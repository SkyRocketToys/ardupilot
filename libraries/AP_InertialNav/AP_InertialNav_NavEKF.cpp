#include <AP_HAL/AP_HAL.h>
#include <DataFlash/DataFlash.h>
#include "AP_InertialNav.h"

#if AP_AHRS_NAVEKF_AVAILABLE

extern const AP_HAL::HAL& hal;

/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */

/**
   update internal state
*/
void AP_InertialNav_NavEKF::update(float dt)
{
    // get the NE position relative to the local earth frame origin
    Vector2f posNE;
    if (_ahrs_ekf.get_relative_position_NE_origin(posNE)) {
        _relpos_cm.x = posNE.x * 100; // convert from m to cm
        _relpos_cm.y = posNE.y * 100; // convert from m to cm
    }

    // get the D position relative to the local earth frame origin
    float posD;
    if (_ahrs_ekf.get_relative_position_D_origin(posD)) {
        _relpos_cm.z = - posD * 100; // convert from m in NED to cm in NEU
    }

    // get the absolute WGS-84 position
    _haveabspos = _ahrs_ekf.get_position(_abspos);

    // get the velocity relative to the local earth frame
    Vector3f velNED;
    if (_ahrs_ekf.get_velocity_NED(velNED)) {
        _velocity_cm = velNED * 100; // convert to cm/s
        _velocity_cm.z = -_velocity_cm.z; // convert from NED to NEU
    }

    // Get a derivative of the vertical position which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
    if (_ahrs_ekf.get_vert_pos_rate(_pos_z_rate)) {
        _pos_z_rate *= 100; // convert to cm/s
        _pos_z_rate = - _pos_z_rate; // InertialNav is NEU
    }

    uint32_t now = AP_HAL::millis();
    if (!hal.util->get_soft_armed()) {
        _arm_time_ms = 0;
    } else if (_arm_time_ms == 0) {
        _arm_time_ms = now;
        _in_takeoff = true;
    }

    update_baro(dt);
    
    if (_ahrs_ekf.get_ekf_type() == 0) {
        _velocity_cm.z = _velocity_z;
        _relpos_cm.z = _position_z;
    }
}

/**
 * get_filter_status : returns filter status as a series of flags
 */
nav_filter_status AP_InertialNav_NavEKF::get_filter_status() const
{
    nav_filter_status status;
    _ahrs_ekf.get_filter_status(status);
    return status;
}

/**
 * get_origin - returns the inertial navigation origin in lat/lon/alt
 */
struct Location AP_InertialNav_NavEKF::get_origin() const
{
    struct Location ret;
     if (!_ahrs_ekf.get_origin(ret)) {
         // initialise location to all zeros if EKF origin not yet set
         memset(&ret, 0, sizeof(ret));
     }
    return ret;
}

/**
 * get_position - returns the current position relative to the home location in cm.
 *
 * @return
 */
const Vector3f &AP_InertialNav_NavEKF::get_position(void) const 
{
    return _relpos_cm;
}

/**
 * get_location - updates the provided location with the latest calculated location
 *  returns true on success (i.e. the EKF knows it's latest position), false on failure
 */
bool AP_InertialNav_NavEKF::get_location(struct Location &loc) const
{
    return _ahrs_ekf.get_location(loc);
}

/**
 * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 */
int32_t AP_InertialNav_NavEKF::get_latitude() const
{
    return _abspos.lat;
}

/**
 * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 * @return
 */
int32_t AP_InertialNav_NavEKF::get_longitude() const
{
    return _abspos.lng;
}

/**
 * get_velocity - returns the current velocity in cm/s
 *
 * @return velocity vector:
 *      		.x : latitude  velocity in cm/s
 * 				.y : longitude velocity in cm/s
 * 				.z : vertical  velocity in cm/s
 */
const Vector3f &AP_InertialNav_NavEKF::get_velocity() const
{
    return _velocity_cm;
}

/**
 * get_velocity_xy - returns the current horizontal velocity in cm/s
 *
 * @returns the current horizontal velocity in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_xy() const
{
    return norm(_velocity_cm.x, _velocity_cm.y);
}

/**
 * get_pos_z_derivative - returns the derivative of the z position in cm/s
*/
float AP_InertialNav_NavEKF::get_pos_z_derivative() const
{
    return _pos_z_rate;
}

/**
 * get_altitude - get latest altitude estimate in cm
 * @return
 */
float AP_InertialNav_NavEKF::get_altitude() const
{
    return _relpos_cm.z;
}

/**
 * getHgtAboveGnd - get latest height above ground level estimate in cm and a validity flag
 *
 * @return
 */
bool AP_InertialNav_NavEKF::get_hagl(float &height) const
{
    // true when estimate is valid
    bool valid = _ahrs_ekf.get_hagl(height);
    // convert height from m to cm
    height *= 100.0f;
    return valid;
}

/**
 * get_velocity_z - returns the current climbrate.
 *
 * @see get_velocity().z
 *
 * @return climbrate in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_z() const
{
    return _velocity_cm.z;
}

// check_baro - check if new baro readings have arrived and use them to correct vertical accelerometer offsets
void AP_InertialNav_NavEKF::check_baro()
{
    AP_Baro &_baro = *AP_Baro::get_instance();
    uint32_t baro_update_time;

    // calculate time since last baro reading (in ms)
    baro_update_time = _baro.get_last_update();
    if( baro_update_time != _baro_last_update ) {
        const float dt = (float)(baro_update_time - _baro_last_update) * 0.001f; // in seconds
        // call correction method
        correct_with_baro(_baro.get_altitude()*100.0f, dt);
        _baro_last_update = baro_update_time;
    }
}

// set_altitude - set base altitude estimate in cm
void AP_InertialNav_NavEKF::set_altitude( float new_altitude)
{
    _position_base_z = new_altitude;
    _position_correction_z = 0;
    _position_z = new_altitude; // _position = _position_base + _position_correction
    _hist_position_estimate_z.clear(); // reset z history to avoid fake z velocity at next baro calibration (next rearm)
}

// correct_with_baro - modifies accelerometer offsets using barometer.  dt is time since last baro reading
void AP_InertialNav_NavEKF::correct_with_baro(float baro_alt, float dt)
{
    static uint8_t first_reads = 0;

    // discard samples where dt is too large
    if( dt > 0.5f ) {
        return;
    }

    // discard first 10 reads but perform some initialisation
    if( first_reads <= 10 ) {
        set_altitude(baro_alt);
        first_reads++;
    }

    // 3rd order samples (i.e. position from baro) are delayed by 150ms (15 iterations at 100hz)
    // so we should calculate error using historical estimates
    float hist_position_base_z;
    if (_hist_position_estimate_z.is_full()) {
        hist_position_base_z = _hist_position_estimate_z.front();
    } else {
        hist_position_base_z = _position_base_z;
    }

    // calculate error in position from baro with our estimate
    _position_error_z = baro_alt - (hist_position_base_z + _position_correction_z);
}

// update_gains - update gains from time constant (given in seconds)
void AP_InertialNav_NavEKF::update_baro_gains()
{
    // Z axis time constant
    if (_time_constant_z <= 0.0f) {
        _k1_z = _k2_z = _k3_z = 0.0f;
    }else{
        _k1_z = 3.0f / _time_constant_z;
        _k2_z = 3.0f / (_time_constant_z*_time_constant_z);
        _k3_z = 1.0f / (_time_constant_z*_time_constant_z*_time_constant_z);
    }
}

// update gain constants
void AP_InertialNav_NavEKF::set_tc_z(float tc_z, float k1, float k2, float k3)
{
    _time_constant_z = tc_z;
    
    // Z axis time constant
    if (_time_constant_z <= 0.0f) {
        _k1_z = _k2_z = _k3_z = 0.0f;
    }else{
        _k1_z = k1 / _time_constant_z;
        _k2_z = k2 / (_time_constant_z*_time_constant_z);
        _k3_z = k3 / (_time_constant_z*_time_constant_z*_time_constant_z);
    }
}

/*
  update vertical velocity and position using barometer based data
 */
void AP_InertialNav_NavEKF::update_baro(float dt)
{
    // discard samples where dt is too large
    if( dt > 0.1f ) {
        return;
    }

    if (_k1_z <= 0) {
        update_baro_gains();
    }

    // check if new baro readings have arrived and use them to correct vertical accelerometer offsets.
    check_baro();

    float accel_ef_z = _ahrs_ekf.get_accel_ef().z;

    // remove influence of gravity
    accel_ef_z += GRAVITY_MSS;
    accel_ef_z *= 100.0f;

    //Convert North-East-Down to North-East-Up
    accel_ef_z = -accel_ef_z;

    accel_correction_z += _position_error_z * _k3_z  * dt;

    _velocity_z += _position_error_z * _k2_z  * dt;

    _position_correction_z += _position_error_z * _k1_z  * dt;

    // calculate velocity increase adding new acceleration from accelerometers
    float velocity_increase_z;
    velocity_increase_z = (accel_ef_z + accel_correction_z) * dt;

    // calculate new estimate of position
    _position_base_z += (_velocity_z + velocity_increase_z*0.5) * dt;

    // update the corrected position estimate
    _position_z = _position_base_z + _position_correction_z;

    // calculate new velocity
    _velocity_z += velocity_increase_z;

    // store 3rd order estimate (i.e. estimated vertical position) for future use
    _hist_position_estimate_z.push_back(_position_base_z);
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
