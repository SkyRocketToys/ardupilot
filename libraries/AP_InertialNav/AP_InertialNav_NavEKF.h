/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */
#pragma once

#include <AP_NavEKF/AP_Nav_Common.h>              // definitions shared by inertial and ekf nav filters

class AP_InertialNav_NavEKF : public AP_InertialNav
{
public:
    // Constructor
    AP_InertialNav_NavEKF(AP_AHRS_NavEKF &ahrs) :
        AP_InertialNav(),
        _haveabspos(false),
        _ahrs_ekf(ahrs)
        {}

    /**
       update internal state
    */
    void        update(float dt);

    /**
     * get_filter_status - returns filter status as a series of flags
     */
    nav_filter_status get_filter_status() const;

    /**
     * get_origin - returns the inertial navigation origin in lat/lon/alt
     *
     * @return origin Location
     */
    struct Location get_origin() const;

    /**
     * get_position - returns the current position relative to the home location in cm.
     *
     * the home location was set with AP_InertialNav::set_home_position(int32_t, int32_t)
     *
     * @return
     */
    const Vector3f&    get_position() const;

    /**
     * get_llh - updates the provided location with the latest calculated location including absolute altitude
     *  returns true on success (i.e. the EKF knows it's latest position), false on failure
     */
    bool get_location(struct Location &loc) const;

    /**
     * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     */
    int32_t     get_latitude() const;

    /**
     * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @return
     */
    int32_t     get_longitude() const;

    /**
     * get_velocity - returns the current velocity in cm/s
     *
     * @return velocity vector:
     *      		.x : latitude  velocity in cm/s
     * 				.y : longitude velocity in cm/s
     * 				.z : vertical  velocity in cm/s
     */
    const Vector3f&    get_velocity() const;

    /**
     * get_pos_z_derivative - returns the derivative of the z position in cm/s
    */
    float    get_pos_z_derivative() const;

    /**
     * get_velocity_xy - returns the current horizontal velocity in cm/s
     *
     * @returns the current horizontal velocity in cm/s
     */
    float        get_velocity_xy() const;

    /**
     * get_altitude - get latest altitude estimate in cm
     * @return
     */
    float       get_altitude() const;

    /**
     * getHgtAboveGnd - get latest altitude estimate above ground level in centimetres and validity flag
     * @return
     */
    bool       get_hagl(float &hagl) const;

    /**
     * get_velocity_z - returns the current climbrate.
     *
     * @see get_velocity().z
     *
     * @return climbrate in cm/s
     */
    float       get_velocity_z() const;

    void set_tc_z(bool enable, float tc_z, float k1, float k2, float k3);
    
private:
    Vector3f _relpos_cm;   // NEU
    Vector3f _velocity_cm; // NEU
    float _pos_z_rate;
    float _vel_z_filter;
    float _pos_z_filter;
    float _pos_low_limit;
    uint32_t _arm_time_ms;
    bool _in_takeoff;
    struct Location _abspos;
    bool _haveabspos;
    AP_AHRS_NavEKF &_ahrs_ekf;

    void check_baro();
    void set_altitude( float new_altitude);
    void correct_with_baro(float baro_alt, float dt);
    void update_baro_gains();
    void update_baro(float dt);

    bool _inav_baro_enabled;
    float _position_error_z;
    float accel_correction_z;
    float _time_constant_z = 1.0;
    float _k1_z, _k2_z, _k3_z;
    float _velocity_z;
    uint32_t _baro_last_update;
    float _position_base_z;
    float _position_correction_z;
    float _position_z;
    AP_BufferFloat_Size15 _hist_position_estimate_z;
};
