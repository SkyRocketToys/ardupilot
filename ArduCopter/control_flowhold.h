#pragma once

/*
  class to support FLOWHOLD mode, which is a position hold mode using
  optical flow directly, avoiding the need for a rangefinder
 */

class FlowHold
{
public:
    friend class Copter;

    FlowHold();
    
    bool init(bool ignore_checks);
    void run(void);

    static const struct AP_Param::GroupInfo var_info[];    
private:

    // calculate attitude from flow data
    void flow_to_angle(Vector2f &bf_angle);

    // flowhold mode
    struct {
        LowPassFilterVector2f flow_filter;
    } flowhold;

    bool flowhold_init(bool ignore_checks);
    void flowhold_run();
    void flowhold_flow_to_angle(Vector2f &angle, const Vector2f &flow_gain);
    void update_height_estimate(void);

    // minimum assumed height
    const float height_min = 0.1;

    // maximum scaling height
    const float height_max = 3.0;
    
    AP_Float flow_max;
    AP_Float flow_speed;
    AC_PI_2D flow_pi_xy;
    AP_Float flow_filter_hz;
    AP_Int8  flow_min_quality;

    float quality_filtered;

    uint8_t log_counter;
    bool limited;

    // accumulated INS delta velocity in north-east form since last flow update
    Vector2f delta_velocity_ne;

    // last flow rate in radians/sec in north-east axis
    Vector2f last_flow_rate_rps_ne;
    
    // timestamp of last flow data
    uint32_t last_flow_ms;

    float last_ins_height;
    float height_estimate;
};
