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
    void flowhold_flow_to_angle(Vector2f &angle);

    AP_Float flow_max;
    AP_Float flow_speed;
    AC_PI_2D flow_pi_xy;
    AP_Float flow_filter_hz;
    AP_Int8  flow_min_quality;

    float quality_filtered;

    uint8_t log_counter;
    bool limited;
};
