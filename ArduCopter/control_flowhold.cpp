#include "Copter.h"
#include <utility>


/*
  implement FLOWHOLD mode, for position hold using opttical flow
  without rangefinder
 */

const AP_Param::GroupInfo FlowHold::var_info[] = {
    // @Param: _XY_P
    // @DisplayName: Flow (horizontal) P gain
    // @Description: Flow (horizontal) P gain.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _XY_I
    // @DisplayName: Flow I gain
    // @Description: Flow I gain
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _XY_IMAX
    // @DisplayName: Flow integrator maximum
    // @Description: Flow integrator maximum
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cdeg
    // @User: Advanced
    AP_SUBGROUPINFO(flow_pi_xy, "_XY_",  1, FlowHold, AC_PI_2D),

    // @Param: _FLOW_MAX
    // @DisplayName: Flow Max
    // @Description: Controls maximum movement rate in FlowMode
    // @Range: 0.1 2.5
    // @User: Standard
    AP_GROUPINFO("_FLOW_MAX", 2, FlowHold, flow_max, 1),

    // @Param: _FILT_HZ
    // @DisplayName: Flow Filter Frequency
    // @Description: Filter frequency for flow data
    // @Range: 1 100
    // @User: Standard
    AP_GROUPINFO("_FILT_HZ", 3, FlowHold, flow_filter_hz, 10),

    // @Param: _MIN_QUAL
    // @DisplayName: Minimum flow quality
    // @Description: Minimum flow quality to use flow position hold
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_MIN_QUAL", 4, FlowHold, flow_min_quality, 10),
    
    AP_GROUPEND
};

// constructor
FlowHold::FlowHold(void) :
    flow_pi_xy(0.2, 0.15, 3000, 10, 0.0025)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// flowhold_init - initialise flowhold controller
bool FlowHold::init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Alt Hold if the Rotor Runup is not complete
    if (!ignore_checks && !motors->rotor_runup_complete()){
        return false;
    }
#endif

    if (!copter.optflow.enabled() || !copter.optflow.healthy()) {
        return false;
    }

    // initialize vertical speeds and leash lengths
    copter.pos_control->set_speed_z(-copter.g.pilot_velocity_z_max, copter.g.pilot_velocity_z_max);
    copter.pos_control->set_accel_z(copter.g.pilot_accel_z);

    // initialise position and desired velocity
    if (!copter.pos_control->is_active_z()) {
        copter.pos_control->set_alt_target_to_current_alt();
        copter.pos_control->set_desired_velocity_z(copter.inertial_nav.get_velocity_z());
    }

    flowhold.flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), flow_filter_hz.get());

    quality_filtered = 0;
    flow_pi_xy.reset_I();
    limited = false;
    
    // stop takeoff if running
    copter.takeoff_stop();

    flow_pi_xy.set_dt(1.0/copter.scheduler.get_loop_rate_hz());

    return true;
}

/*
  calculate desired attitude from flow sensor. Called when flow sensor is healthy
 */
void FlowHold::flowhold_flow_to_angle(Vector2f &bf_angles)
{
    // get pilot desired flow rates
    Vector2f target_flow;

    target_flow.x = -copter.channel_roll->get_control_in() * flow_max / 4500;
    target_flow.y = -copter.channel_pitch->get_control_in() * flow_max / 4500;

    // get corrected raw flow rate
    Vector2f raw_flow = copter.optflow.flowRate() - copter.optflow.bodyRate();

    // limit sensor flow, this prevents oscillation at low altitudes
    raw_flow.x = constrain_float(raw_flow.x, -flow_max, flow_max);
    raw_flow.y = constrain_float(raw_flow.y, -flow_max, flow_max);

    // filter the flow rate
    Vector2f sensor_flow = flowhold.flow_filter.apply(raw_flow);

    // get difference between measured and target flow in body frame
    Vector2f input_bf = sensor_flow - target_flow;
    
    // rotate controller input to earth frame
    Vector2f input_ef = copter.ahrs.rotate_body_to_earth2D(input_bf);
    
    // pass desired flow to controller
    flow_pi_xy.set_input(input_ef);

    // get earth frame controller attitude in centi-degrees
    Vector2f ef_output;
    if (limited) {
        ef_output = flow_pi_xy.get_p() + flow_pi_xy.get_i_shrink();
    } else {
        ef_output = flow_pi_xy.get_pi();
    }
    ef_output *= copter.aparm.angle_max;

    // convert to body frame
    bf_angles = copter.ahrs.rotate_earth_to_body2D(ef_output);

    // set limited flag to prevent integrator windup
    limited = fabsf(bf_angles.x) > copter.aparm.angle_max || fabsf(bf_angles.y) > copter.aparm.angle_max;

    // constrain to angle limit
    bf_angles.x = constrain_float(bf_angles.x, -copter.aparm.angle_max, copter.aparm.angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -copter.aparm.angle_max, copter.aparm.angle_max);

    if (log_counter++ % 20 == 0) {
        DataFlash_Class::instance()->Log_Write("FHLD", "TimeUS,SFx,SFy,TFx,TFy,Ax,Ay,Qual", "Qfffffff",
                                               AP_HAL::micros64(),
                                               sensor_flow.x, sensor_flow.y,
                                               target_flow.x, target_flow.y,
                                               bf_angles.x, bf_angles.y,
                                               quality_filtered);
    }
}

// flowhold_run - runs the flowhold controller
// should be called at 100hz or more
void FlowHold::run()
{
    FlowHoldModeState flowhold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    copter.pos_control->set_speed_z(-copter.g.pilot_velocity_z_max, copter.g.pilot_velocity_z_max);
    copter.pos_control->set_accel_z(copter.g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    copter.update_simple_mode();

    // check for filter change
    if (!is_equal(flowhold.flow_filter.get_cutoff_freq(), flow_filter_hz.get())) {
        flowhold.flow_filter.set_cutoff_frequency(flow_filter_hz.get());
    }

    // get pilot desired climb rate
    float target_climb_rate = copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -copter.g.pilot_velocity_z_max, copter.g.pilot_velocity_z_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = copter.get_pilot_desired_yaw_rate(copter.channel_yaw->get_control_in());
    
#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (copter.ap.land_complete && (target_climb_rate > 0.0f) && copter.motors->rotor_runup_complete());
#else
    bool takeoff_triggered = copter.ap.land_complete && (target_climb_rate > 0.0f);
#endif
    
    // FlowHold State Machine Determination
    if (!copter.motors->armed() || !copter.motors->get_interlock()) {
        flowhold_state = FlowHold_MotorStopped;
    } else if (copter.takeoff_state.running || takeoff_triggered) {
        flowhold_state = FlowHold_Takeoff;
    } else if (!copter.ap.auto_armed || copter.ap.land_complete) {
        flowhold_state = FlowHold_Landed;
    } else {
        flowhold_state = FlowHold_Flying;
    }

    if (copter.optflow.healthy()) {
        const float filter_constant = 0.95;
        quality_filtered = filter_constant * quality_filtered + (1-filter_constant) * copter.optflow.quality();
    } else {
        quality_filtered = 0;
    }
    
    Vector2f bf_angles;

    if (quality_filtered >= flow_min_quality) {
        // use flow when possible
        flowhold_flow_to_angle(bf_angles);
    } else {
        // revert to ALT_HOLD behaviour when flow unavailable
        copter.get_pilot_desired_lean_angles(copter.channel_roll->get_control_in(),
                                             copter.channel_pitch->get_control_in(),
                                             bf_angles.x, bf_angles.y,
                                             copter.attitude_control->get_althold_lean_angle_max());        
    }
            
    // Alt Hold State Machine
    switch (flowhold_state) {

    case FlowHold_MotorStopped:

        copter.motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x, bf_angles.y, target_yaw_rate, copter.get_smoothing_gain());
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        copter.pos_control->set_alt_target_from_climb_rate(-abs(copter.g.land_speed), G_Dt, false);
#else
        copter.attitude_control->reset_rate_controller_I_terms();
        copter.attitude_control->set_yaw_target_to_current_heading();
        copter.pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        flow_pi_xy.reset_I();
        copter.pos_control->update_z_controller();
        break;

    case FlowHold_Takeoff:
        // set motors to full range
        copter.motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!copter.takeoff_state.running) {
            copter.takeoff_timer_start(constrain_float(copter.g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            copter.set_land_complete(false);
            // clear i terms
            copter.set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        copter.takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = copter.get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x, bf_angles.y, target_yaw_rate, copter.get_smoothing_gain());

        // call position controller
        copter.pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, copter.G_Dt, false);
        copter.pos_control->add_takeoff_climb_rate(takeoff_climb_rate, copter.G_Dt);
        copter.pos_control->update_z_controller();
        break;

    case FlowHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            copter.motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            copter.motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

        copter.attitude_control->reset_rate_controller_I_terms();
        copter.attitude_control->set_yaw_target_to_current_heading();
        copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x, bf_angles.y, target_yaw_rate, copter.get_smoothing_gain());
        copter.pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        copter.pos_control->update_z_controller();
        break;

    case FlowHold_Flying:
        copter.motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(bf_angles.x, bf_angles.y, copter.aparm.angle_max);
#endif

        // call attitude controller
        copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x, bf_angles.y, target_yaw_rate, copter.get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (copter.rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = copter.get_surface_tracking_climb_rate(target_climb_rate, copter.pos_control->get_alt_target(), copter.G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = copter.get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        copter.pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, copter.G_Dt, false);
        copter.pos_control->update_z_controller();
        break;
    }
}
