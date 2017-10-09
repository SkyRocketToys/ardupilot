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
    // @Description: Controls maximum apparent flow rate in flowhold
    // @Range: 0.1 2.5
    // @User: Standard
    AP_GROUPINFO("_FLOW_MAX", 2, FlowHold, flow_max, 0.6),

    // @Param: _FILT_HZ
    // @DisplayName: Flow Filter Frequency
    // @Description: Filter frequency for flow data
    // @Range: 1 100
    // @User: Standard
    AP_GROUPINFO("_FILT_HZ", 3, FlowHold, flow_filter_hz, 5),

    // @Param: _MIN_QUAL
    // @DisplayName: Minimum flow quality
    // @Description: Minimum flow quality to use flow position hold
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_MIN_QUAL", 4, FlowHold, flow_min_quality, 10),

    // @Param: _FLOW_SPEED
    // @DisplayName: Flow Speed
    // @Description: Controls maximum pilot speed in FlowMode
    // @Range: 0.1 2.5
    // @User: Standard
    AP_GROUPINFO("_FLOW_SPEED", 5, FlowHold, flow_speed, 1.5),
    
    AP_GROUPEND
};

// constructor
FlowHold::FlowHold(void) :
    flow_pi_xy(0.2, 0.3, 3000, 5, 0.0025)
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

    // start with INS height
    last_ins_height = copter.inertial_nav.get_altitude() * 0.01 - copter.arming_altitude_m;
    height_offset = 0;
    
    return true;
}

/*
  calculate desired attitude from flow sensor. Called when flow sensor is healthy
 */
void FlowHold::flowhold_flow_to_angle(Vector2f &bf_angles, const Vector2f &flow_gain)
{
    // get pilot desired flow rates
    Vector2f target_flow;

    target_flow.x = -copter.channel_roll->get_control_in() * flow_speed / 4500;
    target_flow.y = -copter.channel_pitch->get_control_in() * flow_speed / 4500;

    // get corrected raw flow rate
    Vector2f raw_flow = copter.optflow.flowRate() - copter.optflow.bodyRate();

    // limit sensor flow, this prevents oscillation at low altitudes
    raw_flow.x = constrain_float(raw_flow.x, -flow_max, flow_max);
    raw_flow.y = constrain_float(raw_flow.y, -flow_max, flow_max);

    // filter the flow rate
    Vector2f sensor_flow = flowhold.flow_filter.apply(raw_flow);

    // scale by height estimate, limiting it to height_min to height_max
    float ins_height = copter.inertial_nav.get_altitude() * 0.01 - copter.arming_altitude_m;
    float height_estimate = ins_height + height_offset;
    sensor_flow *= constrain_float(height_estimate, height_min, height_max);
                    
    // get difference between measured and target flow in body frame
    Vector2f input_bf = sensor_flow - target_flow;
    
    // rotate controller input to earth frame
    Vector2f input_ef = copter.ahrs.rotate_body_to_earth2D(input_bf);

    // apply flow gains. This reduces influence of flow as pilot input increases
    input_ef.x *= flow_gain.x;
    input_ef.y *= flow_gain.y;

    // freeze I when we have pilot input
    bool freeze_I = (flow_gain.x < 0.95 || flow_gain.y < 0.95);

    // run PI controller
    flow_pi_xy.set_input(input_ef);

    // get earth frame controller attitude in centi-degrees
    Vector2f ef_output;

    // get P term
    ef_output = flow_pi_xy.get_p();

    // get I term
    if (!freeze_I) {
        if (limited) {
            // only allow I term to shrink in length
            xy_I = flow_pi_xy.get_i_shrink();
        } else {
            // normal I term operation
            xy_I = flow_pi_xy.get_pi();
        }
    }
    
    ef_output += xy_I;
    
    ef_output *= copter.aparm.angle_max;

    // convert to body frame
    bf_angles = copter.ahrs.rotate_earth_to_body2D(ef_output);

    // set limited flag to prevent integrator windup
    limited = fabsf(bf_angles.x) > copter.aparm.angle_max || fabsf(bf_angles.y) > copter.aparm.angle_max;

    // constrain to angle limit
    bf_angles.x = constrain_float(bf_angles.x, -copter.aparm.angle_max, copter.aparm.angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -copter.aparm.angle_max, copter.aparm.angle_max);

    if (log_counter++ % 20 == 0) {
        DataFlash_Class::instance()->Log_Write("FHLD", "TimeUS,SFx,SFy,TFx,TFy,Ax,Ay,Qual,Ix,Iy", "Qfffffffff",
                                               AP_HAL::micros64(),
                                               sensor_flow.x, sensor_flow.y,
                                               target_flow.x, target_flow.y,
                                               bf_angles.x, bf_angles.y,
                                               quality_filtered,
                                               xy_I.x, xy_I.y);
    }
}

// flowhold_run - runs the flowhold controller
// should be called at 100hz or more
void FlowHold::run()
{
    FlowHoldModeState flowhold_state;
    float takeoff_climb_rate = 0.0f;

    update_height_estimate();
    
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

    // calculate alt-hold angles
    float angle_max = copter.attitude_control->get_althold_lean_angle_max();
    copter.get_pilot_desired_lean_angles(copter.channel_roll->get_control_in(),
                                         copter.channel_pitch->get_control_in(),
                                         bf_angles.x, bf_angles.y,
                                         angle_max);
    
    if (quality_filtered >= flow_min_quality &&
        AP_HAL::millis() - copter.arm_time_ms > 3000) {
        // don't use for first 3s when we are just taking off
        Vector2f flow_angles;

        // reduce flow influence as pilot input increases
        Vector2f flow_gain;
        flow_gain.x = linear_interpolate(1, 0,
                                         fabsf(bf_angles.x),
                                         0, angle_max/2);
        flow_gain.y = linear_interpolate(1, 0,
                                         fabsf(bf_angles.y),
                                         0, angle_max/2);

        flowhold_flow_to_angle(flow_angles, flow_gain);
        flow_angles.x = constrain_float(flow_angles.x, -angle_max/2, angle_max/2);
        flow_angles.y = constrain_float(flow_angles.y, -angle_max/2, angle_max/2);
        bf_angles += flow_angles;
    }
    bf_angles.x = constrain_float(bf_angles.x, -angle_max, angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -angle_max, angle_max);
            
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


/*
  update height estimate using integrated accelerometer ratio with optical flow
 */
void FlowHold::update_height_estimate(void)
{
    float ins_height = copter.inertial_nav.get_altitude() * 0.01 - copter.arming_altitude_m;

#if 1
    // assume on ground when disarmed, or if we have only just started spooling the motors up
    if (!hal.util->get_soft_armed() ||
        copter.motors->get_desired_spool_state() != AP_Motors::DESIRED_THROTTLE_UNLIMITED ||
        AP_HAL::millis() - copter.arm_time_ms < 1500) {
        height_offset = -ins_height;
        last_ins_height = ins_height;
        return;
    }
#endif
        
    // get delta velocity in body frame
    Vector3f delta_vel;
    if (!copter.ins.get_delta_velocity(delta_vel)) {
        return;
    }

    // integrate delta velocity in earth frame
    const Matrix3f &rotMat = copter.ahrs.get_rotation_body_to_ned();
    delta_vel = rotMat * delta_vel;
    delta_velocity_ne.x += delta_vel.x;
    delta_velocity_ne.y += delta_vel.y;
    
    if (!copter.optflow.healthy()) {
        // can't update height model with no flow sensor
        last_flow_ms = AP_HAL::millis();
        delta_velocity_ne.zero();
        return;
    }

    if (last_flow_ms == 0) {
        // just starting up
        last_flow_ms = copter.optflow.last_update();
        delta_velocity_ne.zero();
        height_offset = 0;
        return;
    }
    
    if (copter.optflow.last_update() == last_flow_ms) {
        // no new flow data
        return;
    }

    // convert delta velocity back to body frame to match the flow sensor
    Vector2f delta_vel_bf = copter.ahrs.rotate_earth_to_body2D(delta_velocity_ne);

    // and convert to an rate equivalent, to be comparable to flow
    Vector2f delta_vel_rate(-delta_vel_bf.y, delta_vel_bf.x);
    
    // get body flow rate in radians per second
    Vector2f flow_rate_rps = copter.optflow.flowRate() - copter.optflow.bodyRate();

    uint32_t dt_ms = copter.optflow.last_update() - last_flow_ms;
    if (dt_ms > 500) {
        // too long between updates, ignore
        last_flow_ms = copter.optflow.last_update();
        delta_velocity_ne.zero();
        last_flow_rate_rps = flow_rate_rps;
        last_ins_height = ins_height;
        height_offset = 0;
        return;        
    }

    /*
      basic equation is:
      height_m = delta_velocity_mps / delta_flowrate_rps;
     */
    
    // get delta_flowrate_rps
    Vector2f delta_flowrate = flow_rate_rps - last_flow_rate_rps;
    last_flow_rate_rps = flow_rate_rps;
    last_flow_ms = copter.optflow.last_update();

    /*
      update height estimate
     */
    const float min_velocity_change = 0.04;
    const float min_flow_change = 0.04;
    const float height_delta_max = 0.25;

    /*
      for each axis update the height estimate
     */
    float delta_height = 0;
    uint8_t total_weight=0;
    float height_estimate = ins_height + height_offset;

    for (uint8_t i=0; i<2; i++) {
        // only use height estimates when we have significant delta-velocity and significant delta-flow
        float abs_flow = fabsf(delta_flowrate[i]);
        if (abs_flow < min_flow_change ||
            fabsf(delta_vel_rate[i]) < min_velocity_change) {
            continue;
        }
        // get instantaneous height estimate
        float height = delta_vel_rate[i] / delta_flowrate[i];
        if (height <= 0) {
            // discard negative heights
            continue;
        }
        delta_height += (height - height_estimate) * abs_flow;
        total_weight += abs_flow;
    }
    if (total_weight > 0) {
        delta_height /= total_weight;
    }
    
    if (delta_height < 0) {
        // bias towards lower heights, as we'd rather have too low
        // gain than have oscillation. This also compensates a bit for
        // the discard of negative heights above
        delta_height *= 2;
    }

    // don't update height by more than height_delta_max, this is a simple way of rejecting noise
    float new_offset = height_offset + constrain_float(delta_height, -height_delta_max, height_delta_max);

    // apply a simple filter
    height_offset = 0.8 * height_offset + 0.2 * new_offset;
    
    if (ins_height + height_offset < height_min) {
        // height estimate is never allowed below the minimum
        height_offset = height_min - ins_height;
    }

    // new height estimate for logging
    height_estimate = ins_height + height_offset;
    
    DataFlash_Class::instance()->Log_Write("FXY", "TimeUS,DFx,DFy,DVx,DVy,Hest,DH,Hofs,InsH,LastInsH,DTms", "QfffffffffI",
                                           AP_HAL::micros64(),
                                           delta_flowrate.x,
                                           delta_flowrate.y,
                                           delta_vel_rate.x,
                                           delta_vel_rate.y,
                                           height_estimate,
                                           delta_height,
                                           height_offset,
                                           ins_height,
                                           last_ins_height,
                                           dt_ms);
    mavlink_msg_named_value_float_send(MAVLINK_COMM_0, AP_HAL::millis(), "HEST", height_estimate);
    mavlink_msg_named_value_float_send(MAVLINK_COMM_1, AP_HAL::millis(), "HEST", height_estimate);
    delta_velocity_ne.zero();
    last_ins_height = ins_height;
}
