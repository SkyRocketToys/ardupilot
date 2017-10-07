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
    AP_GROUPINFO("_FLOW_MAX", 2, FlowHold, flow_max, 0.3),

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
    flow_pi_xy(0.35, 0.6, 3000, 5, 0.0025)
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
    height_estimate = constrain_float(last_ins_height, height_min, 100);
    
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
    sensor_flow *= constrain_float(height_estimate, height_min, height_max);
                    
    // get difference between measured and target flow in body frame
    Vector2f input_bf = sensor_flow - target_flow;
    
    // rotate controller input to earth frame
    Vector2f input_ef = copter.ahrs.rotate_body_to_earth2D(input_bf);

    // apply flow gains. This reduces influence of flow as pilot input increases
    input_ef.x *= flow_gain.x;
    input_ef.y *= flow_gain.y;
    
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

    // assume on ground when disarmed, or if we have only just started spooling the motors up
    if (!hal.util->get_soft_armed() ||
        copter.motors->get_desired_spool_state() != AP_Motors::DESIRED_THROTTLE_UNLIMITED ||
        AP_HAL::millis() - copter.arm_time_ms < 1500) {
        height_estimate = 0;
        last_ins_height = ins_height;
        return;
    }
        
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
        return;
    }
    
    if (copter.optflow.last_update() == last_flow_ms) {
        // no new flow data
        return;
    }

    // get body flow rate in radians per second
    Vector2f raw_flow_bf_rps = copter.optflow.flowRate() - copter.optflow.bodyRate();

    // convert to earth rate
    Vector2f flow_rate_rps_ef = copter.ahrs.rotate_body_to_earth2D(raw_flow_bf_rps);

    // convert to north-east
    Vector2f flow_rate_rps_ne(flow_rate_rps_ef.y, -flow_rate_rps_ef.x);

    uint32_t dt_ms = copter.optflow.last_update() - last_flow_ms;
    if (dt_ms > 500) {
        // too long between updates, ignore
        last_flow_ms = copter.optflow.last_update();
        delta_velocity_ne.zero();
        last_flow_rate_rps_ne = flow_rate_rps_ne;
        last_ins_height = ins_height;
        return;        
    }

    /*
      basic equation is:
      height_m = delta_velocity_mps / delta_flowrate_rps;
     */
    
    // get delta_flowrate_rps
    Vector2f delta_flowrate_ne = flow_rate_rps_ne - last_flow_rate_rps_ne;
    last_flow_rate_rps_ne = flow_rate_rps_ne;
    last_flow_ms = copter.optflow.last_update();

    /*
      update height estimate
     */
    const float min_velocity_change = 0.05;
    const float min_flow_change = 0.05;
    const float height_delta_max = 0.1;

    height_estimate += ins_height - last_ins_height;
    if (height_estimate < height_min) {
        height_estimate = height_min;
    }

    /*
      for each axis update the height estimate
     */
    float delta_height = 0;
    for (uint8_t i=0; i<2; i++) {
        // only use height estimates when we have significant delta-velocity and significant delta-flow
        if (fabsf(delta_flowrate_ne[i]) < min_flow_change ||
            fabsf(delta_velocity_ne[i]) < min_velocity_change) {
            continue;
        }
        // get instantaneous height estimate
        float height = delta_velocity_ne[i] / delta_flowrate_ne[i];
        if (height <= 0) {
            // discard negative heights
            continue;
        }
        delta_height += height - height_estimate;
    }
    if (delta_height < 0) {
        // bias towards lower heights, as we'd rather have too low
        // gain than have oscillation
        delta_height *= 1.5;
    }
    // don't update height by more than height_delta_max, this is a simple way of rejecting noise
    height_estimate += constrain_float(delta_height, -height_delta_max, height_delta_max);
    if (height_estimate < height_min) {
        // height estimate is never allowed below the minimum
        height_estimate = height_min;
    }
    
    DataFlash_Class::instance()->Log_Write("FXY", "TimeUS,DFn,DFe,DVn,DVe,Hest,DH,InsH,LastInsH,DTms", "QffffffffI",
                                           AP_HAL::micros64(),
                                           delta_flowrate_ne.x,
                                           delta_flowrate_ne.y,
                                           delta_velocity_ne.x,
                                           delta_velocity_ne.y,
                                           height_estimate,
                                           delta_height,
                                           ins_height,
                                           last_ins_height,
                                           dt_ms);
    delta_velocity_ne.zero();
    last_ins_height = ins_height;
}
