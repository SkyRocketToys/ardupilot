#include "Copter.h"
#include <utility>

#if !HAL_MINIMIZE_FEATURES && OPTFLOW == ENABLED

/*
  implement FLOWHOLD mode, for position hold using opttical flow
  without rangefinder
 */

const AP_Param::GroupInfo Copter::ModeFlowHold::var_info[] = {
    // @Param: _XY_P
    // @DisplayName: FlowHold P gain
    // @Description: FlowHold (horizontal) P gain.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _XY_I
    // @DisplayName: FlowHold I gain
    // @Description: FlowHold (horizontal) I gain
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _XY_IMAX
    // @DisplayName: FlowHold Integrator Max
    // @Description: FlowHold (horizontal) integrator maximum
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cdeg
    // @User: Advanced
    AP_SUBGROUPINFO(flow_pi_xy, "_XY_",  1, Copter::ModeFlowHold, AC_PI_2D),

    // @Param: _FLOW_MAX
    // @DisplayName: FlowHold Flow Rate Max
    // @Description: Controls maximum apparent flow rate in flowhold
    // @Range: 0.1 2.5
    // @User: Standard
    AP_GROUPINFO("_FLOW_MAX", 2, Copter::ModeFlowHold, flow_max, 0.6),

    // @Param: _FILT_HZ
    // @DisplayName: FlowHold Filter Frequency
    // @Description: Filter frequency for flow data
    // @Range: 1 100
    // @User: Standard
    AP_GROUPINFO("_FILT_HZ", 3, Copter::ModeFlowHold, flow_filter_hz, 5),

    // @Param: _QUAL_MIN
    // @DisplayName: FlowHold Flow quality minimum
    // @Description: Minimum flow quality to use flow position hold
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_QUAL_MIN", 4, Copter::ModeFlowHold, flow_min_quality, 10),

    // 5 was FLOW_SPEED

    // @Param: _BRAKE_RATE
    // @DisplayName: FlowHold Braking rate
    // @Description: Controls deceleration rate on stick release
    // @Range: 1 30
    // @User: Standard
    // @Units: deg/s
    AP_GROUPINFO("_BRAKE_RATE", 6, Copter::ModeFlowHold, brake_rate_dps, 8),

    // @Param: _HGT_OFS
    // @DisplayName: FlowHold height offset
    // @Description: This height offset is added to height estimate to account for barometric disturbance from motors
    // @Range: -2 2
    // @User: Standard
    // @Units: m
    AP_GROUPINFO("_HGT_OFS", 7, Copter::ModeFlowHold, height_adjustment, 8),

    // @Param: _LAND_BSPK
    // @DisplayName: FlowHold land baro spike threshold
    // @Description: This is the threshold for barometer spike for landing detection
    // @Range: 0 2
    // @User: Standard
    // @Units: m
    AP_GROUPINFO("_LAND_BSPK", 8, Copter::ModeFlowHold, land_baro_spike, 0.5),

    // @Param: _LAND_FSPK
    // @DisplayName: FlowHold land flow spike threshold
    // @Description: This is the threshold for flow rate spike for landing detection. The flow rate must spike above this limit at the same time as the baro spikes above the baro limit to detect landing
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("_LAND_FSPK", 9, Copter::ModeFlowHold, land_flow_spike, 1.0),
    
    AP_GROUPEND
};

Copter::ModeFlowHold::ModeFlowHold(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);            
}

#define CONTROL_FLOWHOLD_EARTH_FRAME 0

// flowhold_init - initialise flowhold controller
bool Copter::ModeFlowHold::init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Flow Hold if the Rotor Runup is not complete
    if (!ignore_checks && !motors->rotor_runup_complete()){
        return false;
    }
#endif

    if (!copter.optflow.enabled() || !copter.optflow.healthy()) {
        return false;
    }

    in_landing = false;

    if (copter.control_mode == FLOWHOLD) {
        // no need to re-init. This is probably a flow landing
        return true;
    }
    
    // initialize vertical speeds and leash lengths
    copter.pos_control->set_speed_z(-get_pilot_speed_dn(), copter.g.pilot_speed_up);
    copter.pos_control->set_accel_z(copter.g.pilot_accel_z);

    // initialise position and desired velocity
    if (copter.control_mode == FLIP) {
        last_stick_input_ms = millis();
        last_ins_height = copter.inertial_nav.get_altitude() * 0.01;
        braking = true;
    } else {
        if (!copter.pos_control->is_active_z()) {
            copter.pos_control->set_alt_target_to_current_alt();
            copter.pos_control->set_desired_velocity_z(copter.inertial_nav.get_velocity_z());
        }
        // start with INS height
        last_ins_height = copter.inertial_nav.get_altitude() * 0.01;
        height_offset = 0;
        quality_filtered = 0;
        flow_pi_xy.reset_I();
        limited = false;
    }
    
    flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), flow_filter_hz.get());
    
    // stop takeoff if running
    copter.takeoff_stop();

    flow_pi_xy.set_dt(1.0/copter.scheduler.get_loop_rate_hz());

    
    return true;
}

/*
  calculate desired attitude from flow sensor. Called when flow sensor is healthy
 */
void Copter::ModeFlowHold::flowhold_flow_to_angle(Vector2f &bf_angles, bool stick_input)
{
    uint32_t now = AP_HAL::millis();
    
    // get corrected raw flow rate
    Vector2f raw_flow = copter.optflow.flowRate() - copter.optflow.bodyRate();

    // limit sensor flow, this prevents oscillation at low altitudes
    raw_flow.x = constrain_float(raw_flow.x, -flow_max, flow_max);
    raw_flow.y = constrain_float(raw_flow.y, -flow_max, flow_max);

    // rotate controller input to earth frame
    raw_flow = copter.ahrs.rotate_body_to_earth2D(raw_flow);
    
    // filter the flow rate
    Vector2f sensor_flow = flow_filter.apply(raw_flow);

    // get constrained height estimate
    float height_estimate = constrain_float(get_height_estimate(), height_min, height_max);

    // get body->NDE rotation matrix
    const Matrix3f &rotmat = copter.ahrs.get_rotation_body_to_ned();

    Vector2f input_ef;
        
    Vector2f lean_dir_unit(rotmat.a.z, rotmat.b.z);
    float lean_dir_length = lean_dir_unit.length();
    Vector2f lean_cross_unit(-rotmat.b.z, rotmat.a.z);
    float lean_cross_length = lean_cross_unit.length();


    if (lean_cross_length > 0.001 && lean_dir_length > 0.001 && rotmat.c.z > 0.01) {
        // we are leaning over - apply the scaling factors to account
        // for increased range and image projection length on the
        // ground. (maths from Leonard)
        lean_dir_unit /= lean_dir_length;
        lean_cross_unit /= lean_cross_length;
        
        input_ef = lean_dir_unit * (sensor_flow * lean_dir_unit) * (height_estimate/(rotmat.c.z*rotmat.c.z));
        input_ef += lean_cross_unit * (sensor_flow * lean_cross_unit) * (height_estimate/rotmat.c.z);
    } else {
        input_ef = sensor_flow * height_estimate;
    }
    
    // run PI controller
    flow_pi_xy.set_input(input_ef);

    // get earth frame controller attitude in centi-degrees
    Vector2f ef_output;

    // get P term
    ef_output = flow_pi_xy.get_p();

    if (stick_input) {
        last_stick_input_ms = now;
        braking = true;
        braking_angle = atan2f(sensor_flow.y, sensor_flow.x);
    }
    if (!stick_input && braking) {
        // stop braking if either 3s has passed, or we have slowed below 0.3m/s
        float flow_angle = atan2f(sensor_flow.y, sensor_flow.x);
        float flow_diff_cd = wrap_180_cd(degrees(flow_angle - braking_angle) * 100);
        //printf("%.2f %u\n", (now - last_stick_input_ms)*0.001, labs(flow_diff_cd));
        if (now - last_stick_input_ms > 3000 || fabsf(flow_diff_cd) > 9000) {
            braking = false;
#if 0
            printf("braking done at %u vel=%f\n", now - last_stick_input_ms,
                   (double)sensor_flow.length());
#endif
        }
    }
    
    if (!stick_input && !braking) {
        // get I term
        if (limited) {
            // only allow I term to shrink in length
            xy_I = flow_pi_xy.get_i_shrink();
        } else {
            // normal I term operation
            xy_I = flow_pi_xy.get_pi();
        }
    }

    if (!stick_input && braking) {
        // calculate brake angle for each axis separately
        for (uint8_t i=0; i<2; i++) {
            float &velocity = sensor_flow[i];
            float abs_vel_cms = fabsf(velocity)*100;
            const float brake_gain = (15.0f * brake_rate_dps.get() + 95.0f) / 100.0f;
            float lean_angle_cd = brake_gain * abs_vel_cms * (1.0f+500.0f/(abs_vel_cms+60.0f));
            if (velocity < 0) {
                lean_angle_cd = -lean_angle_cd;
            }
            bf_angles[i] = lean_angle_cd;
        }
        ef_output.zero();
    }

    
    ef_output += xy_I;
    ef_output *= 4500; // convert to centidegrees

    // convert to body frame
    bf_angles += copter.ahrs.rotate_earth_to_body2D(ef_output);

    // set limited flag to prevent integrator windup
    limited = fabsf(bf_angles.x) > copter.aparm.angle_max || fabsf(bf_angles.y) > copter.aparm.angle_max;

    // constrain to angle limit
    bf_angles.x = constrain_float(bf_angles.x, -copter.aparm.angle_max, copter.aparm.angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -copter.aparm.angle_max, copter.aparm.angle_max);

    if (log_counter++ % 20 == 0) {
        DataFlash_Class::instance()->Log_Write("FHLD", "TimeUS,SFx,SFy,Ax,Ay,Qual,Ix,Iy,R,P,AM,FC,Bmin", "Qffffffffffff",
                                               AP_HAL::micros64(),
                                               (double)sensor_flow.x, (double)sensor_flow.y,
                                               (double)bf_angles.x, (double)bf_angles.y,
                                               (double)quality_filtered,
                                               (double)xy_I.x, (double)xy_I.y,
                                               (double)copter.ahrs.roll_sensor,
                                               (double)copter.ahrs.pitch_sensor,
                                               (double)copter.attitude_control->get_althold_lean_angle_max(),
                                               flow_check,
                                               baro_min_alt);
    }
}

float Copter::ModeFlowHold::get_height_estimate(void)
{
    float ins_height = copter.inertial_nav.get_altitude() * 0.01;
    float height_estimate = ins_height + height_offset;
    uint32_t now = AP_HAL::millis();
    if (copter.motors->armed() && now - copter.arm_time_ms < 10000) {
        float dt = (now - copter.arm_time_ms) * 0.001;
        float adjust = linear_interpolate(height_adjustment, 0, dt, 5, 10);
        height_estimate += adjust;
    }
    return height_estimate;
}

// flowhold_run - runs the flowhold controller
// should be called at 100hz or more
void Copter::ModeFlowHold::run()
{
    FlowHoldModeState flowhold_state;
    float takeoff_climb_rate = 0.0f;

    update_height_estimate();
    
    // initialize vertical speeds and acceleration
    copter.pos_control->set_speed_z(-get_pilot_speed_dn(), copter.g.pilot_speed_up);
    copter.pos_control->set_accel_z(copter.g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    copter.update_simple_mode();

    // check for filter change
    if (!is_equal(flow_filter.get_cutoff_freq(), flow_filter_hz.get())) {
        flow_filter.set_cutoff_frequency(flow_filter_hz.get());
    }

    // get pilot desired climb rate
    float target_climb_rate = copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), copter.g.pilot_speed_up);
    if (fabsf(target_climb_rate) > 0.001) {
        changing_alt = true;
    } else if (changing_alt) {
        changing_alt = false;
        //copter.pos_control->set_alt_target_to_current_alt();
        //copter.pos_control->set_desired_velocity_z(0);
    }

    if (in_landing) {
        float descent_speed = get_pilot_speed_dn() * 0.66;
        float height = get_height_estimate();
        target_climb_rate = -linear_interpolate(descent_speed/3, descent_speed, height, 3, 8);
        copter.pos_control->set_accel_z_limit_max(25);
    }
        
    // get pilot's desired yaw rate
    float target_yaw_rate = copter.get_pilot_desired_yaw_rate(copter.channel_yaw->get_control_in());
    
    if (!copter.motors->armed() || !copter.motors->get_interlock()) {
        flowhold_state = FlowHold_MotorStopped;
    } else if (copter.takeoff_state.running || takeoff_triggered(target_climb_rate)) {
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
    int16_t roll_in = copter.channel_roll->get_control_in();
    int16_t pitch_in = copter.channel_pitch->get_control_in();
    float angle_max = copter.aparm.angle_max;
    copter.get_pilot_desired_lean_angles(roll_in, pitch_in,
                                         bf_angles.x, bf_angles.y,
                                         angle_max);
    
    if (quality_filtered >= flow_min_quality &&
        AP_HAL::millis() - copter.arm_time_ms > 3000 &&
        labs(ahrs.roll_sensor)  < 3500 &&
        labs(ahrs.pitch_sensor) < 3500) {
        // don't use for first 3s when we are just taking off, or at high angles
        Vector2f flow_angles;

        flowhold_flow_to_angle(flow_angles, (roll_in != 0) || (pitch_in != 0));
        flow_angles.x = constrain_float(flow_angles.x, -angle_max/2, angle_max/2);
        flow_angles.y = constrain_float(flow_angles.y, -angle_max/2, angle_max/2);
        bf_angles += flow_angles;
    }
    bf_angles.x = constrain_float(bf_angles.x, -angle_max, angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -angle_max, angle_max);

#if 0
    DataFlash_Class::instance()->Log_Write("FH2", "TimeUS,BFx,BFy,State,RIn,PIn,AMax", "QffBhhf",
                                           AP_HAL::micros64(),
                                           (double)bf_angles.x, (double)bf_angles.y,
                                           flowhold_state,
                                           roll_in, pitch_in, angle_max);
#endif
    
    // Flow Hold State Machine
    switch (flowhold_state) {

    case FlowHold_MotorStopped:

        copter.motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x, bf_angles.y, target_yaw_rate, copter.get_smoothing_gain());
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        copter.pos_control->set_alt_target_from_climb_rate(-abs(copter.g.land_speed), copter.G_Dt, false);
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
    if (in_landing) {
        flow_land_detector();
    }
}


/*
  update height estimate using integrated accelerometer ratio with optical flow
 */
void Copter::ModeFlowHold::update_height_estimate(void)
{
    float ins_height = copter.inertial_nav.get_altitude() * 0.01;

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
                                           (double)delta_flowrate.x,
                                           (double)delta_flowrate.y,
                                           (double)delta_vel_rate.x,
                                           (double)delta_vel_rate.y,
                                           (double)height_estimate,
                                           (double)delta_height,
                                           (double)height_offset,
                                           (double)ins_height,
                                           (double)last_ins_height,
                                           dt_ms);
    mavlink_msg_named_value_float_send(MAVLINK_COMM_0, AP_HAL::millis(), "HEST", height_estimate);
    mavlink_msg_named_value_float_send(MAVLINK_COMM_1, AP_HAL::millis(), "HEST", height_estimate);
    delta_velocity_ne.zero();
    last_ins_height = ins_height;

    // flow rate spike detector. The flow rate rises very rapidly close to the ground
    float flow_rate_check = (copter.optflow.flowRate().length() / (copter.optflow.bodyRate().length()+1));
    flow_check = flow_check * 0.90 + flow_rate_check * 0.1;
    if (braking) {
        // zero when braking to prevent false landing
        flow_check = 0;
    }
}

// run in landing mode
void Copter::ModeFlowHold::run_land()
{
    in_landing = true;
    run();
}

/*
  a flowhold specific landing detector. This uses the distinctive
  spike in optical flow data when very close to the ground along with
  a rise in barometric altitude due to ground effect to trigger
  landing
 */
void Copter::ModeFlowHold::flow_land_detector()
{
    uint32_t now = millis();
    float balt = copter.barometer.get_altitude();
    if (now - last_land_check_ms > 1000 ||
        now - baro_min_alt_ms > 3000) {
        baro_min_alt = balt;
        baro_min_alt_ms = now;
    }
    last_land_check_ms = now;
    if (balt < baro_min_alt) {
        baro_min_alt = balt;
    }

    bool stick_input = copter.channel_roll->get_control_in() ||
        copter.channel_pitch->get_control_in() ||
        copter.channel_yaw->get_control_in();

    if (!stick_input && !ap.land_complete && !braking) {
        if (balt - baro_min_alt > land_baro_spike && flow_check > land_flow_spike) {
            gcs().send_text(MAV_SEVERITY_INFO, "FHLD: land detect %.1 %.1\n", balt - baro_min_alt, flow_check);
            set_land_complete(true);        
        }
    }
}

#endif // OPTFLOW == ENABLED

