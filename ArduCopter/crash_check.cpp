#include "Copter.h"

// Code to detect a crash main ArduCopter code
#define CRASH_CHECK_TRIGGER_SEC         3       // 3 seconds indicates a crash
#define CRASH_CHECK_TRIGGER_SEC         3       // 3 seconds indicates a crash
#define CRASH_CHECK_ANGLE_DEVIATION_DEG 30.0f   // 30 degrees beyond angle max is signal we are inverted
#define CRASH_CHECK_ACCEL_MAX           3.0f    // vehicle must be accelerating less than 3m/s/s to be considered crashed

// crash_check - disarms motors if a crash has been detected
// crashes are detected by the vehicle being more than 20 degrees beyond it's angle limits continuously for more than 1 second
// called at MAIN_LOOP_RATE
void Copter::crash_check()
{
    // return immediately if disarmed, or crash checking disabled
    if (!motors->armed() || ap.land_complete || g.fs_crash_check == 0) {
        crash.counter = 0;
        return;
    }

    // return immediately if we are not in an angle stabilize flight mode or we are flipping
    if (control_mode == ACRO || control_mode == FLIP || control_mode == SPORT) {
        crash.counter = 0;
        return;
    }

    // vehicle not crashed if 1hz filtered acceleration is more than 3m/s (1G on Z-axis has been subtracted)
    if (land_accel_ef_filter.get().length() >= CRASH_CHECK_ACCEL_MAX) {
        crash.counter = 0;
        return;
    }

    bool climb_rate_error = false;
    bool controls_saturated = false;
    bool angle_error = false;
    uint32_t now = AP_HAL::millis();

    // check for angle error over 30 degrees
    float angle_error_deg = attitude_control->get_att_error_angle_deg();
    angle_error = (angle_error_deg > CRASH_CHECK_ANGLE_DEVIATION_DEG);
    
    // calculate filtered climb rate. We use the barometric climb rate
    // as the inertial climb rate is often badly off when we are
    // crashed
    crash.filtered_climb_rate = 0.97 * crash.filtered_climb_rate + barometer.get_climb_rate() * 0.03;
    float scaled_throttle = motors->get_throttle() * ahrs.cos_roll() * ahrs.cos_pitch();

#ifdef CRASH_CHECK_THROTTLE_THRESHOLD
    Vector3f att_target = attitude_control->get_att_target_euler_cd();
    // only check throttle when target angle is less than 20 degrees
    if (fabsf(att_target.x) + fabsf(att_target.y) < 2000) {
        if (fabsf(crash.filtered_climb_rate) < CRASH_CHECK_CLIMB_THRESHOLD &&
            (scaled_throttle > CRASH_CHECK_THROTTLE_THRESHOLD || motors->limit.throttle_upper)) {
            // we should be climbing and we aren't
            climb_rate_error = true;
        }
    }
#endif

    // if roll or pitch controller is saturated for the crash check time then its a crash
    if (motors->limit.roll_pitch) {
        controls_saturated = true;
    }

    if (now - crash.last_log > 25) {
        crash.last_log = now;
        uint8_t flags = 0;
        if (motors->limit.roll_pitch) {
            flags |= 1;
        }
        if (motors->limit.throttle_upper) {
            flags |= 2;
        }
        DataFlash_Class::instance()->Log_Write("CCHK", "TimeUS,Count,FCRt,AErr,SThr,Flags", "QHfffB",
                                               AP_HAL::micros64(),
                                               crash.counter, (double)crash.filtered_climb_rate,
                                               (double)angle_error_deg, (double)scaled_throttle, flags);
                                               
    }

    if (!angle_error && !climb_rate_error && !controls_saturated) {
        crash.counter = 0;
        return;
    }

    // we may be crashing
    crash.counter++;    

    // check if crashing for 2 seconds
    if (crash.counter >= (CRASH_CHECK_TRIGGER_SEC * scheduler.get_loop_rate_hz())) {
        if (!angle_error && climb_rate_error && !controls_saturated && control_mode != LAND) {
            // this could be a false positive due to inability to
            // climb at high throttle. This happens when battery is
            // low in wind. Go into LAND rather than disarming
            gcs_send_text(MAV_SEVERITY_EMERGENCY,"Crash: obstruction landing");
            set_mode(LAND, MODE_REASON_OBSTRUCTION);
            crash.counter = 0;
            crash.last_trigger_ms = now;
            return;
        }
        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_CRASH_CHECK, ERROR_CODE_CRASH_CHECK_CRASH);
        // send message to gcs
        gcs_send_text(MAV_SEVERITY_EMERGENCY,"Crash: Disarming");

        // disarm motors
        init_disarm_motors();
        crash.last_trigger_ms = now;
    }
}

#if PARACHUTE == ENABLED

// Code to detect a crash main ArduCopter code
#define PARACHUTE_CHECK_TRIGGER_SEC         1       // 1 second of loss of control triggers the parachute
#define PARACHUTE_CHECK_ANGLE_DEVIATION_CD  3000    // 30 degrees off from target indicates a loss of control

// parachute_check - disarms motors and triggers the parachute if serious loss of control has been detected
// vehicle is considered to have a "serious loss of control" by the vehicle being more than 30 degrees off from the target roll and pitch angles continuously for 1 second
// called at MAIN_LOOP_RATE
void Copter::parachute_check()
{
    static uint16_t control_loss_count;	// number of iterations we have been out of control
    static int32_t baro_alt_start;

    // exit immediately if parachute is not enabled
    if (!parachute.enabled()) {
        return;
    }

    // call update to give parachute a chance to move servo or relay back to off position
    parachute.update();

    // return immediately if motors are not armed or pilot's throttle is above zero
    if (!motors->armed()) {
        control_loss_count = 0;
        return;
    }

    // return immediately if we are not in an angle stabilize flight mode or we are flipping
    if (control_mode == ACRO || control_mode == FLIP) {
        control_loss_count = 0;
        return;
    }

    // ensure we are flying
    if (ap.land_complete) {
        control_loss_count = 0;
        return;
    }

    // ensure the first control_loss event is from above the min altitude
    if (control_loss_count == 0 && parachute.alt_min() != 0 && (current_loc.alt < (int32_t)parachute.alt_min() * 100)) {
        return;
    }

    // check for angle error over 30 degrees
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error <= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        if (control_loss_count > 0) {
            control_loss_count--;
        }
        return;
    }

    // increment counter
    if (control_loss_count < (PARACHUTE_CHECK_TRIGGER_SEC*scheduler.get_loop_rate_hz())) {
        control_loss_count++;
    }

    // record baro alt if we have just started losing control
    if (control_loss_count == 1) {
        baro_alt_start = baro_alt;

    // exit if baro altitude change indicates we are not falling
    } else if (baro_alt >= baro_alt_start) {
        control_loss_count = 0;
        return;

    // To-Do: add check that the vehicle is actually falling

    // check if loss of control for at least 1 second
    } else if (control_loss_count >= (PARACHUTE_CHECK_TRIGGER_SEC*scheduler.get_loop_rate_hz())) {
        // reset control loss counter
        control_loss_count = 0;
        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_CRASH_CHECK, ERROR_CODE_CRASH_CHECK_LOSS_OF_CONTROL);
        // release parachute
        parachute_release();
    }
}

// parachute_release - trigger the release of the parachute, disarm the motors and notify the user
void Copter::parachute_release()
{
    // send message to gcs and dataflash
    gcs_send_text(MAV_SEVERITY_INFO,"Parachute: Released");
    Log_Write_Event(DATA_PARACHUTE_RELEASED);

    // disarm motors
    init_disarm_motors();

    // release parachute
    parachute.release();

    // deploy landing gear
    landinggear.set_position(AP_LandingGear::LandingGear_Deploy);
}

// parachute_manual_release - trigger the release of the parachute, after performing some checks for pilot error
//   checks if the vehicle is landed 
void Copter::parachute_manual_release()
{
    // exit immediately if parachute is not enabled
    if (!parachute.enabled()) {
        return;
    }

    // do not release if vehicle is landed
    // do not release if we are landed or below the minimum altitude above home
    if (ap.land_complete) {
        // warn user of reason for failure
        gcs_send_text(MAV_SEVERITY_INFO,"Parachute: Landed");
        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_PARACHUTE, ERROR_CODE_PARACHUTE_LANDED);
        return;
    }

    // do not release if we are landed or below the minimum altitude above home
    if ((parachute.alt_min() != 0 && (current_loc.alt < (int32_t)parachute.alt_min() * 100))) {
        // warn user of reason for failure
        gcs_send_text(MAV_SEVERITY_ALERT,"Parachute: Too low");
        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_PARACHUTE, ERROR_CODE_PARACHUTE_TOO_LOW);
        return;
    }

    // if we get this far release parachute
    parachute_release();
}

#endif // PARACHUTE == ENABLED
