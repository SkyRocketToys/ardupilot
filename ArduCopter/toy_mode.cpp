#include "Copter.h"

#if TOY_MODE_ENABLED == ENABLED

// times in 0.1s units
#define TOY_COMMAND_DELAY 15
#define TOY_LONG_PRESS_COUNT 15
#define TOY_LAND_MANUAL_DISARM_COUNT 40
#define TOY_LAND_DISARM_COUNT 1
#define TOY_LAND_ARM_COUNT 1
#define TOY_RIGHT_PRESS_COUNT 1
#define TOY_ACTION_DELAY_MS 200

const AP_Param::GroupInfo ToyMode::var_info[] = {

    // @Param: _ENABLE
    // @DisplayName: tmode enable 
    // @Description: tmode (or "toy" mode) gives a simplified user interface designed for mass market drones.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLE", 1, ToyMode, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _MODE1
    // @DisplayName: Tmode first mode
    // @Description: This is the initial mode when the vehicle is first turned on. This mode is assumed to not require GPS
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS
    // @User: Standard
    AP_GROUPINFO("_MODE1", 2, ToyMode, primary_mode[0], ALT_HOLD),

    // @Param: _MODE2
    // @DisplayName: Tmode second mode
    // @Description: This is the secondary mode. This mode is assumed to require GPS
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS
    // @User: Standard
    AP_GROUPINFO("_MODE2", 3, ToyMode, primary_mode[1], LOITER),

    // @Param: _ACTION1
    // @DisplayName: Tmode action 1
    // @Description: This is the action taken for the left action button
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode
    // @User: Standard
    AP_GROUPINFO("_ACTION1", 4, ToyMode, actions[0], ACTION_TOGGLE_VIDEO),

    // @Param: _ACTION2
    // @DisplayName: Tmode action 2
    // @Description: This is the action taken for the right action button
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode
    // @User: Standard
    AP_GROUPINFO("_ACTION2", 5, ToyMode, actions[1], ACTION_TAKE_PHOTO),

    // @Param: _ACTION3
    // @DisplayName: Tmode action 3
    // @Description: This is the action taken for the power button
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode
    // @User: Standard
    AP_GROUPINFO("_ACTION3", 6, ToyMode, actions[2], ACTION_DISARM),

    // @Param: _ACTION4
    // @DisplayName: Tmode action 4
    // @Description: This is the action taken for the left action button while the mode button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode
    // @User: Standard
    AP_GROUPINFO("_ACTION4", 7, ToyMode, actions[3], ACTION_NONE),

    // @Param: _ACTION5
    // @DisplayName: Tmode action 5
    // @Description: This is the action taken for the right action button while the mode button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode
    // @User: Standard
    AP_GROUPINFO("_ACTION5", 8, ToyMode, actions[4], ACTION_NONE),

    // @Param: _ACTION6
    // @DisplayName: Tmode action 6
    // @Description: This is the action taken for the power button while the mode button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode
    // @User: Standard
    AP_GROUPINFO("_ACTION6", 9, ToyMode, actions[5], ACTION_NONE),

    // @Param: _LEFT
    // @DisplayName: Tmode left action
    // @Description: This is the action taken for the left button (mode button) being pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode
    // @User: Standard
    AP_GROUPINFO("_LEFT", 10, ToyMode, actions[6], ACTION_TOGGLE_MODE),

    // @Param: _LEFT_LONG
    // @DisplayName: Tmode left long action
    // @Description: This is the action taken for a long press of the left button (home button)
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode
    // @User: Standard
    AP_GROUPINFO("_LEFT_LONG", 11, ToyMode, actions[7], ACTION_NONE),

    // @Param: _TRIM_ARM
    // @DisplayName: Stick trim on arming
    // @Description: This is the amount of automatic stick trim that can be applied on arming. It is a percentage of total stick movement. When arming, the stick trim values will be automatically set to the current stick inputs if they are less than this limit.
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("_TRIM_ARM", 12, ToyMode, trim_arm, 20),

    // @Param: _RIGHT
    // @DisplayName: Tmode right action
    // @Description: This is the action taken for the right button (RTL) being pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode
    // @User: Standard
    AP_GROUPINFO("_RIGHT", 13, ToyMode, actions[8], ACTION_ARM_LAND_RTL),

    // @Param: _FLAGS
    // @DisplayName: Tmode flags
    // @Description: Bitmask of flags to change the behaviour of tmode. DisarmOnLowThrottle means to disarm if throttle is held down for 1 second when landed. ArmOnHighThrottle means to arm if throttle is above 80% for 1 second. UpgradeToLoiter means to allow takeoff in LOITER mode by switching to ALT_HOLD, then auto-upgrading to LOITER once GPS is available. RTLStickCancel means that on large stick inputs in RTL mode that LOITER mode is engaged
    // @Bitmask: 0:DisarmOnLowThrottle,1:ArmOnHighThrottle,2:UpgradeToLoiter,3:RTLStickCancel
    // @User: Standard
    AP_GROUPINFO("_FLAGS", 14, ToyMode, flags, FLAG_THR_DISARM),
    
    AP_GROUPEND
};

ToyMode::ToyMode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  special mode handling for toys
 */
void ToyMode::update()
{
    if (!enable) {
        // not enabled
        return;
    }

    // update LEDs
    blink_update();
    
    if (!done_first_update) {
        done_first_update = true;
        copter.set_mode(control_mode_t(primary_mode[0].get()), MODE_REASON_TMODE);
        throttle_mid = copter.channel_throttle->get_control_mid();
    }

    uint16_t ch5_in = hal.rcin->read(CH_5);
    uint16_t ch6_in = hal.rcin->read(CH_6);
    uint16_t ch7_in = hal.rcin->read(CH_7);
    bool left_change = ((ch5_in > 1700 && last_ch5 <= 1700) || (ch5_in <= 1700 && last_ch5 > 1700));

    if (copter.failsafe.radio || ch5_in < 900) {
        // failsafe handling is outside the scope of toy mode, it does
        // normal failsafe actions, just setup a blink pattern
        green_blink_pattern = BLINK_NO_RX;
        red_blink_pattern = BLINK_NO_RX;
        red_blink_index = green_blink_index;
        return;
    }
        
    last_ch5 = ch5_in;
                        
    // get buttons from channels
    bool left_button = (ch5_in > 2050 || (ch5_in > 1050 && ch5_in < 1150));
    bool right_button = (ch6_in > 1500);
    uint8_t ch7_bits = (ch7_in>1000)?uint8_t((ch7_in-1000)/100):0;
    bool left_action_button = (ch7_bits&1) != 0;
    bool right_action_button = (ch7_bits&2) != 0;
    bool power_button = (ch7_bits&4) != 0;

    // decode action buttons into an action
    uint8_t action_input = 0;    
    if (left_action_button) {
        action_input = 1;
    } else if (right_action_button) {
        action_input = 2;
    } else if (power_button) {
        action_input = 3;
    }
    
    if (action_input != 0 && left_button) {
        // combined button actions
        action_input += 3;
        left_press_counter = 0;
    } else if (left_button) {
        left_press_counter++;
    } else {
        left_press_counter = 0;
    }

    /*
      work out commanded action, if any
     */
    enum toy_action action = action_input?toy_action(actions[action_input-1].get()):ACTION_NONE;
   
    // check for long left button press
    if (action == ACTION_NONE && left_press_counter > TOY_LONG_PRESS_COUNT) {
        left_press_counter = -TOY_COMMAND_DELAY;
        action = toy_action(actions[7].get());
        ignore_left_change = true;
    }

    // cope with long left press triggering a left change
    if (ignore_left_change && left_change) {
        left_change = false;
        ignore_left_change = false;
    }

    // check for left button latching change
    if (action == ACTION_NONE && left_change) {
        action = toy_action(actions[6].get());
    }

    // check for right button
    if (action == ACTION_NONE && right_button) {
        right_press_counter++;
        if (right_press_counter >= TOY_RIGHT_PRESS_COUNT) {
            action = toy_action(actions[8].get());
            right_press_counter = -TOY_COMMAND_DELAY;
        }
    } else {
        right_press_counter = 0;
    }

    uint32_t now = AP_HAL::millis();

    /*
      some actions shouldn't repeat too fast
     */
    switch (action) {
    case ACTION_TAKE_PHOTO:
    case ACTION_TOGGLE_VIDEO:
    case ACTION_TOGGLE_MODE:
    case ACTION_TOGGLE_SIMPLE:
    case ACTION_TOGGLE_SSIMPLE:
    case ACTION_ARM_LAND_RTL:
        if (now - last_action_ms < TOY_ACTION_DELAY_MS) {
            action = ACTION_NONE;
        }
        break;

    default:
        break;
    }
    
    if (action != ACTION_NONE) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Tmode: action %u", action);
        last_action_ms = now;
    }

    // we use 150 for throttle_at_min to cope with varying stick throws
    bool throttle_at_min =
        copter.channel_throttle->get_control_in() < 150;

    // throttle threshold for throttle arming
    bool throttle_near_max =
        copter.channel_throttle->get_control_in() > 700;
    
    /*
      disarm if throttle is low for 1 second when landed
     */
    if ((flags & FLAG_THR_DISARM) && throttle_at_min && copter.motors->armed() && copter.ap.land_complete) {
        throttle_low_counter++;
        const uint8_t disarm_limit = copter.mode_has_manual_throttle(copter.control_mode)?TOY_LAND_MANUAL_DISARM_COUNT:TOY_LAND_DISARM_COUNT;
        if (throttle_low_counter >= disarm_limit) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Tmode: throttle disarm");
            copter.init_disarm_motors();
        }
    } else {
        throttle_low_counter = 0;
    }

    /*
      arm if throttle is high for 1 second when landed
     */
    if ((flags & FLAG_THR_ARM) && throttle_near_max && !copter.motors->armed()) {
        throttle_high_counter++;
        if (throttle_high_counter >= TOY_LAND_ARM_COUNT) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Tmode: throttle arm");
            if (!copter.init_arm_motors(true) && (flags & FLAG_UPGRADE_LOITER) && copter.control_mode == LOITER) {
                /*
                  support auto-switching to ALT_HOLD, then upgrade to LOITER once GPS available
                 */
                if (copter.set_mode(ALT_HOLD, MODE_REASON_TMODE)) {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Tmode: ALT_HOLD update arm");
                    copter.fence.enable(false);
                    if (!copter.init_arm_motors(true)) {
                        // go back to LOITER
                        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: ALT_HOLD arm failed");
                        copter.set_mode(LOITER, MODE_REASON_TMODE);
                    } else {
                        upgrade_to_loiter = true;
                    }
                }
            } else {
                throttle_arm_ms = AP_HAL::millis();
            }
        }
    } else {
        throttle_high_counter = 0;
    }

    if (upgrade_to_loiter) {
        if (!copter.motors->armed() || copter.control_mode != ALT_HOLD) {
            upgrade_to_loiter = false;
        } else if (copter.position_ok() && copter.set_mode(LOITER, MODE_REASON_TMODE)) {
            copter.fence.enable(true);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Tmode: LOITER update");            
        }
    }

    if (copter.control_mode == RTL && (flags & FLAG_RTL_CANCEL) &&
        (labs(copter.channel_roll->get_control_in()) > 3000 ||
         labs(copter.channel_pitch->get_control_in()) > 3000 ||
         throttle_near_max)) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Tmode: RTL cancel");        
        copter.set_mode(LOITER, MODE_REASON_TMODE);
    }
    
    enum control_mode_t new_mode = copter.control_mode;

    /*
      implement actions
     */
    switch (action) {
    case ACTION_NONE:
        break;

    case ACTION_TAKE_PHOTO:
        send_named_int("SNAPSHOT", 1);
        break;

    case ACTION_TOGGLE_VIDEO:
        send_named_int("VIDEOTOG", 1);
        break;

    case ACTION_MODE_ACRO:
        new_mode = ACRO;
        break;
        
    case ACTION_MODE_ALTHOLD:
        new_mode = ALT_HOLD;
        break;

    case ACTION_MODE_AUTO:
        new_mode = AUTO;
        break;

    case ACTION_MODE_LOITER:
        new_mode = LOITER;
        break;

    case ACTION_MODE_RTL:
        new_mode = RTL;
        break;

    case ACTION_MODE_CIRCLE:
        new_mode = CIRCLE;
        break;

    case ACTION_MODE_LAND:
        new_mode = LAND;
        break;

    case ACTION_MODE_DRIFT:
        new_mode = DRIFT;
        break;

    case ACTION_MODE_SPORT:
        new_mode = SPORT;
        break;

    case ACTION_MODE_AUTOTUNE:
        new_mode = AUTOTUNE;
        break;

    case ACTION_MODE_POSHOLD:
        new_mode = POSHOLD;
        break;

    case ACTION_MODE_BRAKE:
        new_mode = BRAKE;
        break;

    case ACTION_MODE_THROW:
        new_mode = THROW;
        break;

    case ACTION_MODE_FLIP:
        new_mode = FLIP;
        break;

    case ACTION_MODE_STAB:
        new_mode = STABILIZE;
        break;
        
    case ACTION_DISARM:
        if (copter.motors->armed()) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: Force disarm");
            copter.init_disarm_motors();
        }
        break;

    case ACTION_TOGGLE_MODE:
        last_mode_choice = (last_mode_choice+1) % 2;
        new_mode = control_mode_t(primary_mode[last_mode_choice].get());
        break;

    case ACTION_TOGGLE_SIMPLE:
        copter.set_simple_mode(copter.ap.simple_mode?0:1);
        break;

    case ACTION_TOGGLE_SSIMPLE:
        copter.set_simple_mode(copter.ap.simple_mode?0:2);
        break;
        
    case ACTION_ARM_LAND_RTL:
        if (!copter.motors->armed()) {
            action_arm();
        } else if (new_mode == RTL) {
            // toggle between RTL and LAND
            new_mode = LAND;
        } else if (new_mode == LAND) {
            // toggle between LAND and RTL
            new_mode = RTL;
        } else if (copter.mode_requires_GPS(new_mode)) {
            // if we're in a GPS mode, then RTL
            new_mode = RTL;
        } else {
            // if we're in a non-GPS mode, then LAND
            new_mode = LAND;
        }
        break;
    }

    if (!copter.motors->armed() && (copter.control_mode == LAND || copter.control_mode == RTL)) {
        // revert back to last primary flight mode if disarmed after landing
        new_mode = control_mode_t(primary_mode[last_mode_choice].get());
    }
    
    if (new_mode != copter.control_mode) {
        if (copter.mode_requires_GPS(new_mode)) {
            copter.fence.enable(true);
        } else {
            copter.fence.enable(false);
        }
        if (copter.set_mode(new_mode, MODE_REASON_TX_COMMAND)) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Tmode: mode %s", copter.flight_mode_string(new_mode));
        } else {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: mode %s FAILED", copter.flight_mode_string(new_mode));
            if (new_mode == RTL) {
                // if we can't RTL then land
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: LANDING");
                copter.set_mode(LAND, MODE_REASON_TMODE);
            }
        }
    }
}

/*
  automatic stick trimming
 */
void ToyMode::trim_sticks(void)
{
    uint16_t limit = trim_arm * 0.01 * ROLL_PITCH_YAW_INPUT_MAX;

    if (labs(copter.channel_roll->get_control_in()) >= limit ||
        labs(copter.channel_pitch->get_control_in()) >= limit ||
        labs(copter.channel_yaw->get_control_in()) >= limit ||
        labs(copter.channel_throttle->get_control_in() - 500) >= trim_arm * 0.01 * 1000) {
        // inputs too large to auto-trim
        return;
    }

    // auto-trim roll, pitch and yaw
    copter.channel_roll->set_radio_trim(copter.channel_roll->get_radio_in());
    copter.channel_pitch->set_radio_trim(copter.channel_pitch->get_radio_in());
    copter.channel_yaw->set_radio_trim(copter.channel_yaw->get_radio_in());

    // remember the throttle trim
    throttle_mid = copter.channel_throttle->get_control_in();
}

/*
  handle arming action
 */
void ToyMode::action_arm(void)
{
    bool needs_gps = copter.mode_requires_GPS(copter.control_mode);

    // check if we should auto-trim
    if (trim_arm > 0) {
        trim_sticks();
    }
        
    // don't arm if sticks aren't in deadzone, to prevent pot problems
    // on TX causing flight control issues
    bool sticks_centered =
        copter.channel_roll->get_control_in() == 0 &&
        copter.channel_pitch->get_control_in() == 0 &&
        copter.channel_yaw->get_control_in() == 0;

    if (!sticks_centered) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: sticks not centered\n");
        return;
    }
    
    if (needs_gps && copter.arming.pre_arm_gps_checks(false)) {
        // we want GPS and checks are passing, arm and enable fence
        copter.fence.enable(true);
        copter.init_arm_motors(false);
        if (!copter.motors->armed()) {
            AP_Notify::events.arming_failed = true;
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: GPS arming failed");
        } else {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: GPS armed motors");
        }
    } else if (needs_gps) {
        // notify of arming fail
        AP_Notify::events.arming_failed = true;
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: GPS arming failed");
    } else {
        // non-GPS mode
        copter.fence.enable(false);
        copter.init_arm_motors(false);
        if (!copter.motors->armed()) {
            AP_Notify::events.arming_failed = true;
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: non-GPS arming failed");
        } else {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: non-GPS armed motors");
        }
    }
}

/*
  adjust throttle for throttle takeoff
  This prevents sudden climbs when using throttle for arming
*/
void ToyMode::throttle_adjust(float &throttle_control)
{
    uint32_t now = AP_HAL::millis();
    const uint32_t soft_start_ms = 5000;
    const uint16_t throttle_start = 600 + copter.g.throttle_deadzone;
    if (!copter.motors->armed() && (flags & FLAG_THR_ARM)) {
        throttle_control = MIN(throttle_control, 500);
    } else if (now - throttle_arm_ms < soft_start_ms) {
        float p = (now - throttle_arm_ms) / float(soft_start_ms);
        throttle_control = MIN(throttle_control, throttle_start + p * (1000 - throttle_start));
    }
}

/*
  update blinking. Blinking is done with a 16 bit pattern for each
  LED. A count can be set for a pattern, which makes the pattern
  persist until the count is zero. When it is zero the normal pattern
  settings based on system status are used
 */
void ToyMode::blink_update(void)
{
    if (red_blink_pattern & (1U<<red_blink_index)) {
        copter.relay.on(1);
    } else {
        copter.relay.off(1);
    }
    if (green_blink_pattern & (1U<<green_blink_index)) {
        copter.relay.on(0);
    } else {
        copter.relay.off(0);
    }
    green_blink_index = (green_blink_index+1) % 16;
    red_blink_index = (red_blink_index+1) % 16;
    if (green_blink_index == 0 && green_blink_count > 0) {
        green_blink_count--;
    }
    if (red_blink_index == 0 && red_blink_count > 0) {
        red_blink_count--;
    }

    if (red_blink_count > 0 && green_blink_count > 0) {
        return;
    }
    
    // setup normal patterns based on flight mode and arming
    uint16_t pattern = 0;

    // full on when we can see the TX, except for battery failsafe,
    // when we blink rapidly
    if (copter.motors->armed() && AP_Notify::flags.failsafe_battery) {
        pattern = BLINK_8;
    } else {
        pattern = BLINK_FULL;
    }

    if (red_blink_count == 0) {
        red_blink_pattern = pattern;
    }
    if (green_blink_count == 0) {
        green_blink_pattern = pattern;
    }
    if (red_blink_count == 0 && green_blink_count == 0) {
        // get LEDs in sync
        red_blink_index = green_blink_index;
    }
}

// handle a mavlink message
void ToyMode::handle_message(mavlink_message_t *msg)
{
    if (msg->msgid != MAVLINK_MSG_ID_NAMED_VALUE_INT) {
        return;
    }
    mavlink_named_value_int_t m;
    mavlink_msg_named_value_int_decode(msg, &m);
    if (strncmp(m.name, "BLINKR", 10) == 0) {
        red_blink_pattern = (uint16_t)m.value;
        red_blink_count = m.value >> 16;
        red_blink_index = 0;
    } else if (strncmp(m.name, "BLINKG", 10) == 0) {
        green_blink_pattern = (uint16_t)m.value;
        green_blink_count = m.value >> 16;
        green_blink_index = 0;
    } else if (strncmp(m.name, "VNOTIFY", 10) == 0) {
        // taking photos or video
        if (green_blink_pattern != BLINK_2) {
            green_blink_index = 0;
        }
        green_blink_pattern = BLINK_2;
        green_blink_count = 1;
    } else if (strncmp(m.name, "WIFICHAN", 10) == 0) {
#ifdef HAL_RCINPUT_WITH_AP_RADIO
        AP_Radio *radio = AP_Radio::instance();
        if (radio) {
            radio->set_wifi_channel(m.value);
        }
#endif
    }
}

/*
  send a named int to primary telem channel
 */
void ToyMode::send_named_int(const char *name, int32_t value)
{
    mavlink_msg_named_value_int_send(MAVLINK_COMM_1, AP_HAL::millis(), name, value);
}

#if TOY_MODE_ENABLED == ENABLED
/*
  called from scheduler at 10Hz
 */
void Copter::toy_mode_update(void)
{
    g2.toy_mode.update();
}
#endif


#endif // TOY_MODE_ENABLED
