#include "Copter.h"

#if TOY_MODE_ENABLED == ENABLED

// times in 0.1s units
#define TOY_ARM_COUNT 5
#define TOY_LAND_COUNT 15
#define TOY_FORCE_DISARM_COUNT 5
#define TOY_LAND_DISARM_COUNT 15
#define TOY_COMMMAND_DELAY 30

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
    AP_GROUPINFO("_MODE1", 2, ToyMode, primary_mode1, ALT_HOLD),

    // @Param: _MODE2
    // @DisplayName: Tmode second mode
    // @Description: This is the secondary mode. This mode is assumed to require GPS
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS
    // @User: Standard
    AP_GROUPINFO("_MODE2", 3, ToyMode, primary_mode2, LOITER),
    
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

    if (first_update) {
        first_update = false;
        copter.set_mode(control_mode_t(primary_mode2.get()), MODE_REASON_TX_COMMAND);
    }

    if (copter.failsafe.radio) {
        // failsafe handling is outside the scope of toy mode, it does
        // normal failsafe actions
        return;
    }
    
    uint16_t ch5_in = hal.rcin->read(CH_5);
    uint16_t ch6_in = hal.rcin->read(CH_6);
    uint16_t ch7_in = hal.rcin->read(CH_7);
    bool secondary_mode = (ch5_in > 1700);
    bool mode_change = (ch5_in > 900 && secondary_mode != last_secondary_mode);

    // get buttons from channels
    bool left_button = (ch5_in > 1950 || (ch5_in > 1050 && ch5_in < 1150));
    bool right_button = (ch6_in > 1500);
    uint8_t ch7_bits = (ch7_in>1000)?uint8_t((ch7_in-1000)/100):0;
    bool left_action_button = (ch7_bits&1) != 0;
    bool right_action_button = (ch7_bits&2) != 0;
    bool power_button = (ch7_bits&4) != 0;
    
    last_secondary_mode = secondary_mode;

    if (right_button && !left_button) {
        arm_counter++;
    } else {
        arm_counter = 0;
    }

    // don't arm if sticks aren't in deadzone, to prevent pot problems
    // on TX causing flight control issues
    bool sticks_centered =
        copter.channel_roll->get_control_in() == 0 &&
        copter.channel_pitch->get_control_in() == 0 &&
        copter.channel_yaw->get_control_in() == 0;

    bool throttle_at_min =
        copter.channel_throttle->get_control_in() == 0;
    
    if (power_button && copter.motors->armed()) {
        power_counter++;
        if (power_counter >= TOY_FORCE_DISARM_COUNT) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: Force disarm");
            copter.init_disarm_motors();
        }
    } else {
        power_counter = 0;
    }


    if (throttle_at_min && copter.motors->armed() && copter.ap.land_complete) {
        throttle_low_counter++;
        if (throttle_low_counter >= TOY_LAND_DISARM_COUNT) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: disarm");
            copter.init_disarm_motors();
        }
    } else {
        throttle_low_counter = 0;
    }
    
    if (arm_counter > TOY_ARM_COUNT && !copter.motors->armed() && sticks_centered) {
        // 1 second for arming.
        if (secondary_mode && copter.arming.pre_arm_gps_checks(false)) {
            // we want GPS and checks are passing, arm and enable fence
            copter.set_mode(control_mode_t(primary_mode2.get()), MODE_REASON_TX_COMMAND);
            copter.fence.enable(true);
            if (copter.control_mode == primary_mode2) {
                copter.init_arm_motors(false);
                if (!copter.motors->armed()) {
                    AP_Notify::events.arming_failed = true;
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: GPS arming failed");
                } else {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: GPS armed motors");
                }
                arm_counter = -TOY_COMMMAND_DELAY;
            }
        } else if (secondary_mode) {
            // notify of arming fail
            AP_Notify::events.arming_failed = true;
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: GPS arming failed");
        } else {
            // non-GPS mode
            copter.set_mode(control_mode_t(primary_mode1.get()), MODE_REASON_TX_COMMAND);
            copter.fence.enable(false);
            if (copter.control_mode == primary_mode1) {
                copter.init_arm_motors(false);
                arm_counter = -TOY_COMMMAND_DELAY;
                if (!copter.motors->armed()) {
                    AP_Notify::events.arming_failed = true;
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: non-GPS arming failed");
                } else {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: non-GPS armed motors");
                }
            }
        }
    }
    if (arm_counter > TOY_LAND_COUNT && copter.motors->armed() && !copter.ap.land_complete) {
        if (copter.control_mode == primary_mode2) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: GPS RTL");
            copter.set_mode(RTL, MODE_REASON_TX_COMMAND);
        } else {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: non-GPS LAND");
            copter.set_mode(LAND, MODE_REASON_TX_COMMAND);
        }
        arm_counter = -TOY_COMMMAND_DELAY;
    }
    if (arm_counter > TOY_LAND_COUNT && copter.motors->armed() && copter.ap.land_complete) {
        copter.init_disarm_motors();
        arm_counter = -TOY_COMMMAND_DELAY;
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: disarmed");
    }

    if (mode_change) {
        if (!secondary_mode) {
            copter.fence.enable(false);
            copter.set_mode(control_mode_t(primary_mode1.get()), MODE_REASON_TX_COMMAND);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: first mode");
        } else {
            copter.fence.enable(true);
            copter.set_mode(control_mode_t(primary_mode2.get()), MODE_REASON_TX_COMMAND);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Tmode: second mode");
        }
    }
}

/*
  called from scheduler at 10Hz
 */
void Copter::toy_mode_update(void)
{
    g2.toy_mode.update();
}

#endif // TOY_MODE_ENABLED
