#include "Copter.h"

#if TOY_MODE_ENABLED == ENABLED

#define TOY_GPS_MODE LOITER
#define TOY_NON_GPS_MODE ALT_HOLD

// times in 0.1s units
#define TOY_ARM_COUNT 5
#define TOY_LAND_COUNT 15
#define TOY_FORCE_DISARM_COUNT 5
#define TOY_LAND_DISARM_COUNT 15
#define TOY_COMMMAND_DELAY 30

#define TOY_CH5_RESCALE 0

/*
  special mode handling for toys
 */
void ToyMode::update()
{
    uint16_t ch5_in = hal.rcin->read(CH_5);
    bool gps_enable = (ch5_in > 1700);
    bool mode_change = (ch5_in > 900 && gps_enable != last_gps_enable);

    last_gps_enable = gps_enable;

    if (hal.rcin->read(CH_6) > 1700) {
        ch6_counter++;
    } else {
        ch6_counter = 0;
    }

    // don't arm if sticks aren't in deadzone, to prevent pot problems
    // on TX causing flight control issues
    bool sticks_centered =
        copter.channel_roll->get_control_in() == 0 &&
        copter.channel_pitch->get_control_in() == 0 &&
        copter.channel_yaw->get_control_in() == 0;

    bool throttle_at_min =
        copter.channel_throttle->get_control_in() == 0;
    
    // power button adds 400 to ch7
    bool power_pressed = hal.rcin->read(CH_7) >= 1380;

    if (power_pressed && copter.motors->armed()) {
        power_counter++;
        if (power_counter >= TOY_FORCE_DISARM_COUNT) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: Force disarm");
            copter.init_disarm_motors();
        }
    } else {
        power_counter = 0;
    }


    if (throttle_at_min && copter.motors->armed() && copter.ap.land_complete) {
        throttle_low_counter++;
        if (throttle_low_counter >= TOY_LAND_DISARM_COUNT) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: disarm");
            copter.init_disarm_motors();
        }
    } else {
        throttle_low_counter = 0;
    }
    
    if (ch6_counter > TOY_ARM_COUNT && !copter.motors->armed() && sticks_centered) {
        // 1 second for arming.
        if (gps_enable && copter.arming.pre_arm_gps_checks(false)) {
            // we want GPS and checks are passing, arm and enable fence
            copter.set_mode(TOY_GPS_MODE, MODE_REASON_TX_COMMAND);
            copter.fence.enable(true);
            if (copter.control_mode == TOY_GPS_MODE) {
                copter.init_arm_motors(false);
                if (!copter.motors->armed()) {
                    AP_Notify::events.arming_failed = true;
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: GPS arming failed");
                } else {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: GPS armed motors");
                }
                ch6_counter = -TOY_COMMMAND_DELAY;
            }
        } else if (gps_enable) {
            // notify of arming fail
            AP_Notify::events.arming_failed = true;
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: GPS arming failed");
        } else {
            // non-GPS mode
            copter.set_mode(TOY_NON_GPS_MODE, MODE_REASON_TX_COMMAND);
            copter.fence.enable(false);
            if (copter.control_mode == TOY_NON_GPS_MODE) {
                copter.init_arm_motors(false);
                ch6_counter = -TOY_COMMMAND_DELAY;
                if (!copter.motors->armed()) {
                    AP_Notify::events.arming_failed = true;
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: non-GPS arming failed");
                } else {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: non-GPS armed motors");
                }
            }
        }
    }
    if (ch6_counter > TOY_LAND_COUNT && copter.motors->armed() && !copter.ap.land_complete) {
        if (copter.control_mode == TOY_GPS_MODE) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: GPS RTL");
            copter.set_mode(RTL, MODE_REASON_TX_COMMAND);
        } else {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: non-GPS LAND");
            copter.set_mode(LAND, MODE_REASON_TX_COMMAND);
        }
        ch6_counter = -TOY_COMMMAND_DELAY;
    }
    if (ch6_counter > TOY_LAND_COUNT && copter.motors->armed() && copter.ap.land_complete) {
        copter.init_disarm_motors();
        ch6_counter = -TOY_COMMMAND_DELAY;
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: disarmed");
    }

    if (mode_change) {
        if (!gps_enable) {
            copter.fence.enable(false);
            copter.set_mode(TOY_NON_GPS_MODE, MODE_REASON_TX_COMMAND);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: GPS mode");
        } else {
            copter.fence.enable(true);
            copter.set_mode(TOY_GPS_MODE, MODE_REASON_TX_COMMAND);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: non-GPS mode");
        }
    }
}

/*
  called from scheduler at 10Hz
 */
void Copter::toy_mode_update(void)
{
    toy_mode.update();
}

#endif // TOY_MODE_ENABLED
