#include "Copter.h"

#define TOY_GPS_MODE LOITER
#define TOY_NON_GPS_MODE ALT_HOLD

// times in 0.1s units
#define TOY_ARM_COUNT 5
#define TOY_LAND_COUNT 15
#define TOY_FORCE_DISARM_COUNT 5
#define TOY_COMMMAND_DELAY 30

#define TOY_CH5_RESCALE 0

/*
  special mode handling for toys
 */
void Copter::toy_input_check()
{
    static int32_t ch6_counter;
    static uint32_t power_counter;
    uint16_t ch5_in = hal.rcin->read(CH_5);
    static bool last_gps_enable;
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
        channel_roll->get_control_in() == 0 &&
        channel_pitch->get_control_in() == 0 &&
        channel_yaw->get_control_in() == 0;

    // power button adds 400 to ch7
    bool power_pressed = hal.rcin->read(CH_7) >= 1380;

    if (power_pressed && motors->armed()) {
        power_counter++;
        if (power_counter >= TOY_FORCE_DISARM_COUNT) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: Force disarm");
            copter.init_disarm_motors();
        }
    } else {
        power_counter = 0;
    }
    
    if (ch6_counter > TOY_ARM_COUNT && !motors->armed() && sticks_centered) {
        // 1 second for arming.
        if (gps_enable && arming.pre_arm_gps_checks(false)) {
            // we want GPS and checks are passing, arm and enable fence
            set_mode(TOY_GPS_MODE, MODE_REASON_TX_COMMAND);
            fence.enable(true);
            if (control_mode == TOY_GPS_MODE) {
                init_arm_motors(false);
                if (!motors->armed()) {
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
            set_mode(TOY_NON_GPS_MODE, MODE_REASON_TX_COMMAND);
            fence.enable(false);
            if (control_mode == TOY_NON_GPS_MODE) {
                init_arm_motors(false);
                ch6_counter = -TOY_COMMMAND_DELAY;
                if (!motors->armed()) {
                    AP_Notify::events.arming_failed = true;
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: non-GPS arming failed");
                } else {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: non-GPS armed motors");
                }
            }
        }
    }
    if (ch6_counter > TOY_LAND_COUNT && motors->armed() && !ap.land_complete) {
        if (control_mode == TOY_GPS_MODE) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: GPS RTL");
            set_mode(RTL, MODE_REASON_TX_COMMAND);
        } else {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: non-GPS LAND");
            set_mode(LAND, MODE_REASON_TX_COMMAND);
        }
        ch6_counter = -TOY_COMMMAND_DELAY;
    }
    if (ch6_counter > TOY_LAND_COUNT && motors->armed() && ap.land_complete) {
        init_disarm_motors();
        ch6_counter = -TOY_COMMMAND_DELAY;
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: disarmed");
    }

    if (mode_change) {
        if (!gps_enable) {
            fence.enable(false);
            set_mode(TOY_NON_GPS_MODE, MODE_REASON_TX_COMMAND);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: GPS mode");
        } else {
            fence.enable(true);
            set_mode(TOY_GPS_MODE, MODE_REASON_TX_COMMAND);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Toy: non-GPS mode");
        }
    }
}
