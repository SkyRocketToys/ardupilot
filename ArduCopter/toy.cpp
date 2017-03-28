#include "Copter.h"

#define TOY_GPS_MODE LOITER
#define TOY_NON_GPS_MODE ALT_HOLD

// times in 0.1s units
#define TOY_ARM_COUNT 5
#define TOY_LAND_COUNT 15
#define TOY_COMMMAND_DELAY 15

#define TOY_CH5_RESCALE 0

/*
  special mode handling for toys
 */
void Copter::toy_input_check()
{
    static int32_t ch6_counter;
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
    if (ch6_counter > TOY_ARM_COUNT && !motors->armed()) {
        // 1 second for arming.
        if (gps_enable && arming.pre_arm_gps_checks(false)) {
            // we want GPS and checks are passing, arm and enable fence
            set_mode(TOY_GPS_MODE, MODE_REASON_TX_COMMAND);
            fence.enable(true);
            if (control_mode == TOY_GPS_MODE) {
                init_arm_motors(false);
                if (!motors->armed()) {
                    AP_Notify::events.arming_failed = true;
                }
                ch6_counter = -TOY_COMMMAND_DELAY;
            }
        } else if (gps_enable) {
            // notify of arming fail
            AP_Notify::events.arming_failed = true;
        } else {
            // non-GPS mode
            set_mode(TOY_NON_GPS_MODE, MODE_REASON_TX_COMMAND);
            fence.enable(false);
            if (control_mode == TOY_NON_GPS_MODE) {
                init_arm_motors(false);
                ch6_counter = -TOY_COMMMAND_DELAY;
                if (!motors->armed()) {
                    AP_Notify::events.arming_failed = true;
                }
            }
        }
    }
    if (ch6_counter > TOY_LAND_COUNT && motors->armed() && !ap.land_complete) {
        set_mode(LAND, MODE_REASON_TX_COMMAND);        
        ch6_counter = -TOY_COMMMAND_DELAY;
    }
    if (ch6_counter > TOY_LAND_COUNT && motors->armed() && ap.land_complete) {
        init_disarm_motors();
        ch6_counter = -TOY_COMMMAND_DELAY;
    }

    if (mode_change) {
        if (!gps_enable) {
            fence.enable(false);
            set_mode(TOY_NON_GPS_MODE, MODE_REASON_TX_COMMAND);
        } else {
            fence.enable(true);
            set_mode(TOY_GPS_MODE, MODE_REASON_TX_COMMAND);
        }
    }
}

/*
  this copes with a strange parkzone transmitter that changes scale on
  sticks when CH5 is high
 */
void Copter::toy_chan_fix(void)
{
#if TOY_CH5_RESCALE
    uint16_t ch5_in = hal.rcin->read(CH_5);
    if (ch5_in < 1700) {
        // nothing to do
        return;
    }
    for (uint8_t ch=CH_1; ch<= CH_4; ch++) {
        RC_Channel *chan = RC_Channels::rc_channel(ch);
        if (!chan) {
            continue;
        }
        int16_t v = chan->get_radio_in();
        if (v < 1100 || v > 1850) {
            continue;
        }
        v = ((v - 1500) * 1.3423) + 1500;
        chan->set_radio_in(v);
    }
#endif // TOY_CH5_RESCALE
}
