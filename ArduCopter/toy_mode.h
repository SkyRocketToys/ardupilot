#pragma once

/*
  class to support "toy" mode for simplified user interaction for
  large volume consumer vehicles
 */

class ToyMode
{
public:
    friend class Copter;

    ToyMode();
    bool enabled(void) const {
        return enable.get() != 0;
    }
    
    void update(void);

    // get throttle mid-point
    int16_t get_throttle_mid(void) {
        return throttle_mid;
    }
    
    static const struct AP_Param::GroupInfo var_info[];
    
private:

    void trim_sticks(void);
    
    enum toy_action {
        ACTION_NONE         = 0,
        ACTION_TAKE_PHOTO   = 1,
        ACTION_TOGGLE_VIDEO = 2,
        ACTION_MODE_ACRO    = 3,
        ACTION_MODE_ALTHOLD = 4,
        ACTION_MODE_AUTO    = 5,
        ACTION_MODE_LOITER  = 6,
        ACTION_MODE_RTL     = 7,
        ACTION_MODE_CIRCLE  = 8,
        ACTION_MODE_LAND    = 9,
        ACTION_MODE_DRIFT   = 10,
        ACTION_MODE_SPORT   = 11,
        ACTION_MODE_AUTOTUNE= 12,
        ACTION_MODE_POSHOLD = 13,
        ACTION_MODE_BRAKE   = 14,
        ACTION_MODE_THROW   = 15,
        ACTION_MODE_FLIP    = 16,
        ACTION_MODE_STAB    = 17,
        ACTION_DISARM       = 18,
        ACTION_TOGGLE_MODE  = 19,
    };
    
    bool first_update;
    AP_Int8 enable;
    AP_Int8 primary_mode[2];
    AP_Int8 actions[8];
    AP_Int8 trim_arm;
    
    int32_t arm_counter;
    uint32_t power_counter;
    uint32_t throttle_low_counter;
    uint16_t last_ch5;
    uint8_t last_mode_choice;
    int32_t left_press_counter;
    bool ignore_left_change;
    int16_t throttle_mid = 500;
};
