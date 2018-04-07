#pragma once

/*
  class to support "toy" mode for simplified user interaction for
  large volume consumer vehicles
 */
#define MAX_NUM_PROFILES 2
class ToyMode;
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

    // adjust throttle for throttle takeoff
    void throttle_adjust(float &throttle_control);
    void takeoff_throttle_adjust(float &throttle_control);

    // handle mavlink message
    void handle_message(mavlink_message_t *msg);

    void load_test_run(void);
    
    static const struct AP_Param::GroupInfo var_info[];

    class Profile {
        friend class ToyMode;
        public:
            Profile()
            {
                AP_Param::setup_object_defaults(this, var_info);
            }

            static const struct AP_Param::GroupInfo var_info[];

        private:
            AP_Int8                 mode;
            AP_Float                acro_rp_p;
            AP_Float                acro_yaw_p;
            AP_Float                acro_balance_roll;
            AP_Float                acro_balance_pitch;
            AP_Int8                 acro_trainer;
            AP_Float                acro_rp_expo;
            AP_Float                acro_y_expo;
            AP_Float                acro_thr_mid;
            AP_Int16                angle_max;
            AP_Float                accel_roll_max;
            AP_Float                accel_pitch_max;
            AP_Float                accel_yaw_max;
            AP_Int16                pilot_speed_up;
            AP_Int8                 rc_feel_rp;
        
            AP_Float                loiter_speed_cms;
            AP_Float                loiter_jerk_max_cmsss;
            AP_Float                loiter_accel_cmss;
            AP_Float                loiter_accel_min_cmss;
            AP_Float                wp_speed_cms;
            AP_Float                wp_speed_up_cms;
            AP_Float                wp_speed_down_cms;
            AP_Float                wp_radius_cm;
            AP_Float                wp_accel_cms;
            AP_Float                wp_accel_z_cms;
    };
private:

    void trim_update(void);
    void action_arm(void);
    void blink_update(void);
    void param_update(void);
    void send_named_int(const char *name, int32_t value);
    bool set_and_remember_mode(control_mode_t mode, mode_reason_t reason);

    void thrust_limiting(float *thrust, uint8_t num_motors);
    void arm_check_compass(void);

    // work out type of button setup
    bool is_v2450_buttons(void) const {
        return enable == 1;
    }
    bool is_f412_buttons(void) const {
        return enable == 2;
    }
    
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
        ACTION_ARM_LAND_RTL = 20,
        ACTION_TOGGLE_SIMPLE = 21,
        ACTION_TOGGLE_SSIMPLE = 22,
        ACTION_LOAD_TEST = 23,
        ACTION_MODE_FLOW = 24,
        ACTION_TOGGLE_PROFILE = 25,
        ACTION_LAUNCH_LAND = 26,
    };

    enum toy_action last_action;

    // these are bitmask indexes for TMODE_FLAGS
    enum toy_flags {
        FLAG_THR_DISARM     = 1<<0,  // disarm on low throttle
        FLAG_THR_ARM        = 1<<1,  // arm on high throttle
        FLAG_UPGRADE_LOITER = 1<<2,  // auto upgrade from ALT_HOLD to LOITER
        FLAG_RTL_CANCEL     = 1<<3,  // cancel RTL on large stick input
        FLAG_ACCEL_CAL      = 1<<4,  // accel cal on mode+stunt for 2s
        FLAG_TXMODE_CHANGE  = 1<<5,  // change TX mode on mode+stick for 2s
    };

    enum blink_patterns {
        BLINK_FULL   = 0xFFFF,
        BLINK_OFF    = 0x0000,
        BLINK_1      = 0xBFFF,
        BLINK_2      = 0xAFFF,
        BLINK_3      = 0xABFF,
        BLINK_4      = 0xAAFF,
        BLINK_6      = 0xAAAF,
        BLINK_8      = 0xAAAA,
        BLINK_NO_RX  = 0x1111,
        BLINK_SLOW_1 = 0xF0FF,
        BLINK_VSLOW  = 0xF000,
        BLINK_MED_1  = 0xF0F0,
    };

    bool done_first_update;
    AP_Int8 enable;
    AP_Int8 primary_mode[2];
    AP_Int8 actions[10];
    AP_Int8 trim_auto;
    AP_Int8 profile_id;
    AP_Int16 flags;
    AP_Float takeoff_time;
    AP_Float takeoff_delay;
    AP_Int16 land_throttle;
    int8_t last_profile_id = -1;

    /*
      a table mapping profile variable names to entries in the profile
      objects. We use the offset within the object
     */
    struct ProfileTable {
        uint32_t param_offset;
        const char *name;
        ap_var_type var_type;
    };

#define PROFILE_ENTRY(var_p, var_name, var_type) { offsetof(Profile, var_p), var_name, var_type }
    static const uint8_t num_profile_vars = 24;
    const ProfileTable profile_table[num_profile_vars] = {
        PROFILE_ENTRY(acro_rp_p,             "ACRO_RP_P", AP_PARAM_FLOAT),
        PROFILE_ENTRY(acro_yaw_p,            "ACRO_YAW_P", AP_PARAM_FLOAT),
        PROFILE_ENTRY(acro_balance_roll,     "ACRO_BAL_ROLL", AP_PARAM_FLOAT),
        PROFILE_ENTRY(acro_balance_pitch,    "ACRO_BAL_PITCH", AP_PARAM_FLOAT),
        PROFILE_ENTRY(acro_trainer,          "ACRO_TRAINER", AP_PARAM_INT8),
        PROFILE_ENTRY(acro_rp_expo,          "ACRO_RP_EXPO", AP_PARAM_FLOAT),
        PROFILE_ENTRY(acro_y_expo,           "ACRO_Y_EXPO", AP_PARAM_FLOAT),
        PROFILE_ENTRY(acro_thr_mid,          "ACRO_THR_MID", AP_PARAM_FLOAT),
        PROFILE_ENTRY(angle_max,             "ANGLE_MAX", AP_PARAM_INT16),
        PROFILE_ENTRY(accel_roll_max,        "ATC_ACCEL_R_MAX", AP_PARAM_FLOAT),
        PROFILE_ENTRY(accel_pitch_max,       "ATC_ACCEL_P_MAX", AP_PARAM_FLOAT),
        PROFILE_ENTRY(accel_yaw_max,         "ATC_ACCEL_Y_MAX", AP_PARAM_FLOAT),
        PROFILE_ENTRY(pilot_speed_up,        "PILOT_SPEED_UP", AP_PARAM_INT16),
        PROFILE_ENTRY(rc_feel_rp,            "RC_FEEL_RP", AP_PARAM_INT8),
        PROFILE_ENTRY(loiter_speed_cms,      "WPNAV_LOIT_SPEED", AP_PARAM_FLOAT),
        PROFILE_ENTRY(loiter_jerk_max_cmsss, "WPNAV_LOIT_JERK", AP_PARAM_FLOAT),
        PROFILE_ENTRY(loiter_accel_cmss,     "WPNAV_LOIT_MAXA", AP_PARAM_FLOAT),
        PROFILE_ENTRY(loiter_accel_min_cmss, "WPNAV_LOIT_MINA", AP_PARAM_FLOAT),
        PROFILE_ENTRY(wp_speed_cms,          "WPNAV_SPEED", AP_PARAM_FLOAT),
        PROFILE_ENTRY(wp_speed_up_cms,       "WPNAV_SPEED_UP", AP_PARAM_FLOAT),
        PROFILE_ENTRY(wp_speed_down_cms,     "WPNAV_SPEED_DN", AP_PARAM_FLOAT),
        PROFILE_ENTRY(wp_radius_cm,          "WPNAV_RADIUS", AP_PARAM_FLOAT),
        PROFILE_ENTRY(wp_accel_cms,          "WPNAV_ACCEL", AP_PARAM_FLOAT),
        PROFILE_ENTRY(wp_accel_z_cms,        "WPNAV_ACCEL_Z", AP_PARAM_FLOAT),
    };
    AP_Param *profile_ptrs[num_profile_vars];
    
    Profile _var_info_profile[MAX_NUM_PROFILES];
    bool ptr_to_param_loaded;

    struct {
        uint32_t start_ms;
        uint16_t chan[4];
    } trim;
    
    uint32_t power_counter;
    uint32_t throttle_low_counter;
    uint32_t throttle_high_counter;
    uint16_t last_ch5;
    bool last_mode_button;
    uint8_t last_mode_choice;
    int32_t mode_press_counter;
    int32_t right_press_counter;
    int32_t accel_cal_counter;
    uint32_t accel_cal_time_ms;
    int32_t txmode_change_counter;
    uint32_t txmode_change_time_ms;
    bool ignore_mode_button_change;
    int16_t throttle_mid = 500;
    uint32_t throttle_arm_ms;
    bool upgrade_to_loiter;
    uint32_t last_action_ms;
    uint32_t reset_turtle_start_ms;

    // time when we were last told we are recording video
    uint32_t last_video_ms;
    
    // current blink indexes
    uint16_t red_blink_pattern;
    uint16_t green_blink_pattern;
    uint8_t red_blink_index;
    uint8_t green_blink_index;
    uint16_t red_blink_count;
    uint16_t green_blink_count;
    uint8_t blink_disarm;

    // are we in a user takeoff?
    uint32_t takeoff_start_ms;

    // are we in a user land in FLOWHOLD mode?
    bool user_land;

    struct {
        AP_Float throttle_threshold;
        AP_Float climbrate_threshold;
    } obs;

    struct {
        AP_Float volt_min;
        AP_Float volt_max;
        AP_Float thrust_min;
        AP_Float thrust_max;
    } filter;
    
    // low-pass voltage
    float filtered_voltage = 4.0;

    uint8_t motor_log_counter;

    // remember the last mode we set
    control_mode_t last_set_mode = LOITER;

    struct load_data {
        uint16_t m[4];
    };

    enum load_type {
        LOAD_TYPE_CONSTANT=0,
        LOAD_TYPE_LOG1=1,
        LOAD_TYPE_LOG2=2,
    };
    
    struct {
        bool running;
        uint32_t row;
        uint8_t filter_counter;
        AP_Float load_mul;
        AP_Int8  load_filter;
        AP_Int8  load_type;
    } load_test;
    
    static const struct load_data load_data1[];

    AP_Radio *radio;
};
