#include "Copter.h"

#if TOY_MODE_ENABLED == ENABLED

const AP_Param::GroupInfo ToyMode::Profile::var_info[] = {

    // @Param: A_RP_P
    // @DisplayName: Acro Roll and Pitch P gain
    // @Description: Converts pilot roll and pitch into a desired rate of rotation in ACRO and SPORT mode.  Higher values mean faster rate of rotation.
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("A_RP_P", 1, ToyMode::Profile, acro_rp_p, ACRO_RP_P),

    // @Param: A_YAW_P
    // @DisplayName: Acro Yaw P gain
    // @Description: Converts pilot yaw input into a desired rate of rotation in ACRO, Stabilize and SPORT modes.  Higher values mean faster rate of rotation.
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("A_YAW_P", 2, ToyMode::Profile, acro_yaw_p, ACRO_YAW_P),

    // @Param: A_BAL_RL
    // @DisplayName: Acro Balance Roll
    // @Description: rate at which roll angle returns to level in acro mode.  A higher value causes the vehicle to return to level faster.
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("A_BAL_R", 3, ToyMode::Profile, acro_balance_roll, ACRO_BALANCE_ROLL),

    // @Param: A_BAL_PI
    // @DisplayName: Acro Balance Pitch
    // @Description: rate at which pitch angle returns to level in acro mode.  A higher value causes the vehicle to return to level faster.
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("A_BAL_P", 4, ToyMode::Profile, acro_balance_pitch, ACRO_BALANCE_PITCH),

    // @Param: A_TRNR
    // @DisplayName: Acro Trainer
    // @Description: Type of trainer used in acro mode
    // @Values: 0:Disabled,1:Leveling,2:Leveling and Limited
    // @User: Advanced
    AP_GROUPINFO("A_TRNR", 5, ToyMode::Profile, acro_trainer, ACRO_TRAINER_LIMITED),

    // @Param: A_RP_XPO
    // @DisplayName: Acro Roll/Pitch Expo
    // @Description: Acro roll/pitch Expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -0.5 1.0
    // @User: Advanced
    AP_GROUPINFO("A_RP_XP", 6, ToyMode::Profile, acro_rp_expo, ACRO_RP_EXPO_DEFAULT),

    // @Param: A_Y_XPO
    // @DisplayName: Acro Yaw Expo
    // @Description: Acro yaw expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -0.5 1.0
    // @User: Advanced
    AP_GROUPINFO("A_Y_XP", 7, ToyMode::Profile, acro_y_expo, ACRO_Y_EXPO_DEFAULT),

    // @Param: A_THR_M
    // @DisplayName: Acro Thr Mid
    // @Description: Acro Throttle Mid
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("A_THR_M", 8, ToyMode::Profile, acro_thr_mid, ACRO_THR_MID_DEFAULT),

    // @Param: ANG_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all flight modes
    // @Units: cdeg
    // @Range: 1000 8000
    // @User: Advanced
    AP_GROUPINFO("ANG_MAX", 9, ToyMode::Profile, angle_max, DEFAULT_ANGLE_MAX),

    // @Param: XL_Y_MX
    // @DisplayName: Acceleration Max for Yaw
    // @Description: Maximum acceleration in yaw axis
    // @Units: cdeg/s/s
    // @Range: 0 72000
    // @Values: 0:Disabled, 9000:VerySlow, 18000:Slow, 36000:Medium, 54000:Fast
    // @Increment: 1000
    // @User: Advanced
    AP_GROUPINFO("XL_Y_MX",  10, ToyMode::Profile, accel_yaw_max, AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS),

    // @Param: XL_R_MX
    // @DisplayName: Acceleration Max for Roll
    // @Description: Maximum acceleration in roll axis
    // @Units: cdeg/s/s
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 30000:VerySlow, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("XL_R_MX", 11, ToyMode::Profile, accel_roll_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // @Param: XL_P_MX
    // @DisplayName: Acceleration Max for Pitch
    // @Description: Maximum acceleration in pitch axis
    // @Units: cdeg/s/s
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 30000:VerySlow, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("XL_P_MX", 12, ToyMode::Profile, accel_pitch_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // @Param: PLT_V_UP
    // @DisplayName: Pilot maximum vertical speed ascending
    // @Description: The maximum vertical ascending velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("PL_V_UP", 13, ToyMode::Profile, pilot_speed_up, PILOT_VELZ_MAX),

    // @Param: FEEL_RP
    // @DisplayName: RC Feel Roll/Pitch
    // @Description: RC feel for roll/pitch which controls vehicle response to user input with 0 being extremely soft and 100 being crisp
    // @Range: 0 100
    // @Increment: 10
    // @User: Standard
    // @Values: 0:Very Soft, 25:Soft, 50:Medium, 75:Crisp, 100:Very Crisp
    AP_GROUPINFO("FEEL_RP", 14, ToyMode::Profile, rc_feel_rp,  RC_FEEL_RP_MEDIUM),

    // @Param: WP_SPD
    // @DisplayName: Waypoint Horizontal Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
    // @Units: cm/s
    // @Range: 20 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("WP_SPD", 15, ToyMode::Profile, wp_speed_cms, WPNAV_WP_SPEED),

    // @Param: WP_RAD
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: cm
    // @Range: 10 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("WP_RAD", 16, ToyMode::Profile, wp_radius_cm, WPNAV_WP_RADIUS),

    // @Param: WP_V_UP
    // @DisplayName: Waypoint Climb Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
    // @Units: cm/s
    // @Range: 10 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("WP_V_UP", 17, ToyMode::Profile, wp_speed_up_cms, WPNAV_WP_SPEED_UP),

    // @Param: WP_V_DN
    // @DisplayName: Waypoint Descent Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
    // @Units: cm/s
    // @Range: 10 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("WP_V_DN", 18, ToyMode::Profile, wp_speed_down_cms, WPNAV_WP_SPEED_DOWN),

    // @Param: LTR_SPD
    // @DisplayName: Loiter Horizontal Maximum Speed
    // @Description: Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode
    // @Units: cm/s
    // @Range: 20 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("LTR_SPD", 19, ToyMode::Profile, loiter_speed_cms, WPNAV_LOITER_SPEED),

    // @Param: WP_XL
    // @DisplayName: Waypoint Acceleration 
    // @Description: Defines the horizontal acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("WP_XL", 20, ToyMode::Profile, wp_accel_cms, WPNAV_ACCELERATION),

    // @Param: WP_XL_Z
    // @DisplayName: Waypoint Vertical Acceleration
    // @Description: Defines the vertical acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("WP_XL_Z", 21, ToyMode::Profile, wp_accel_z_cms, WPNAV_WP_ACCEL_Z_DEFAULT),

    // @Param: LTR_JRK
    // @DisplayName: Loiter maximum jerk
    // @Description: Loiter maximum jerk in cm/s/s/s
    // @Units: cm/s/s/s
    // @Range: 500 5000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LTR_JRK", 22, ToyMode::Profile, loiter_jerk_max_cmsss, WPNAV_LOITER_JERK_MAX_DEFAULT),

    // @Param: LTR_MXA
    // @DisplayName: Loiter maximum acceleration
    // @Description: Loiter maximum acceleration in cm/s/s.  Higher values cause the copter to accelerate and stop more quickly.
    // @Units: cm/s/s
    // @Range: 100 981
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LTR_MXA", 23, ToyMode::Profile, loiter_accel_cmss, WPNAV_LOITER_ACCEL),

    // @Param: LTR_MNA
    // @DisplayName: Loiter minimum acceleration
    // @Description: Loiter minimum acceleration in cm/s/s. Higher values stop the copter more quickly when the stick is centered, but cause a larger jerk when the copter stops.
    // @Units: cm/s/s
    // @Range: 25 250
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LTR_MNA", 24, ToyMode::Profile, loiter_accel_min_cmss, WPNAV_LOITER_ACCEL_MIN),

    AP_GROUPEND

};
#endif
