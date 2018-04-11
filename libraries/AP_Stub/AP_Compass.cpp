#include <AP_Compass/AP_Compass.h>
#include <AP_Compass/Compass_learn.h>

#ifdef STUB_OUT_AP_Compass

const AP_Param::GroupInfo Compass::var_info[] = {
    AP_GROUPEND
};

Compass::Compass(void)
{
}

bool Compass::init()
{
    return true;
}

bool Compass::healthy(uint8_t i) const
{
    return false;
}

bool Compass::healthy(void) const
{
    return true;
}

bool Compass::is_calibrating() const
{
    return false;
}

bool Compass::configured(uint8_t i)
{
    return false;
}

bool Compass::read(void)
{
    return false;
}

bool Compass::configured()
{
    // sure, we're configured....
    return true;
}

void Compass::motor_compensation_type(const uint8_t comp_type)
{
}

void Compass::set_motor_compensation(uint8_t i, const Vector3f &motor_comp_factor)
{
}

void Compass::accumulate(void)
{
}

void Compass::save_motor_compensation()
{
}

void Compass::set_and_save_offsets(uint8_t i, const Vector3f &offsets)
{
}

void Compass::compass_cal_update()
{
}

void Compass::start_calibration_all(bool retry, bool autosave, float delay, bool autoreboot)
{
}

void Compass::set_initial_location(int32_t latitude, int32_t longitude)
{
}

float Compass::get_declination() const
{
    // danger, danger!
    return 0;
}

void Compass::set_declination(float radians, bool save_to_eeprom)
{
}

void Compass::cancel_calibration_all()
{
}

bool Compass::use_for_yaw(void) const
{
    return false;
}

bool Compass::use_for_yaw(uint8_t i) const
{
    return false;
}

MAV_RESULT Compass::handle_mag_cal_command(const mavlink_command_long_t &packet)
{
    return MAV_RESULT_UNSUPPORTED;
}

void Compass::send_mag_cal_progress(mavlink_channel_t chan)
{
}

void Compass::send_mag_cal_report(mavlink_channel_t chan)
{
}

bool Compass::consistent() const
{
    return false;
}

void Compass_PerMotor::calibration_start(void)
{
}

void Compass_PerMotor::calibration_update(void)
{
}

void Compass_PerMotor::calibration_end(void)
{
}




CompassCalibrator::CompassCalibrator()
{
}

Compass_PerMotor::Compass_PerMotor(Compass &_compass) :
    compass(_compass)
{
}

float Compass::calculate_heading(const Matrix3f &dcm_matrix, uint8_t i) const
{
    return 0.0;
}

void CompassLearn::update(void) {
}

CompassLearn::CompassLearn(AP_AHRS &_ahrs, Compass &_compass) :
    ahrs(_ahrs),
    compass(_compass)
{
}

MAV_RESULT Compass::fixed_mag_cal_field(const Vector3f &field) {
    return MAV_RESULT_UNSUPPORTED;
}

MAV_RESULT Compass::fixed_mag_cal(float roll_rad, float pitch_rad, float declination_deg, float inclination_deg, float intensity_mgauss, float yaw_deg)
{
    return MAV_RESULT_UNSUPPORTED;
}
#endif
