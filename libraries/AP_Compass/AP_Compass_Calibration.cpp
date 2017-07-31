#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>

#include "AP_Compass.h"

extern AP_HAL::HAL& hal;

void
Compass::compass_cal_update()
{
    bool running = false;

    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        bool failure;
        _calibrator[i].update(failure);
        if (failure) {
            AP_Notify::events.compass_cal_failed = 1;
        }

        if (_calibrator[i].check_for_timeout()) {
            AP_Notify::events.compass_cal_failed = 1;
            cancel_calibration_all();
        }

        if (_calibrator[i].running()) {
            running = true;
        } else if (_cal_autosave && !_cal_saved[i] && _calibrator[i].get_status() == COMPASS_CAL_SUCCESS) {
            _accept_calibration(i);
        }
    }

    AP_Notify::flags.compass_cal_running = running;

    if (is_calibrating()) {
        _cal_has_run = true;
        return;
    } else if (_cal_has_run && _auto_reboot()) {
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }
}

bool
Compass::_start_calibration(uint8_t i, bool retry, float delay)
{
    if (!healthy(i)) {
        return false;
    }
    if (!use_for_yaw(i)) {
        return false;
    }
    if (!is_calibrating()) {
        AP_Notify::events.initiated_compass_cal = 1;
    }
    if (i == get_primary() && _state[i].external != 0) {
        _calibrator[i].set_tolerance(_calibration_threshold);
    } else {
        // internal compasses or secondary compasses get twice the
        // threshold. This is because internal compasses tend to be a
        // lot noisier
        _calibrator[i].set_tolerance(_calibration_threshold*2);
    }
    _cal_saved[i] = false;
    _calibrator[i].start(retry, delay, get_offsets_max());

    // disable compass learning both for calibration and after completion
    if (_learn.get() == 1) {
        _learn.set_and_save(0);
    }

    return true;
}

bool
Compass::_start_calibration_mask(uint8_t mask, bool retry, bool autosave, float delay, bool autoreboot)
{
    _cal_autosave = autosave;
    _compass_cal_autoreboot = autoreboot;

    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if ((1<<i) & mask) {
            if (!_start_calibration(i,retry,delay)) {
                _cancel_calibration_mask(mask);
                return false;
            }
        }
    }
    return true;
}

void
Compass::start_calibration_all(bool retry, bool autosave, float delay, bool autoreboot)
{
    _cal_autosave = autosave;
    _compass_cal_autoreboot = autoreboot;

    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        // ignore any compasses that fail to start calibrating
        // start all should only calibrate compasses that are being used
        _start_calibration(i,retry,delay);
    }
}

void
Compass::_cancel_calibration(uint8_t i)
{
    AP_Notify::events.initiated_compass_cal = 0;

    if (_calibrator[i].running() || _calibrator[i].get_status() == COMPASS_CAL_WAITING_TO_START) {
        AP_Notify::events.compass_cal_canceled = 1;
    }
    _cal_saved[i] = false;
    _calibrator[i].clear();
}

void
Compass::_cancel_calibration_mask(uint8_t mask)
{
    for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if((1<<i) & mask) {
            _cancel_calibration(i);
        }
    }
}

void
Compass::cancel_calibration_all()
{
    _cancel_calibration_mask(0xFF);
}

bool
Compass::_accept_calibration(uint8_t i)
{
    CompassCalibrator& cal = _calibrator[i];
    uint8_t cal_status = cal.get_status();

    if (_cal_saved[i] || cal_status == COMPASS_CAL_NOT_STARTED) {
        return true;
    } else if (cal_status == COMPASS_CAL_SUCCESS) {
        _cal_complete_requires_reboot = true;
        _cal_saved[i] = true;

        Vector3f ofs, diag, offdiag;
        cal.get_calibration(ofs, diag, offdiag);

        set_and_save_offsets(i, ofs);
        set_and_save_diagonals(i,diag);
        set_and_save_offdiagonals(i,offdiag);

        if (!is_calibrating()) {
            AP_Notify::events.compass_cal_saved = 1;
        }
        return true;
    } else {
        return false;
    }
}

bool
Compass::_accept_calibration_mask(uint8_t mask)
{
    bool success = true;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if ((1<<i) & mask) {
            if (!_accept_calibration(i)) {
                success = false;
            }
            _calibrator[i].clear();
        }
    }

    return success;
}

void
Compass::send_mag_cal_progress(mavlink_channel_t chan)
{
    uint8_t cal_mask = _get_cal_mask();

    for (uint8_t compass_id=0; compass_id<COMPASS_MAX_INSTANCES; compass_id++) {
        // ensure we don't try to send with no space available
        if (!HAVE_PAYLOAD_SPACE(chan, MAG_CAL_PROGRESS)) {
            return;
        }

        auto& calibrator = _calibrator[compass_id];
        uint8_t cal_status = calibrator.get_status();

        if (cal_status == COMPASS_CAL_WAITING_TO_START  ||
            cal_status == COMPASS_CAL_RUNNING_STEP_ONE ||
            cal_status == COMPASS_CAL_RUNNING_STEP_TWO) {
            uint8_t completion_pct = calibrator.get_completion_percent();
            auto& completion_mask = calibrator.get_completion_mask();
            Vector3f direction(0.0f,0.0f,0.0f);
            uint8_t attempt = _calibrator[compass_id].get_attempt();

            mavlink_msg_mag_cal_progress_send(
                chan,
                compass_id, cal_mask,
                cal_status, attempt, completion_pct, completion_mask,
                direction.x, direction.y, direction.z
            );
        }
    }
}

void Compass::send_mag_cal_report(mavlink_channel_t chan)
{
    uint8_t cal_mask = _get_cal_mask();

    for (uint8_t compass_id=0; compass_id<COMPASS_MAX_INSTANCES; compass_id++) {
        // ensure we don't try to send with no space available
        if (!HAVE_PAYLOAD_SPACE(chan, MAG_CAL_REPORT)) {
            return;
        }

        uint8_t cal_status = _calibrator[compass_id].get_status();
        if ((cal_status == COMPASS_CAL_SUCCESS ||
            cal_status == COMPASS_CAL_FAILED)) {
            float fitness = _calibrator[compass_id].get_fitness();
            Vector3f ofs, diag, offdiag;
            _calibrator[compass_id].get_calibration(ofs, diag, offdiag);
            uint8_t autosaved = _cal_saved[compass_id];

            mavlink_msg_mag_cal_report_send(
                chan,
                compass_id, cal_mask,
                cal_status, autosaved,
                fitness,
                ofs.x, ofs.y, ofs.z,
                diag.x, diag.y, diag.z,
                offdiag.x, offdiag.y, offdiag.z
            );
        }
    }
}

bool
Compass::is_calibrating() const
{
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        switch(_calibrator[i].get_status()) {
            case COMPASS_CAL_NOT_STARTED:
            case COMPASS_CAL_SUCCESS:
            case COMPASS_CAL_FAILED:
                break;
            default:
                return true;
        }
    }
    return false;
}

uint8_t
Compass::_get_cal_mask() const
{
    uint8_t cal_mask = 0;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if (_calibrator[i].get_status() != COMPASS_CAL_NOT_STARTED) {
            cal_mask |= 1 << i;
        }
    }
    return cal_mask;
}


/*
  handle an incoming MAG_CAL command
 */
uint8_t Compass::handle_mag_cal_command(const mavlink_command_long_t &packet)
{
    uint8_t result = MAV_RESULT_FAILED;

    switch (packet.command) {
    case MAV_CMD_DO_START_MAG_CAL: {
        result = MAV_RESULT_ACCEPTED;
        if (hal.util->get_soft_armed()) {
            hal.console->printf("Disarm for compass calibration\n");
            result = MAV_RESULT_FAILED;
            break;
        }
        if (packet.param1 < 0 || packet.param1 > 255) {
            result = MAV_RESULT_FAILED;
            break;
        }

        uint8_t mag_mask = packet.param1;
        bool retry = !is_zero(packet.param2);
        bool autosave = !is_zero(packet.param3);
        float delay = packet.param4;
        bool autoreboot = !is_zero(packet.param5);

        if (mag_mask == 0) { // 0 means all
            start_calibration_all(retry, autosave, delay, autoreboot);
        } else {
            if (!_start_calibration_mask(mag_mask, retry, autosave, delay, autoreboot)) {
                result = MAV_RESULT_FAILED;
            }
        }
        
        break;
    }

    case MAV_CMD_DO_ACCEPT_MAG_CAL: {
        result = MAV_RESULT_ACCEPTED;
        if(packet.param1 < 0 || packet.param1 > 255) {
            result = MAV_RESULT_FAILED;
            break;
        }
        
        uint8_t mag_mask = packet.param1;
        
        if (mag_mask == 0) { // 0 means all
            mag_mask = 0xFF;
        }
        
        if(!_accept_calibration_mask(mag_mask)) {
            result = MAV_RESULT_FAILED;
        }
        break;
    }
        
    case MAV_CMD_DO_CANCEL_MAG_CAL: {
        result = MAV_RESULT_ACCEPTED;
        if(packet.param1 < 0 || packet.param1 > 255) {
            result = MAV_RESULT_FAILED;
            break;
        }
        
        uint8_t mag_mask = packet.param1;
        
        if (mag_mask == 0) { // 0 means all
            cancel_calibration_all();
            break;
        }
        
        _cancel_calibration_mask(mag_mask);
        break;
    }
    }
    
    return result;
}


/*
  perform a magnetometer calibration assuming a fixed position in a known field

  This is an alternative magnetometer calibration method that involves
  placing the vehicle at a known yaw, with a known earths field
*/
uint8_t Compass::fixed_mag_cal_field(const Vector3f &mag_bf)
{
    if (hal.util->get_soft_armed()) {
        // refuse while armed
        return MAV_RESULT_FAILED;
    }

    /*
      we can't just adjust the offsets by the difference in field, as
      the ODI and DIA values affect the corrections applied by the
      offsets. We need to invert the correction matrix if we can, and
      use that to calculate the right offsets. If we can't invert it
      then we use it without the eliptical correction.
     */

    // read two samples to clear possible change of eliptical corrections
    read();
    hal.scheduler->delay(100);
    read();

    for (uint8_t i=0; i<get_count(); i++) {
        const Vector3f &diagonals = _state[i].diagonals.get();
        const Vector3f &offdiagonals = _state[i].offdiagonals.get();
        const Vector3f &field = get_field(i);
        const Vector3f &ofs = get_offsets(i);
        // form eliptical correction matrix
        Matrix3f mat(
            diagonals.x, offdiagonals.x, offdiagonals.y,
            offdiagonals.x,    diagonals.y, offdiagonals.z,
            offdiagonals.y, offdiagonals.z,    diagonals.z
            );
        Vector3f correction;
        if (mat.invert()) {
            Vector3f v1 = mat * field;
            Vector3f v2 = mat * mag_bf;
            correction = v2 - v1;
        } else {
            // is this the best we can do??
            correction = mag_bf - field;
        }
        Vector3f new_offset = ofs + correction;

        // save offsets
        set_offsets(i, new_offset);
        save_offsets(i);
    }

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Finished fixed calibration");

    return MAV_RESULT_ACCEPTED;
}

/*
  perform a magnetometer calibration assuming a fixed position in a known field

  This is an alternative magnetometer calibration method that involves
  placing the vehicle at a known yaw, with a known earths declination,
  inclination and field intensity.
*/
uint8_t Compass::fixed_mag_cal(const AP_AHRS &ahrs, float declination_deg, float inclination_deg, float intensity_mgauss, float yaw_deg)
{
    // create earth field
    Vector3f mag_ef(intensity_mgauss, 0.0, 0.0);
    Matrix3f R;
    R.from_euler(0.0f, -ToRad(inclination_deg), ToRad(declination_deg));
    mag_ef = R * mag_ef;

    Matrix3f dcm;
    dcm.from_euler(ahrs.roll, ahrs.pitch, ToRad(yaw_deg));
    
    // Rotate field into body frame
    Vector3f mag_bf = dcm.transposed() * mag_ef;
    
    return fixed_mag_cal_field(mag_bf);
}
