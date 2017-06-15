/*
  per-motor compass compensation
 */

#include "AP_Compass.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

// return current scaled motor output
float Compass::per_motor_output(uint8_t motor)
{
    // this currently assumes first 4 channels. 
    uint16_t pwm = hal.rcout->read_last_sent(motor);

    // get 0 to 1 motor demand
    float output = (hal.rcout->scale_esc_to_unity(pwm)+1) * 0.5;

    if (output <= 0) {
        return 0;
    }
    
    // scale for voltage
    output *= _per_motor.voltage;

    // apply expo correction
    output = powf(output, _per_motor.expo);
    return output;
}

// per-motor calibration update
void Compass::per_motor_calibration_start(void)
{
    for (uint8_t i=0; i<4; i++) {
        _per_motor.field_sum[i].zero();
        _per_motor.output_sum[i] = 0;
        _per_motor.count[i] = 0;
        _per_motor.start_ms[i] = 0;
    }
    _per_motor.base_field = get_field(0);
    _per_motor.running = true;

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "PMOT starting");
}

// per-motor calibration update
void Compass::per_motor_calibration_update(void)
{
    uint32_t now = AP_HAL::millis();
    
    // accumulate per-motor sums
    for (uint8_t i=0; i<4; i++) {
        float output = per_motor_output(i);

        if (output <= 0) {
            // motor is off
            _per_motor.start_ms[i] = 0;
            continue;
        }
        if (_per_motor.start_ms[i] == 0) {
            _per_motor.start_ms[i] = now;
        }
        if (now - _per_motor.start_ms[i] < 500) {
            // motor must run for 0.5s to settle
            continue;
        }

        // accumulate a sample
        _per_motor.field_sum[i] += get_field(0);
        _per_motor.output_sum[i] += output;
        _per_motor.count[i]++;
    }
}

// calculate per-motor calibration values
void Compass::per_motor_calibration_end(void)
{
    for (uint8_t i=0; i<4; i++) {
        if (_per_motor.count[i] == 0) {
            continue;
        }

        // calculate effective output
        float output = _per_motor.output_sum[i] / _per_motor.count[i];

        // calculate amount that field changed from base field
        Vector3f field_change = _per_motor.base_field - (_per_motor.field_sum[i] / _per_motor.count[i]);
        if (output <= 0) {
            continue;
        }
        
        Vector3f c = field_change / output;
        _per_motor.compensation[i].set_and_save(c);
    }
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "PMOT finished");

    // enable per-motor compensation
    _motor_comp_type.set_and_save(AP_COMPASS_MOT_COMP_PER_MOTOR);
    
    _per_motor.running = false;
}

/*
  calculate total offset for per-motor compensation
 */
void Compass::per_motor_compensate(Vector3f &offset)
{
    offset.zero();

    if (_per_motor.running) {
        // don't compensate while calibrating
        return;
    }

    for (uint8_t i=0; i<4; i++) {
        float output = per_motor_output(i);

        const Vector3f &c = _per_motor.compensation[i].get();

        offset += c * output;
    }
}
