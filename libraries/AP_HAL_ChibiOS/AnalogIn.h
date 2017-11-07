#pragma once

#include "AP_HAL_ChibiOS.h"

#define ANALOG_MAX_CHANNELS 16


class ChibiOS::ChibiAnalogSource : public AP_HAL::AnalogSource {
public:
    friend class ChibiOS::ChibiAnalogIn;
    ChibiAnalogSource(int16_t pin, float initial_value);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric();
    void set_stop_pin(uint8_t p) {}
    void set_settle_time(uint16_t settle_time_ms) {}

private:
    // what value it has
    int16_t _pin;
    float _value;
    float _value_ratiometric;
    float _latest_value;
    uint8_t _sum_count;
    float _sum_value;
    float _sum_ratiometric;
    void _add_value(float v, float vcc5V);
    float _pin_scaler();
};

class ChibiOS::ChibiAnalogIn : public AP_HAL::AnalogIn {
public:
    ChibiAnalogIn();
    void init() override;
    AP_HAL::AnalogSource* channel(int16_t pin) override;
    void _timer_tick(void);
    float board_voltage(void) override { return _board_voltage; }
    float servorail_voltage(void) override { return _servorail_voltage; }
    uint16_t power_status_flags(void) override { return _power_flags; }

private:
    void read_adc(uint32_t *val);
    int _adc_fd = -1;
    int _battery_handle;
    int _servorail_handle;
    int _system_power_handle;
    uint64_t _battery_timestamp;
    uint64_t _servorail_timestamp;
    ChibiOS::ChibiAnalogSource* _channels[ANALOG_MAX_CHANNELS];

    uint32_t _last_run;
    float _board_voltage;
    float _servorail_voltage;
    uint16_t _power_flags;
    ADCConversionGroup adcgrpcfg;
};
