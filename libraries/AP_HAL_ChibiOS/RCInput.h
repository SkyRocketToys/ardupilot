#pragma once

#include "AP_HAL_ChibiOS.h"
#include <AP_Radio/AP_Radio.h>


#ifndef RC_INPUT_MAX_CHANNELS
#define RC_INPUT_MAX_CHANNELS 18
#endif

class ChibiOS::ChibiRCInput : public AP_HAL::RCInput {
public:
    void init() override;
    bool new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;

    int16_t get_rssi(void) override {
        return _rssi;
    }
        
    
    bool set_overrides(int16_t *overrides, uint8_t len) override;
    bool set_override(uint8_t channel, int16_t override) override;
    void clear_overrides() override;

    void _timer_tick(void);
    bool rc_bind(int dsmMode) override;

private:
    /* override state */
    uint16_t _override[RC_INPUT_MAX_CHANNELS];
    uint16_t _rc_values[RC_INPUT_MAX_CHANNELS] = {0};

    uint64_t _last_read;
    bool _override_valid;
    uint8_t _num_channels;
    mutex_t rcin_mutex;
    int16_t _rssi = -1;
    uint32_t _rcin_timestamp_last_signal;
    bool _init;
#ifdef HAL_RCINPUT_WITH_AP_RADIO
    AP_Radio *radio;
    uint32_t last_radio_us;
#endif
};
