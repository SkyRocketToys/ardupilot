#include "RCInput.h"
#include "hal.h"
#include "hwdef/common/ppm.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS


using namespace ChibiOS;
extern const AP_HAL::HAL& hal;
void ChibiRCInput::init()
{
    ppm_init(1000000, true);
    chMtxObjectInit(&rcin_mutex);
    _init = true;
}

bool ChibiRCInput::new_input()
{
    if (!_init) {
        return false;
    }
    chMtxLock(&rcin_mutex);
    bool valid = _rcin_timestamp_last_signal != _last_read;

    if (_override_valid) {
        // if we have RC overrides active, then always consider it valid
        valid = true;
    }
    _last_read = _rcin_timestamp_last_signal;
    _override_valid = false;
    chMtxUnlock(&rcin_mutex);

    return valid;
}

uint8_t ChibiRCInput::num_channels()
{
    if (!_init) {
        return 0;
    }
    chMtxLock(&rcin_mutex);
    uint8_t n = _num_channels;
    chMtxUnlock(&rcin_mutex);
    return n;
}

uint16_t ChibiRCInput::read(uint8_t channel)
{
    if (!_init) {
        return 0;
    }
    if (channel >= RC_INPUT_MAX_CHANNELS) {
        return 0;
    }
    chMtxLock(&rcin_mutex);
    if (_override[channel]) {
        uint16_t v = _override[channel];
        chMtxUnlock(&rcin_mutex);
        return v;
    }
    if (channel >=  _num_channels) {
        chMtxUnlock(&rcin_mutex);
        return 0;
    }
    uint16_t v = _rc_values[channel];
    chMtxUnlock(&rcin_mutex);
    return v;
}

uint8_t ChibiRCInput::read(uint16_t* periods, uint8_t len)
{
    if (!_init) {
        return false;
    }
 
    if (len > RC_INPUT_MAX_CHANNELS) {
        len = RC_INPUT_MAX_CHANNELS;
    }
    for (uint8_t i = 0; i < len; i++){
        periods[i] = read(i);
    }
    return len;
}

bool ChibiRCInput::set_overrides(int16_t *overrides, uint8_t len)
{
    if (!_init) {
        return false;
    }

    bool res = false;
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool ChibiRCInput::set_override(uint8_t channel, int16_t override)
{
    if (!_init) {
        return false;
    }

    if (override < 0) {
        return false; /* -1: no change. */
    }
    if (channel >= RC_INPUT_MAX_CHANNELS) {
        return false;
    }
    _override[channel] = override;
    if (override != 0) {
        _override_valid = true;
        return true;
    }
    return false;
}

void ChibiRCInput::clear_overrides()
{
    for (uint8_t i = 0; i < RC_INPUT_MAX_CHANNELS; i++) {
        set_override(i, 0);
    }
}


void ChibiRCInput::_timer_tick(void)
{
    if (!_init) {
        return;
    }
    if (ppm_available()) {
        chMtxLock(&rcin_mutex);
        _num_channels = ppm_read_bulk(_rc_values, RC_INPUT_MAX_CHANNELS);
        chMtxUnlock(&rcin_mutex);
    }
    // note, we rely on the vehicle code checking new_input()
    // and a timeout for the last valid input to handle failsafe
}
#endif //#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
