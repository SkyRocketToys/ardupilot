#pragma once

#include "AP_HAL_ChibiOS.h"
#include "ch.h"
#include "hal.h"
#define PWM_CHAN_MAP(n) (1 << n)

class ChibiOS::ChibiRCOutput : public AP_HAL::RCOutput {
public:
    void     init();
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);
    void     cork(void) override {}
    void     push(void) override {}
private:
    struct pwm_group {
        uint32_t ch_mask[4];
        PWMConfig pwm_cfg;
        PWMDriver* pwm_drv;
    };
    static const pwm_group pwm_group_list[];
    uint8_t _num_groups;
    uint16_t period[16] = {900};
};
