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
    uint16_t read_last_sent(uint8_t ch) override;
    void     read_last_sent(uint16_t* period_us, uint8_t len) override;
    void     set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) override {
        _esc_pwm_min = min_pwm;
        _esc_pwm_max = max_pwm;
    }
    void set_output_mode(enum output_mode mode) override;

    void     cork(void) override {}
    void     push(void) override {}
private:
    struct pwm_group {
        uint32_t ch_mask[4];
        PWMConfig pwm_cfg;
        PWMDriver* pwm_drv;
    };
    enum output_mode _output_mode = MODE_PWM_NORMAL;

    uint32_t en_mask;
    static const pwm_group pwm_group_list[];
    uint16_t _esc_pwm_min;
    uint16_t _esc_pwm_max;
    uint8_t _num_groups;
    uint16_t period[16] = {900};
    uint16_t _last_sent[16] = {900};
};
