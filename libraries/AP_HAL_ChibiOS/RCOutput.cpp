
#include "RCOutput.h"
using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

#define PWM_CLK_FREQ        8000000
#define PWM_US_WIDTH_FROM_CLK(x) ((PWM_CLK_FREQ/1000000)*x)
const struct ChibiRCOutput::pwm_group ChibiRCOutput::pwm_group_list[] = 
{
    //Group 1 Config
    {   //Channels in the Group and respective mapping
        {PWM_CHAN_MAP(0) , PWM_CHAN_MAP(1) , PWM_CHAN_MAP(2) , PWM_CHAN_MAP(3)}, 
        //Group Initial Config
        {
          8000000,                                  /* 8MHz PWM clock frequency.   */
          160000,                                    /* Initial PWM period 20ms.     */
          NULL,
          {
           //Channel Config
           {PWM_OUTPUT_ACTIVE_HIGH, NULL},
           {PWM_OUTPUT_ACTIVE_HIGH, NULL},
           {PWM_OUTPUT_ACTIVE_HIGH, NULL},
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}
          },
          0,
          0
        },
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_PIXHAWK_CUBE || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_V2450
        &PWMD1
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_NUCLEO_F412
        &PWMD3
#endif
    }
};

void ChibiRCOutput::init()
{
    _num_groups = sizeof(pwm_group_list)/sizeof(pwm_group);
    for (uint8_t i = 0; i < _num_groups; i++ ) {
        //Start Pwm groups
        pwmStart(pwm_group_list[i].pwm_drv, &pwm_group_list[i].pwm_cfg);
    }
}

void ChibiRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    //check if the request spans accross any of the channel groups
    uint8_t update_mask = 0;
    uint32_t grp_ch_mask;
    // greater than 400 doesn't give enough room at higher periods for
    // the down pulse
    if (freq_hz > 400 && _output_mode != MODE_PWM_BRUSHED) {
        freq_hz = 400;
    }
    for (uint8_t i = 0; i < _num_groups; i++ ) {
        grp_ch_mask = pwm_group_list[i].ch_mask[0] | \
                      pwm_group_list[i].ch_mask[1] | \
                      pwm_group_list[i].ch_mask[2] | \
                      pwm_group_list[i].ch_mask[3];
        if ((grp_ch_mask & chmask) == grp_ch_mask) {
            update_mask |= grp_ch_mask;
            pwmChangePeriod(pwm_group_list[i].pwm_drv, 
                pwm_group_list[i].pwm_cfg.frequency/freq_hz);
        }
    }
    if (chmask != update_mask) {
        hal.console->printf("RCOutput: Failed to set PWM frequency req %x set %x\n", (unsigned)chmask, (unsigned)update_mask);
    }
}

uint16_t ChibiRCOutput::get_freq(uint8_t chan)
{
    for (uint8_t i = 0; i < _num_groups; i++ ) {
        for (uint8_t j = 0; j < 4; j++) {
            if (pwm_group_list[i].ch_mask[j] & 1<<chan) {
                return pwm_group_list[i].pwm_drv->config->frequency / pwm_group_list[i].pwm_drv->period;
            }
        }
    }
    return 50;
}

void ChibiRCOutput::enable_ch(uint8_t chan)
{
    for (uint8_t i = 0; i < _num_groups; i++ ) {
        for (uint8_t j = 0; j < 4; j++) {
            if ((pwm_group_list[i].ch_mask[j] & 1<<chan) && !(en_mask & 1<<chan)) {
                pwmEnableChannel(pwm_group_list[i].pwm_drv, j, PWM_US_WIDTH_FROM_CLK(900));
                en_mask |= 1<<chan;
                if(_output_mode == MODE_PWM_BRUSHED) {
                    period[chan] = 0;
                } else {
                    period[chan] = 900;
                }
            }
        }
    }
}

void ChibiRCOutput::disable_ch(uint8_t chan)
{
    for (uint8_t i = 0; i < _num_groups; i++ ) {
        for (uint8_t j = 0; j < 4; j++) {
            if (pwm_group_list[i].ch_mask[j] & 1<<chan) {
                pwmDisableChannel(pwm_group_list[i].pwm_drv, j);
                en_mask &= ~(1<<chan);
            }
        }
    }
}

void ChibiRCOutput::write(uint8_t chan, uint16_t period_us)
{
    for (uint8_t i = 0; i < _num_groups; i++ ) {
        for (uint8_t j = 0; j < 4; j++) {
            if (pwm_group_list[i].ch_mask[j] & 1<<chan) {
                _last_sent[chan] = period_us;
                if(_output_mode == MODE_PWM_BRUSHED) {
                    if (period_us <= _esc_pwm_min) {
                        period_us = 0;
                    } else if (period_us >= _esc_pwm_max) {
                        period_us = PWM_FRACTION_TO_WIDTH(pwm_group_list[i].pwm_drv, 1, 1);
                    } else {
                        period_us = PWM_FRACTION_TO_WIDTH(pwm_group_list[i].pwm_drv,\
                               (_esc_pwm_max - _esc_pwm_min), (period_us - _esc_pwm_min));
                    }
                    pwmEnableChannel(pwm_group_list[i].pwm_drv, j, period_us);
                } else {
                    pwmEnableChannel(pwm_group_list[i].pwm_drv, j, PWM_US_WIDTH_FROM_CLK(period_us));
                }
                period[chan] = period_us;
            }
        }
    }
}

uint16_t ChibiRCOutput::read(uint8_t chan)
{
    return period[chan];
}

void ChibiRCOutput::read(uint16_t* period_us, uint8_t len)
{
    memcpy(period_us, period, len*sizeof(uint16_t));
}

uint16_t ChibiRCOutput::read_last_sent(uint8_t chan)
{
    return _last_sent[chan];
}

void ChibiRCOutput::read_last_sent(uint16_t* period_us, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        period_us[i] = read_last_sent(i);
    }
}
/*
  setup output mode
 */
void ChibiRCOutput::set_output_mode(enum output_mode mode)
{
    _output_mode = mode;
}
