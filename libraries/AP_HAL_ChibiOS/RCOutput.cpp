
#include "RCOutput.h"
using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

const struct ChibiRCOutput::pwm_group ChibiRCOutput::pwm_group_list[] = 
{
    //Group 1 Config
    {   //Channels in the Group and respective mapping
        {PWM_CHAN_MAP(0) , PWM_CHAN_MAP(1) , PWM_CHAN_MAP(2) , PWM_CHAN_MAP(3)}, 
        //Group Initial Config
        {
          1000000,                                  /* 1MHz PWM clock frequency.   */
          20000,                                    /* Initial PWM period 20ms.     */
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
        &PWMD1
    }
};

void ChibiRCOutput::init()
{
    _num_groups = sizeof(pwm_group_list)/sizeof(pwm_group);
    for (uint8_t i = 0; i < _num_groups; i++ ) {
        //Start Pwm groups
        pwmStart(pwm_group_list[i].pwm_drv, &pwm_group_list[i].pwm_cfg);
        //enable channel in the group
        for (uint8_t j =0; j < 4; j++) {
            if (pwm_group_list[i].ch_mask[j] != 0) {
                pwmEnableChannel(pwm_group_list[i].pwm_drv, j, 900);
            }
        }
    }
}

void ChibiRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    //check if the request spans accross any of the channel groups
    uint8_t update_mask = 0;
    uint32_t grp_ch_mask;

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
                pwmEnableChannel(pwm_group_list[i].pwm_drv, j, 900);
                en_mask |= 1<<chan;
                period[chan] = 900;
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
                pwmEnableChannel(pwm_group_list[i].pwm_drv, j, period_us);
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

