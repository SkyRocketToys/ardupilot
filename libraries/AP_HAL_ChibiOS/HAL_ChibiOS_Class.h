#pragma once

#include <AP_HAL/AP_HAL.h>

#include <AP_HAL_Empty/AP_HAL_Empty_Namespace.h>
#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS_Namespace.h>
#include <halconf.h>
#include "ch.h"
#include "hal.h"
#include "hrt.h"

class HAL_ChibiOS : public AP_HAL::HAL {
public:
    HAL_ChibiOS();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};
void hal_chibios_set_priority(uint8_t priority);

thread_t* get_main_thread(void);
