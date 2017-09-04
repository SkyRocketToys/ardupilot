#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_Namespace.h"
#include "Semaphores.h"

class ChibiOS::ChibiUtil : public AP_HAL::Util {
public:
    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
    AP_HAL::Semaphore *new_semaphore(void) override { return new ChibiOS::Semaphore; }
    uint32_t available_memory() override;
};
