#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "AP_HAL_ChibiOS.h"

class ChibiOS::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore() {
        chMtxObjectInit(&_lock);
    }
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:
    mutex_t _lock;
};
#endif // CONFIG_HAL_BOARD
