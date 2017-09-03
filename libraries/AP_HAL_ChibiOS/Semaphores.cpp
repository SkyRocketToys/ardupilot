#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include "Semaphores.h"

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

bool Semaphore::give()
{
    chMtxUnlock(&_lock);
    return true;
}

bool Semaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        chMtxLock(&_lock);
        return true;
    }
    if (take_nonblocking()) {
        return true;
    }
    uint64_t start = AP_HAL::micros64();
    do {
        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms*1000);
    return false;
}

bool Semaphore::take_nonblocking()
{
    return chMtxTryLock(&_lock);
}

#endif // CONFIG_HAL_BOARD
