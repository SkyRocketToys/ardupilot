#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "Util.h"
#include <chheap.h>

using namespace ChibiOS;
/**
   how much free memory do we have in bytes.
*/
uint32_t ChibiUtil::available_memory(void)
{
    size_t totalp = 0;
    // get memory available on heap
    chHeapStatus(nullptr, &totalp, nullptr);

    // we also need to add in memory that is not yet allocated to the heap
    totalp += chCoreGetStatusX();
    
    return totalp;
}
#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
