#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "Util.h"

using namespace ChibiOS;
/**
   how much free memory do we have in bytes.
*/
uint32_t ChibiUtil::available_memory(void)
{
    return chCoreGetStatusX();
}
#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
