#pragma once
/*
 * Board identifier.
 */
#define BOARD_ST_PIXHAWK1
#define BOARD_NAME                  "Pixhawk1"
#define HAL_BOARD_INIT_HOOK_DEFINE void pixhawk1BoardInit(void);
#define HAL_BOARD_INIT_HOOK_CALL pixhawk1BoardInit();
#include "../fmuv3/board.h"
