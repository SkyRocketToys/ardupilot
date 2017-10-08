#pragma once
/*
 * Board identifier.
 */
#define BOARD_ST_PIXHAWK_CUBE
#define BOARD_NAME                  "Pixhawk Cube"
#define HAL_BOARD_INIT_HOOK_DEFINE void pixhawkCubeBoardInit(void);
#define HAL_BOARD_INIT_HOOK_CALL pixhawkCubeBoardInit();
#include "../fmuv3/board.h"