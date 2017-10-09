#pragma once
/*
 * Board identifier.
 */
#define BOARD_ST_SKYVIPER_V2450
#define BOARD_NAME                  "Skyrocket V2450"
#define HAL_BOARD_INIT_HOOK_DEFINE void skyrocketV2450BoardInit(void);
#define HAL_BOARD_INIT_HOOK_CALL skyrocketV2450BoardInit();
#include "../fmuv3/board.h"