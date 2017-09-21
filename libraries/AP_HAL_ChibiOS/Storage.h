#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_Namespace.h"
#include <AP_Common/Bitmask.h>
#include <AP_FlashStorage/AP_FlashStorage.h>
#include "hwdef/common/flash.h"

#define CH_STORAGE_SIZE HAL_STORAGE_SIZE

// when using flash storage we use a small line size to make storage
// compact and minimise the number of erase cycles needed
#define CH_STORAGE_LINE_SHIFT 3

#define CH_STORAGE_LINE_SIZE (1<<CH_STORAGE_LINE_SHIFT)
#define CH_STORAGE_NUM_LINES (CH_STORAGE_SIZE/CH_STORAGE_LINE_SIZE)

class ChibiOS::ChibiStorage : public AP_HAL::Storage {
public:
    void init() {}
    void read_block(void *dst, uint16_t src, size_t n);
    void write_block(uint16_t dst, const void* src, size_t n);

    void _timer_tick(void);

private:
    volatile bool _initialised;
    void _storage_create(void);
    void _storage_open(void);
    void _mark_dirty(uint16_t loc, uint16_t length);
    uint8_t _buffer[CH_STORAGE_SIZE] __attribute__((aligned(4)));
    Bitmask _dirty_mask{CH_STORAGE_NUM_LINES};

    bool _flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length);
    bool _flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length);
    bool _flash_erase_sector(uint8_t sector);
    bool _flash_erase_ok(void);
    uint8_t _flash_page;
    bool _flash_failed;
    uint32_t _last_re_init_ms;
    
    AP_FlashStorage _flash{_buffer,
            stm32_flash_getpagesize(STORAGE_FLASH_PAGE),
            FUNCTOR_BIND_MEMBER(&ChibiStorage::_flash_write_data, bool, uint8_t, uint32_t, const uint8_t *, uint16_t),
            FUNCTOR_BIND_MEMBER(&ChibiStorage::_flash_read_data, bool, uint8_t, uint32_t, uint8_t *, uint16_t),
            FUNCTOR_BIND_MEMBER(&ChibiStorage::_flash_erase_sector, bool, uint8_t),
            FUNCTOR_BIND_MEMBER(&ChibiStorage::_flash_erase_ok, bool)};
    
    void _flash_load(void);
    void _flash_write(uint16_t line);
};
