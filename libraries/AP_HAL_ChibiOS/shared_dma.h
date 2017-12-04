#pragma once

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "AP_HAL_ChibiOS.h"

class Shared_DMA;
class Shared_DMA
{
public:
    Shared_DMA() {}
    static void suspend_until_avail(uint32_t dma_stream);
    static bool dma_available(uint32_t dma_stream);
    static void lock_dma(uint32_t dma_stream, void* ctx);
    static void unlock_dma(uint32_t dma_stream, void* ctx);
    static void unlock_dma_from_irq(uint32_t dma_stream, void* ctx);
    static void* get_ctx(uint32_t dma_stream);
    static void register_dma(uint32_t dma_stream);
    struct dma_locks {
        binary_semaphore_t sem;
        uint32_t dma_stream;
        void* last_ctx;
        uint8_t dep_cnt;
        dma_locks *next;
    };
private:
    dma_locks *lock_head;
    static Shared_DMA* _s_instance;
    static dma_locks* get_lock(uint32_t dma_stream);
    static Shared_DMA* get_obj();
};
#endif