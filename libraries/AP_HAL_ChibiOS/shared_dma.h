#pragma once

#include "AP_HAL_ChibiOS.h"

class Shared_DMA
{
public:
    static void init(void);
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
        dma_locks *next;
    };
private:
    static mutex_t list_mutex;
    static dma_locks *lock_head;
    static dma_locks* get_lock(uint32_t dma_stream);
};
