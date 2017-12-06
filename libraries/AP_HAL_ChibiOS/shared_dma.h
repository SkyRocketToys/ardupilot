#pragma once

#include "AP_HAL_ChibiOS.h"

#define SHARED_DMA_MAX_STREAM_ID (8*2)

// DMA stream ID for stream_id2 when only one is needed
#define SHARED_DMA_NONE 255

class Shared_DMA
{
public:
    FUNCTOR_TYPEDEF(dma_allocate_fn_t, void);
    FUNCTOR_TYPEDEF(dma_deallocate_fn_t, void);

    // the use of two stream IDs is for support of peripherals that
    // need both a RX and TX DMA channel
    Shared_DMA(uint8_t stream_id1, uint8_t stream_id2,
               dma_allocate_fn_t allocate,
               dma_allocate_fn_t deallocate);

    // initialise the stream locks
    static void init(void);
    
    // blocking lock call
    void lock(void);

    // unlock call. The DMA channel will not be immediately
    // deallocated. Instead it will be deallocated if another driver
    // needs it
    void unlock(void);

    // unlock call from an IRQ
    void unlock_from_IRQ(void);
    
private:
    dma_allocate_fn_t allocate;
    dma_allocate_fn_t deallocate;
    uint8_t stream_id1;
    uint8_t stream_id2;

    static struct dma_lock {
        // semaphore to ensure only one peripheral uses a DMA channel at a time
        binary_semaphore_t semaphore;

        // a de-allocation function that is called to release an existing user
        dma_deallocate_fn_t deallocate;

        // point to object that holds the allocation, if allocated
        Shared_DMA *obj;
    } locks[SHARED_DMA_MAX_STREAM_ID];
};
