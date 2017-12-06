#include "shared_dma.h"

/*
  code to handle sharing of DMA channels between peripherals
 */

Shared_DMA::dma_lock Shared_DMA::locks[SHARED_DMA_MAX_STREAM_ID];

void Shared_DMA::init(void)
{
    for (uint8_t i=0; i<SHARED_DMA_MAX_STREAM_ID; i++) {
        chBSemObjectInit(&locks[i].semaphore, false);
    }
}

// constructor
Shared_DMA::Shared_DMA(uint8_t _stream_id1,
                       uint8_t _stream_id2,
                       dma_allocate_fn_t _allocate,
                       dma_deallocate_fn_t _deallocate)
{
    stream_id1 = _stream_id1;
    stream_id2 = _stream_id2;
    allocate = _allocate;
    deallocate = _deallocate;
}

// lock the DMA channels
void Shared_DMA::lock(void)
{
    if (stream_id1 != SHARED_DMA_NONE) {
        chBSemWait(&locks[stream_id1].semaphore);
    }
    if (stream_id2 != SHARED_DMA_NONE) {
        chBSemWait(&locks[stream_id2].semaphore);
    }
    // see if another driver has DMA allocated. If so, call their
    // deallocation function
    if (stream_id1 != SHARED_DMA_NONE &&
        locks[stream_id1].obj && locks[stream_id1].obj != this) {
        locks[stream_id1].deallocate();
        locks[stream_id1].obj = nullptr;
    }
    if (stream_id2 != SHARED_DMA_NONE &&
        locks[stream_id2].obj && locks[stream_id2].obj != this) {
        locks[stream_id2].deallocate();
        locks[stream_id2].obj = nullptr;
    }
    if ((stream_id1 != SHARED_DMA_NONE && locks[stream_id1].obj == nullptr) ||
        (stream_id2 != SHARED_DMA_NONE && locks[stream_id2].obj == nullptr)) {
        // allocate the DMA channels and put our deallocation function in place
        allocate();
        if (stream_id1 != SHARED_DMA_NONE) {
            locks[stream_id1].deallocate = deallocate;
            locks[stream_id1].obj = this;
        }
        if (stream_id2 != SHARED_DMA_NONE) {
            locks[stream_id2].deallocate = deallocate;
            locks[stream_id2].obj = this;
        }
    }
}

// unlock the DMA channels
void Shared_DMA::unlock(void)
{
    if (stream_id2 != SHARED_DMA_NONE) {
        chBSemSignal(&locks[stream_id2].semaphore);        
    }
    if (stream_id1 != SHARED_DMA_NONE) {
        chBSemSignal(&locks[stream_id1].semaphore);
    }
}

// unlock the DMA channels from an IRQ
void Shared_DMA::unlock_from_IRQ(void)
{
    chSysLockFromISR();
    if (stream_id2 != SHARED_DMA_NONE) {
        chBSemSignalI(&locks[stream_id2].semaphore);        
    }
    if (stream_id1 != SHARED_DMA_NONE) {
        chBSemSignalI(&locks[stream_id1].semaphore);
    }
    chSysUnlockFromISR();
}
