#include "shared_dma.h"

Shared_DMA::dma_locks *Shared_DMA::lock_head;
mutex_t Shared_DMA::list_mutex;

void Shared_DMA::init(void)
{
    chMtxObjectInit(&list_mutex);
}

void Shared_DMA::lock_dma(uint32_t dma_stream, void* ctx)
{
    dma_locks* lock = get_lock(dma_stream);
    if (lock != nullptr) {
        chBSemWait(&lock->sem);
        lock->last_ctx = ctx;
    }
}

void Shared_DMA::unlock_dma(uint32_t dma_stream, void* ctx)
{
    dma_locks* lock = get_lock(dma_stream);
    if (lock != nullptr) {
        if (lock->last_ctx == ctx) {
            lock->last_ctx = nullptr;
            chBSemSignal(&lock->sem);
        } else {
            //panic??
        }
    }
}

void Shared_DMA::unlock_dma_from_irq(uint32_t dma_stream, void* ctx)
{
    dma_locks* lock = get_lock(dma_stream);
    if (lock != nullptr) {
        if (lock->last_ctx == ctx) {
            lock->last_ctx = nullptr;
            chBSemSignalI(&lock->sem);
        }
    }
}

void Shared_DMA::register_dma(uint32_t dma_stream)
{
    dma_locks* temp;
    chMtxLock(&list_mutex);
    for (temp = lock_head; temp; temp = temp->next) {
        if (temp->dma_stream == dma_stream) {
            chMtxUnlock(&list_mutex);
            return;
        }
    }
    temp = new dma_locks;
    if (temp == nullptr) {
        //Do Panic Here
        return;
    }
    temp->dma_stream = dma_stream;
    chBSemObjectInit(&temp->sem, false);
    temp->next = lock_head;
    temp->last_ctx = nullptr;
    lock_head = temp;
    chMtxUnlock(&list_mutex);
}

void* Shared_DMA::get_ctx(uint32_t dma_stream)
{
    dma_locks* lock = get_lock(dma_stream);
    if (lock == nullptr) {
        return nullptr;
    } else {
        return lock->last_ctx;
    }
}

Shared_DMA::dma_locks* Shared_DMA::get_lock(uint32_t dma_stream)
{
    dma_locks *temp;
    for (temp = lock_head; temp; temp = temp->next) {
        if (temp->dma_stream == dma_stream) {
            return temp;
        }
    }
    return nullptr;
}
