#include "shared_dma.h"

Shared_DMA *Shared_DMA::_s_instance = nullptr;

Shared_DMA* Shared_DMA::get_obj()
{
    if (_s_instance == nullptr) {
        _s_instance = new Shared_DMA;
        _s_instance->lock_head = nullptr;
    }
    return _s_instance;
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
        chSysLock();
        chSysUnlock();
        if (lock->last_ctx == ctx) {
            chBSemSignal(&lock->sem);
            lock->last_ctx = nullptr;
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
            chBSemSignalI(&lock->sem);
            lock->last_ctx = nullptr;
        }
    }
}

void Shared_DMA::register_dma(uint32_t dma_stream)
{
    dma_locks** temp = &get_obj()->lock_head;
    while(*temp != nullptr) {
        if ((*temp)->dma_stream == dma_stream) {
            //already registered
            (*temp)->dep_cnt++;
            return;
        }
        temp = &(*temp)->next;
    }
    *temp = new dma_locks;
    if (*temp == nullptr) {
        //Do Panic Here
        return;
    }
    (*temp)->dma_stream = dma_stream;
    chBSemObjectInit(&(*temp)->sem, false);
    (*temp)->next == nullptr;
    (*temp)->dep_cnt = 0;
}

void* Shared_DMA::get_ctx(uint32_t dma_stream)
{
    dma_locks* lock =get_lock(dma_stream);
    if (lock == nullptr) {
        return nullptr;
    } else {
        return lock->last_ctx;
    }
}

Shared_DMA::dma_locks* Shared_DMA::get_lock(uint32_t dma_stream)
{
    dma_locks* temp = get_obj()->lock_head;
    while(temp != nullptr) {
        if (temp->dma_stream == dma_stream) {
            return temp;
        }
        temp = temp->next;
    }
    return nullptr;
}