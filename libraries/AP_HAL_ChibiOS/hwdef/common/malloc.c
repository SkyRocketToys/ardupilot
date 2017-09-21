/*
  wrappers for allocation functions

  Relies on linker wrap options
 */

#include <stdio.h>
#include <string.h>
#include <hal.h>
#include <chheap.h>

#define MIN_ALIGNMENT 8

void *__wrap_malloc(size_t size)
{
    return chHeapAllocAligned(NULL, size, MIN_ALIGNMENT);
}

void *__wrap_calloc(size_t nmemb, size_t size)
{
    void *p = chHeapAllocAligned(NULL, nmemb*size, MIN_ALIGNMENT);
    if (p != NULL) {
        memset(p, 0, nmemb*size);
    }
    return p;
}

void __wrap_free(void *ptr)
{
    return chHeapFree(ptr);
}
