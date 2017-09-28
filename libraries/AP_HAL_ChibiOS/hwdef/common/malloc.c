/*
  wrappers for allocation functions

  Relies on linker wrap options

  Note that not all functions that have been wrapped are implemented
  here. The others are wrapped to ensure the function is not used
  without an implementation. If we need them then we can implement as
  needed.
 */

#include <posix.h>
#include <string.h>
#include <hal.h>
#include <chheap.h>
#include <stdarg.h>

#define MIN_ALIGNMENT 8

void *malloc(size_t size)
{
    return chHeapAllocAligned(NULL, size, MIN_ALIGNMENT);
}

void *calloc(size_t nmemb, size_t size)
{
    void *p = chHeapAllocAligned(NULL, nmemb*size, MIN_ALIGNMENT);
    if (p != NULL) {
        memset(p, 0, nmemb*size);
    }
    return p;
}

void free(void *ptr)
{
    if(ptr != NULL) {
        chHeapFree(ptr);
    }
}
