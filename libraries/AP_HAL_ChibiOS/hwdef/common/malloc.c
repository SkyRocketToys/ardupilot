/*
  wrappers for allocation functions

  Relies on linker wrap options

  Note that not all functions that have been wrapped are implemented
  here. The others are wrapped to ensure the function is not used
  without an implementation. If we need them then we can implement as
  needed.
 */

#include <stdio.h>
#include <string.h>
#include <hal.h>
#include <chheap.h>
#include <stdarg.h>

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

int __wrap_vsnprintf(char *str, size_t size, const char *fmt, va_list ap)
{
    return __real_vsnprintf(str, size, fmt, ap);
}

int __wrap_vasprintf(char **strp, const char *fmt, va_list ap)
{
    int len = __wrap_vsnprintf(NULL, 0, fmt, ap);
    if (len <= 0) {
        return -1;
    }
    char *buf = __wrap_calloc(len+1, 1);
    if (!buf) {
        return -1;
    }
    __wrap_vsnprintf(buf, len, fmt, ap);
    (*strp) = buf;
    return len;
}

int __wrap_asprintf(char **strp, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int ret = __wrap_vasprintf(strp, fmt, ap);
    va_end(ap);
    return ret;
}
