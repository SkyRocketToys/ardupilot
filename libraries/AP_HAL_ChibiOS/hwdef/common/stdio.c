/*
  wrappers for stdio functions

  Relies on linker wrap options

  Note that not all functions that have been wrapped are implemented
  here. The others are wrapped to ensure the function is not used
  without an implementation. If we need them then we can implement as
  needed.
 */

#include <stdio.h>
#include <string.h>
#include <hal.h>
#include <stdarg.h>

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

int __wrap_vprintf(const char *fmt, va_list arg)
{
  return chvprintf ((BaseSequentialStream*)&HAL_STDOUT_SERIAL, fmt, arg);
}

int __wrap_printf(const char *fmt, ...)
{
   va_list arg;
   int done;
 
   va_start (arg, fmt);
   done =   __wrap_vprintf(fmt, arg);
   va_end (arg);
 
   return done;
}
