#include <stdarg.h>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

#include "ch.h"
#include "hal.h"

extern const AP_HAL::HAL& hal;
extern "C"
{

__attribute__((weak))
void *__dso_handle;

__attribute__((weak))
int __errno;

__attribute__((weak))
void __cxa_pure_virtual() { while (1); } //TODO: Handle properly, maybe generate a traceback
}
namespace AP_HAL {

void init()
{
}

void panic(const char *errormsg, ...)
{
    va_list ap;

    va_start(ap, errormsg);
    vdprintf(1, errormsg, ap);
    va_end(ap);

    hal.scheduler->delay_microseconds(10000);
    while(1) {}
}

uint32_t micros()
{
    return micros64() & 0xFFFFFFFF;
}

uint32_t millis()
{
    return millis64() & 0xFFFFFFFF;
}

uint64_t micros64()
{
    return ST2US(chVTGetSystemTime());
;
}

uint64_t millis64()
{
    return micros64() / 1000;
}

} // namespace AP_HAL
