
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <assert.h>

#include "HAL_ChibiOS_Class.h"
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

static Empty::UARTDriver uartADriver;
static Empty::UARTDriver uartBDriver;
static Empty::UARTDriver uartCDriver;
static Empty::I2CDeviceManager i2cDeviceManager;
static Empty::SPIDeviceManager spiDeviceManager;
static Empty::AnalogIn analogIn;
static Empty::Storage storageDriver;
static Empty::GPIO gpioDriver;
static Empty::RCInput rcinDriver;
static Empty::RCOutput rcoutDriver;
static Empty::Scheduler schedulerInstance;
static Empty::Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;

HAL_ChibiOS::HAL_ChibiOS() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        nullptr,            /* no uartD */
        nullptr,            /* no uartE */
        nullptr,            /* no uartF */
        &i2cDeviceManager,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        nullptr
        )
{}

void HAL_ChibiOS::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init();
    uartA->begin(115200);

    callbacks->setup();
    scheduler->system_initialized();

    for (;;) {
        callbacks->loop();
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_ChibiOS hal;
    return hal;
}

#endif
