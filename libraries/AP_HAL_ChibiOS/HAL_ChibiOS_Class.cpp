
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <assert.h>

#include "HAL_ChibiOS_Class.h"
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS_Private.h>


static ChibiOS::ChibiUARTDriver uartADriver(0);
static ChibiOS::ChibiUARTDriver uartBDriver(1);
static ChibiOS::ChibiUARTDriver uartCDriver(2);
static ChibiOS::I2CDeviceManager i2cDeviceManager;
static ChibiOS::SPIDeviceManager spiDeviceManager;
static ChibiOS::ChibiAnalogIn analogIn;
static ChibiOS::ChibiStorage storageDriver;
static ChibiOS::ChibiGPIO gpioDriver;
static ChibiOS::ChibiRCInput rcinDriver;
static ChibiOS::ChibiRCOutput rcoutDriver;
static ChibiOS::ChibiScheduler schedulerInstance;
static ChibiOS::ChibiUtil utilInstance;
static Empty::OpticalFlow opticalFlowDriver;
#ifdef USE_POSIX
static FATFS SDC_FS; // FATFS object
#endif
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

static bool thread_running = false;        /**< Daemon status flag */
static thread_t* daemon_task;              /**< Handle of daemon task / thread */
bool chibios_ran_overtime;

extern const AP_HAL::HAL& hal;


/*
  set the priority of the main APM task
 */
void hal_chibios_set_priority(uint8_t priority)
{
    chSysLock();
    if ((daemon_task->prio == daemon_task->realprio) || (priority > daemon_task->prio)) {
      daemon_task->prio = priority;
    }
    daemon_task->realprio = priority;
    chSchRescheduleS();
    chSysUnlock();
}

thread_t* get_main_thread()
{
    return daemon_task;
}
/*
  this is called when loop() takes more than 1 second to run. If that
  happens then something is blocking for a long time in the main
  sketch - probably waiting on a low priority driver. Set the priority
  of the APM task low to let the driver run.
 */
static void loop_overtime(void *)
{
#if 0
    // disabled due to locking issue. When this fires we get an assert. 
    hal_chibios_set_priority(APM_OVERTIME_PRIORITY);
    chibios_ran_overtime = true;
#endif
}

static AP_HAL::HAL::Callbacks* g_callbacks;
THD_WORKING_AREA(_main_thread_wa, APM_MAIN_THREAD_STACK_SIZE);
static THD_FUNCTION(main_loop,arg)
{
    daemon_task = chThdGetSelfX();

    hal.uartA->begin(115200);
    hal.uartB->begin(38400);
    hal.uartC->begin(57600);
    hal.rcin->init();
    hal.rcout->init();
    hal.analogin->init();
    hal.gpio->init();
    hal.scheduler->init();

    /*
      run setup() at low priority to ensure CLI doesn't hang the
      system, and to allow initial sensor read loops to run
     */
    hal_chibios_set_priority(APM_STARTUP_PRIORITY);

    schedulerInstance.hal_initialized();

    g_callbacks->setup();
    hal.scheduler->system_initialized();

    thread_running = true;
    daemon_task->name = SKETCHNAME;
    virtual_timer_t loop_overtime_call;
    /*
      switch to high priority for main loop
     */
    chThdSetPriority(APM_MAIN_PRIORITY);

    while (true) {
        /*
          this ensures a tight loop waiting on a lower priority driver
          will eventually give up some time for the driver to run. It
          will only ever be called if a loop() call runs for more than
          0.1 second
         */
        chVTSet(&loop_overtime_call, US2ST(100000), loop_overtime, nullptr);

        g_callbacks->loop();

        if (chibios_ran_overtime) {
            /*
              we ran over 1s in loop(), and our priority was lowered
              to let a driver run. Set it back to high priority now.
             */
            hal_chibios_set_priority(APM_MAIN_PRIORITY);
            chibios_ran_overtime = false;
        }

        /*
          give up 250 microseconds of time, to ensure drivers get a
          chance to run.
         */
        hal.scheduler->delay_microseconds(250);
    }
    thread_running = false;
}

void HAL_ChibiOS::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    /*
     * System initializations.
     * - ChibiOS HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    hrt_init();
    //STDOUT Initialistion
    SerialConfig stdoutcfg =
    {
      HAL_STDOUT_BAUDRATE,
      0,
      USART_CR2_STOP1_BITS,
      0
    };
    sdStart((SerialDriver*)&HAL_STDOUT_SERIAL, &stdoutcfg);

    //Setup SD Card and Initialise FATFS bindings
    /*
     * Start SD Driver
     */
#ifdef USE_POSIX
    FRESULT err;
    sdcStart(&SDCD1, NULL);

    if(sdcConnect(&SDCD1) == HAL_FAILED) {
        printf("Err: Failed to initialize SDIO!\n");
    } else {
        err = f_mount(&SDC_FS, "/", 1);
        if (err != FR_OK) {
            printf("Err: Failed to mount SD Card!\n");
            sdcDisconnect(&SDCD1);
        } else {
            printf("Successfully mounted SDCard..\n");
        }
        //Create APM Directory
        mkdir("/APM", 0777);
    }
#endif
    assert(callbacks);
    g_callbacks = callbacks;

    chThdCreateStatic(_main_thread_wa,
                      sizeof(_main_thread_wa),
                      APM_MAIN_PRIORITY,     /* Initial priority.    */
                      main_loop,             /* Thread function.     */
                      nullptr);              /* Thread parameter.    */
    chThdExit(0);
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_ChibiOS hal_chibios;
    return hal_chibios;
}

#endif
