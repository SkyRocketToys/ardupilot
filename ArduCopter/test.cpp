#include "Copter.h"

#if CLI_ENABLED == ENABLED

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
static const struct Menu::command test_menu_commands[] = {
#if HIL_MODE == HIL_MODE_DISABLED
    {"baro",                MENU_FUNC(test_baro)},
#endif
    {"ins",                 MENU_FUNC(test_ins)},
    {"optflow",             MENU_FUNC(test_optflow)},
    {"relay",               MENU_FUNC(test_relay)},
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    {"shell", 				MENU_FUNC(test_shell)},
#endif
#if HIL_MODE == HIL_MODE_DISABLED
    {"rangefinder",         MENU_FUNC(test_rangefinder)},
#endif
};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

int8_t Copter::test_mode(uint8_t argc, const Menu::arg *argv)
{
    test_menu.run();
    return 0;
}

#if HIL_MODE == HIL_MODE_DISABLED
int8_t Copter::test_baro(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    init_barometer(true);

    while(1) {
        delay(100);
        read_barometer();

        if (!barometer.healthy()) {
            cliSerial->printf("not healthy\n");
        } else {
            cliSerial->printf("Alt: %0.2fm, Raw: %f Temperature: %.1f\n",
                                (double)(baro_alt / 100.0f),
                                (double)barometer.get_pressure(),
                                (double)barometer.get_temperature());
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
}
#endif


int8_t Copter::test_ins(uint8_t argc, const Menu::arg *argv)
{
    Vector3f gyro, accel;
    print_hit_enter();
    cliSerial->printf("INS\n");
    delay(1000);

    ahrs.init();
    ins.init(scheduler.get_loop_rate_hz());
    cliSerial->printf("...done\n");

    delay(50);

    while(1) {
        ins.update();
        gyro = ins.get_gyro();
        accel = ins.get_accel();

        float test = accel.length() / GRAVITY_MSS;

        cliSerial->printf("a %7.4f %7.4f %7.4f g %7.4f %7.4f %7.4f t %7.4f \n",
                (double)accel.x, (double)accel.y, (double)accel.z,
            (double)gyro.x, (double)gyro.y, (double)gyro.z,
            (double)test);

        delay(40);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

int8_t Copter::test_optflow(uint8_t argc, const Menu::arg *argv)
{
#if OPTFLOW == ENABLED
    if(optflow.enabled()) {
        cliSerial->printf("dev id: %d\t",(int)optflow.device_id());
        print_hit_enter();

        while(1) {
            delay(200);
            optflow.update();
            const Vector2f& flowRate = optflow.flowRate();
            cliSerial->printf("flowX : %7.4f\t flowY : %7.4f\t flow qual : %d\n",
                            (double)flowRate.x,
                            (double)flowRate.y,
                            (int)optflow.quality());

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    } else {
        cliSerial->printf("OptFlow: ");
        print_enabled(false);
    }
    return (0);
#else
    return (0);
#endif      // OPTFLOW == ENABLED
}

int8_t Copter::test_relay(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        cliSerial->printf("Relay on\n");
        relay.on(0);
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }

        cliSerial->printf("Relay off\n");
        relay.off(0);
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
/*
 *  run a debug shell
 */
int8_t Copter::test_shell(uint8_t argc, const Menu::arg *argv)
{
    hal.util->run_debug_shell(cliSerial);
    return 0;
}
#endif

#if HIL_MODE == HIL_MODE_DISABLED
/*
 *  test the rangefinders
 */
int8_t Copter::test_rangefinder(uint8_t argc, const Menu::arg *argv)
{
#if RANGEFINDER_ENABLED == ENABLED
	rangefinder.init();

    cliSerial->printf("RangeFinder: %d devices detected\n", rangefinder.num_sensors());

    print_hit_enter();
    while(1) {
        delay(100);
        rangefinder.update();

        for (uint8_t i=0; i<rangefinder.num_sensors(); i++) {
            cliSerial->printf("Dev%d: status %d distance_cm %d\n",
                    (int)i,
                    (int)rangefinder.status(i),
                    (int)rangefinder.distance_cm(i));
        }

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
#endif
    return (0);
}
#endif

void Copter::print_hit_enter()
{
    cliSerial->printf("Hit Enter to exit.\n\n");
}

#endif // CLI_ENABLED
