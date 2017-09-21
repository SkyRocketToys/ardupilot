
#include "GPIO.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

using namespace ChibiOS;
static uint32_t _gpio_tab[]  = {
    LINE_LED1,
#if CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_CHIBIOS_PIXHAWK_CUBE
    LINE_LED2,
    LINE_LED3
#endif
};

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_NUCLEO_F412
static const uint8_t num_leds = 3;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_PIXHAWK_CUBE
static const uint8_t num_leds = 1;
#endif
ChibiGPIO::ChibiGPIO()
{}

void ChibiGPIO::init()
{
    palClearLine(LINE_LED1);
#if CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_CHIBIOS_PIXHAWK_CUBE
    palClearLine(LINE_LED2);
    palClearLine(LINE_LED3);
#endif
}

void ChibiGPIO::pinMode(uint8_t pin, uint8_t output)
{
    if(pin >= num_leds) {
        return;
    }
    palSetLineMode(_gpio_tab[pin], output);
}

int8_t ChibiGPIO::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}


uint8_t ChibiGPIO::read(uint8_t pin) {
    if(pin >= num_leds) {
        return 0;
    }
    return palReadLine(_gpio_tab[pin]);
}

void ChibiGPIO::write(uint8_t pin, uint8_t value)
{
    if(pin >= num_leds) {
        return;
    }
    if (value == PAL_LOW) {
        palClearLine(_gpio_tab[pin]);
    } else {
        palSetLine(_gpio_tab[pin]);
    }
}

void ChibiGPIO::toggle(uint8_t pin)
{
    if(pin >= num_leds) {
        return;
    }
    palToggleLine(_gpio_tab[pin]);
}

/* Alternative interface: */
AP_HAL::DigitalSource* ChibiGPIO::channel(uint16_t n) {
    return new ChibiDigitalSource(0);
}

/* Interrupt interface: */
bool ChibiGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {
    return true;
}

bool ChibiGPIO::usb_connected(void)
{
    return _usb_connected;
}

ChibiDigitalSource::ChibiDigitalSource(uint8_t v) :
    _v(v)
{}

void ChibiDigitalSource::mode(uint8_t output)
{}

uint8_t ChibiDigitalSource::read() {
    return _v;
}

void ChibiDigitalSource::write(uint8_t value) {
    _v = value;
}

void ChibiDigitalSource::toggle() {
    _v = !_v;
}
#endif //HAL_BOARD_ChibiOS
