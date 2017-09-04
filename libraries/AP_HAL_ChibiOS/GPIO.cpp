
#include "GPIO.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

using namespace ChibiOS;
static uint32_t _gpio_tab[]  = {
    LINE_LED1,
    LINE_LED2,
    LINE_LED3
};

ChibiGPIO::ChibiGPIO()
{}

void ChibiGPIO::init()
{
    palClearLine(LINE_LED1);
    palClearLine(LINE_LED2);
    palClearLine(LINE_LED3);
}

void ChibiGPIO::pinMode(uint8_t pin, uint8_t output)
{
    palSetLineMode(_gpio_tab[pin], output);
}

int8_t ChibiGPIO::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}


uint8_t ChibiGPIO::read(uint8_t pin) {
    return palReadLine(_gpio_tab[pin]);
}

void ChibiGPIO::write(uint8_t pin, uint8_t value)
{
    if (value == PAL_LOW) {
        palClearLine(_gpio_tab[pin]);
    } else {
        palSetLine(_gpio_tab[pin]);
    }
}

void ChibiGPIO::toggle(uint8_t pin)
{
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
