#pragma once

#include "AP_HAL_ChibiOS.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
 # define HAL_GPIO_A_LED_PIN        0
 # define HAL_GPIO_B_LED_PIN        1
 # define HAL_GPIO_C_LED_PIN        2
 # define HAL_GPIO_LED_ON           LOW
 # define HAL_GPIO_LED_OFF          HIGH
#endif

class ChibiOS::ChibiGPIO : public AP_HAL::GPIO {
public:
    ChibiGPIO();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    int8_t  analogPinToDigitalPin(uint8_t pin);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);
};

class ChibiOS::ChibiDigitalSource : public AP_HAL::DigitalSource {
public:
    ChibiDigitalSource(uint8_t v);
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value);
    void    toggle();
private:
    uint8_t _v;
};
