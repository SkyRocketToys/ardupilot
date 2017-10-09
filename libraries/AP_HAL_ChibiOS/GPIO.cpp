
#include "GPIO.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

using namespace ChibiOS;

static uint32_t _gpio_tab[]  = {
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_NUCLEO_F412
    LINE_LED1,
    LINE_LED2,
    LINE_LED3
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_PIXHAWK_CUBE ||\
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_V2450
    LINE_LED1,
    PAL_LINE(GPIOB, 0U)
#endif
};

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_NUCLEO_F412
static const uint8_t num_leds = 3;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_PIXHAWK_CUBE ||\
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_V2450
static const uint8_t num_leds = 1;
#endif

static void ext_interrupt_cb(EXTDriver *extp, expchannel_t channel);

static AP_HAL::Proc ext_irq[22]; // ext int irq list
static EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

static const uint32_t irq_port_list[] = {
    EXT_MODE_GPIOA, //Chan 0
    EXT_MODE_GPIOA, //Chan 1
    EXT_MODE_GPIOA, //Chan 2
    EXT_MODE_GPIOA, //Chan 3
    EXT_MODE_GPIOA, //Chan 4
    EXT_MODE_GPIOA, //Chan 5
    EXT_MODE_GPIOA, //Chan 6
    EXT_MODE_GPIOA, //Chan 7
    EXT_MODE_GPIOA, //Chan 8
    EXT_MODE_GPIOA, //Chan 9
    EXT_MODE_GPIOA, //Chan 10
    EXT_MODE_GPIOA, //Chan 11
    EXT_MODE_GPIOA, //Chan 12
    EXT_MODE_GPIOA, //Chan 13
    EXT_MODE_GPIOA, //Chan 14
    //Cypress IRQ PortD, 15
    EXT_MODE_GPIOD //Chan 15
};

ChibiGPIO::ChibiGPIO()
{}

void ChibiGPIO::init()
{
    palClearLine(LINE_LED1);
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_NUCLEO_F412
    palClearLine(LINE_LED2);
    palClearLine(LINE_LED3);
#endif
    extStart(&EXTD1, &extcfg);
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
bool ChibiGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode) {
    extStop(&EXTD1);
    switch(mode) {
        case HAL_GPIO_INTERRUPT_LOW:
            extcfg.channels[interrupt_num].mode = EXT_CH_MODE_LOW_LEVEL;
            break;
        case HAL_GPIO_INTERRUPT_FALLING:
            extcfg.channels[interrupt_num].mode = EXT_CH_MODE_FALLING_EDGE;
            break;
        case HAL_GPIO_INTERRUPT_RISING:
            extcfg.channels[interrupt_num].mode = EXT_CH_MODE_RISING_EDGE;
            break;
        case HAL_GPIO_INTERRUPT_BOTH:
            extcfg.channels[interrupt_num].mode = EXT_CH_MODE_BOTH_EDGES;
            break;
        default: return false;
    }
    extcfg.channels[interrupt_num].mode |= EXT_CH_MODE_AUTOSTART | irq_port_list[interrupt_num];
    ext_irq[interrupt_num] = p;
    extcfg.channels[interrupt_num].cb = ext_interrupt_cb;
    extStart(&EXTD1, &extcfg);
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

void ext_interrupt_cb(EXTDriver *extp, expchannel_t channel) {
    if (ext_irq[channel] != nullptr) {
        ext_irq[channel]();
    }
}
#endif //HAL_BOARD_ChibiOS
