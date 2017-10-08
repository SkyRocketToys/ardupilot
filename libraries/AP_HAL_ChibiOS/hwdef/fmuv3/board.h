/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#pragma once

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                24000000U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F427xx

/*
 * APM HW Defines
 */
#define PPM_ICU_TIMER  ICUD4
#define PPM_ICU_CHANNEL  ICU_CHANNEL_2

#define HRT_TIMER GPTD5
#define LINE_LED1 PAL_LINE(GPIOE,12)

#define HAL_STDOUT_SERIAL SD3
#define HAL_STDOUT_BAUDRATE 115200
/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
*/
/*
//NC GPIO_USART1_RX_1      (GPIO_ALT|GPIO_AF7|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN10)
 GPIO_UART4_RX_1       (GPIO_ALT|GPIO_AF8|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN1)
 GPIO_UART4_TX_1       (GPIO_ALT|GPIO_AF8|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN0)
 GPIO_SPI1_MISO_1      (GPIO_ALT|GPIO_AF5|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN6)
 GPIO_SPI1_MOSI_1      (GPIO_ALT|GPIO_AF5|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN7)
 GPIO_SPI1_SCK_1       (GPIO_ALT|GPIO_AF5|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN5)

 GPIO_USB_OTG_FS_DM     (GPIO_ALT|GPIO_AF10|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN11)
 GPIO_USB_OTG_FS_DP     (GPIO_ALT|GPIO_AF10|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN12)
 SWDIO
 SWDCLK
*/
#define VAL_GPIOA_MODER             (PIN_MODE_ALTERNATE(0U) |  \
                                     PIN_MODE_ALTERNATE(1U) |  \
                                     PIN_MODE_INPUT(2U) |      \
                                     PIN_MODE_INPUT(3U) |         \
                                     PIN_MODE_INPUT(4U) |        \
                                     PIN_MODE_ALTERNATE(5U) |    \
                                     PIN_MODE_ALTERNATE(6U) |    \
                                     PIN_MODE_ALTERNATE(7U) |    \
                                     PIN_MODE_INPUT(8U) |    \
                                     PIN_MODE_INPUT(9U) |      \
                                     PIN_MODE_ALTERNATE(10U) |     \
                                     PIN_MODE_ALTERNATE(11U) |     \
                                     PIN_MODE_ALTERNATE(12U) |     \
                                     PIN_MODE_ALTERNATE(13U) |      \
                                     PIN_MODE_ALTERNATE(14U) |      \
                                     PIN_MODE_INPUT(15U))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(0U) |    \
                                     PIN_OTYPE_PUSHPULL(1U) |       \
                                     PIN_OTYPE_PUSHPULL(2U) |     \
                                     PIN_OTYPE_PUSHPULL(3U) |     \
                                     PIN_OTYPE_PUSHPULL(4U) |    \
                                     PIN_OTYPE_PUSHPULL(5U) |    \
                                     PIN_OTYPE_PUSHPULL(6U) |    \
                                     PIN_OTYPE_PUSHPULL(7U) |    \
                                     PIN_OTYPE_PUSHPULL(8U) |    \
                                     PIN_OTYPE_PUSHPULL(9U) |   \
                                     PIN_OTYPE_PUSHPULL(10U) |     \
                                     PIN_OTYPE_PUSHPULL(11U) |     \
                                     PIN_OTYPE_PUSHPULL(12U) |     \
                                     PIN_OTYPE_PUSHPULL(13U) |      \
                                     PIN_OTYPE_PUSHPULL(14U) |      \
                                     PIN_OTYPE_PUSHPULL(15U))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(0U) |       \
                                     PIN_OSPEED_HIGH(1U) |       \
                                     PIN_OSPEED_HIGH(2U) |     \
                                     PIN_OSPEED_HIGH(3U) |        \
                                     PIN_OSPEED_HIGH(4U) |       \
                                     PIN_OSPEED_MEDIUM(5U) |       \
                                     PIN_OSPEED_MEDIUM(6U) |       \
                                     PIN_OSPEED_MEDIUM(7U) |       \
                                     PIN_OSPEED_HIGH(8U) |       \
                                     PIN_OSPEED_HIGH(9U) |      \
                                     PIN_OSPEED_HIGH(10U) |        \
                                     PIN_OSPEED_HIGH(11U) |        \
                                     PIN_OSPEED_HIGH(12U) |        \
                                     PIN_OSPEED_HIGH(13U) |         \
                                     PIN_OSPEED_HIGH(14U) |         \
                                     PIN_OSPEED_HIGH(15U))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(0U) |      \
                                     PIN_PUPDR_PULLUP(1U) |         \
                                     PIN_PUPDR_FLOATING(2U) |       \
                                     PIN_PUPDR_FLOATING(3U) |       \
                                     PIN_PUPDR_FLOATING(4U) |      \
                                     PIN_PUPDR_FLOATING(5U) |      \
                                     PIN_PUPDR_FLOATING(6U) |      \
                                     PIN_PUPDR_FLOATING(7U) |      \
                                     PIN_PUPDR_FLOATING(8U) |    \
                                     PIN_PUPDR_FLOATING(9U) |   \
                                     PIN_PUPDR_FLOATING(10U) |     \
                                     PIN_PUPDR_FLOATING(11U) |     \
                                     PIN_PUPDR_FLOATING(12U) |     \
                                     PIN_PUPDR_PULLUP(13U) |      \
                                     PIN_PUPDR_PULLDOWN(14U) |      \
                                     PIN_PUPDR_FLOATING(15U))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(0U) |          \
                                     PIN_ODR_HIGH(1U) |             \
                                     PIN_ODR_HIGH(2U) |           \
                                     PIN_ODR_HIGH(3U) |           \
                                     PIN_ODR_HIGH(4U) |          \
                                     PIN_ODR_HIGH(5U) |          \
                                     PIN_ODR_HIGH(6U) |          \
                                     PIN_ODR_HIGH(7U) |          \
                                     PIN_ODR_HIGH(8U) |          \
                                     PIN_ODR_HIGH(9U) |         \
                                     PIN_ODR_HIGH(10U) |           \
                                     PIN_ODR_HIGH(11U) |           \
                                     PIN_ODR_HIGH(12U) |           \
                                     PIN_ODR_HIGH(13U) |            \
                                     PIN_ODR_HIGH(14U) |            \
                                     PIN_ODR_HIGH(15U))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(0U, 8U) |        \
                                     PIN_AFIO_AF(1U, 8U) |           \
                                     PIN_AFIO_AF(2U, 0U) |         \
                                     PIN_AFIO_AF(3U, 0U) |         \
                                     PIN_AFIO_AF(4U, 0U) |        \
                                     PIN_AFIO_AF(5U, 5U) |        \
                                     PIN_AFIO_AF(6U, 5U) |        \
                                     PIN_AFIO_AF(7U, 5U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(8U, 0U) |       \
                                     PIN_AFIO_AF(9U, 0U) |       \
                                     PIN_AFIO_AF(10U, 10U) |        \
                                     PIN_AFIO_AF(11U, 10U) |        \
                                     PIN_AFIO_AF(12U, 10U) |        \
                                     PIN_AFIO_AF(13U, 0U) |          \
                                     PIN_AFIO_AF(14U, 0U) |          \
                                     PIN_AFIO_AF(15U, 0U))

/*
 * GPIOB setup:
 GPIO_I2C1_SCL_2       (GPIO_ALT|GPIO_AF4|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTB|GPIO_PIN8)
 GPIO_I2C1_SDA_2       (GPIO_ALT|GPIO_AF4|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTB|GPIO_PIN9)

 GPIO_I2C2_SCL_1       (GPIO_ALT|GPIO_AF4|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTB|GPIO_PIN10)
 GPIO_I2C2_SDA_1       (GPIO_ALT|GPIO_AF4|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTB|GPIO_PIN11)


 GPIO_SPI2_MISO_1      (GPIO_ALT|GPIO_AF5|GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN14)
 GPIO_SPI2_MOSI_1      (GPIO_ALT|GPIO_AF5|GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN15)
 GPIO_SPI2_SCK_2       (GPIO_ALT|GPIO_AF5|GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN13)
*/
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(0U) |        \
                                     PIN_MODE_INPUT(1U) |           \
                                     PIN_MODE_INPUT(2U) |         \
                                     PIN_MODE_INPUT(3U) |         \
                                     PIN_MODE_INPUT(4U) |        \
                                     PIN_MODE_INPUT(5U) |    \
                                     PIN_MODE_INPUT(6U) |    \
                                     PIN_MODE_INPUT(7U) |    \
                                     PIN_MODE_ALTERNATE(8U) |    \
                                     PIN_MODE_ALTERNATE(9U) |      \
                                     PIN_MODE_ALTERNATE(10U) |     \
                                     PIN_MODE_ALTERNATE(11U) |     \
                                     PIN_MODE_INPUT(12U) |     \
                                     PIN_MODE_ALTERNATE(13U) |      \
                                     PIN_MODE_ALTERNATE(14U) |      \
                                     PIN_MODE_ALTERNATE(15U))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(0U) |    \
                                     PIN_OTYPE_PUSHPULL(1U) |       \
                                     PIN_OTYPE_PUSHPULL(2U) |     \
                                     PIN_OTYPE_PUSHPULL(3U) |     \
                                     PIN_OTYPE_PUSHPULL(4U) |    \
                                     PIN_OTYPE_PUSHPULL(5U) |    \
                                     PIN_OTYPE_PUSHPULL(6U) |    \
                                     PIN_OTYPE_PUSHPULL(7U) |    \
                                     PIN_OTYPE_OPENDRAIN(8U) |    \
                                     PIN_OTYPE_OPENDRAIN(9U) |   \
                                     PIN_OTYPE_OPENDRAIN(10U) |     \
                                     PIN_OTYPE_OPENDRAIN(11U) |     \
                                     PIN_OTYPE_PUSHPULL(12U) |     \
                                     PIN_OTYPE_PUSHPULL(13U) |      \
                                     PIN_OTYPE_PUSHPULL(14U) |      \
                                     PIN_OTYPE_PUSHPULL(15U))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(0U) |       \
                                     PIN_OSPEED_HIGH(1U) |       \
                                     PIN_OSPEED_HIGH(2U) |     \
                                     PIN_OSPEED_HIGH(3U) |        \
                                     PIN_OSPEED_HIGH(4U) |       \
                                     PIN_OSPEED_HIGH(5U) |       \
                                     PIN_OSPEED_HIGH(6U) |       \
                                     PIN_OSPEED_HIGH(7U) |       \
                                     PIN_OSPEED_MEDIUM(8U) |       \
                                     PIN_OSPEED_MEDIUM(9U) |      \
                                     PIN_OSPEED_MEDIUM(10U) |        \
                                     PIN_OSPEED_MEDIUM(11U) |        \
                                     PIN_OSPEED_HIGH(12U) |        \
                                     PIN_OSPEED_MEDIUM(13U) |         \
                                     PIN_OSPEED_MEDIUM(14U) |         \
                                     PIN_OSPEED_MEDIUM(15U))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(0U) |      \
                                     PIN_PUPDR_FLOATING(1U) |         \
                                     PIN_PUPDR_FLOATING(2U) |       \
                                     PIN_PUPDR_FLOATING(3U) |       \
                                     PIN_PUPDR_FLOATING(4U) |      \
                                     PIN_PUPDR_FLOATING(5U) |      \
                                     PIN_PUPDR_FLOATING(6U) |      \
                                     PIN_PUPDR_FLOATING(7U) |      \
                                     PIN_PUPDR_FLOATING(8U) |    \
                                     PIN_PUPDR_FLOATING(9U) |   \
                                     PIN_PUPDR_FLOATING(10U) |     \
                                     PIN_PUPDR_FLOATING(11U) |     \
                                     PIN_PUPDR_FLOATING(12U) |     \
                                     PIN_PUPDR_FLOATING(13U) |      \
                                     PIN_PUPDR_FLOATING(14U) |      \
                                     PIN_PUPDR_FLOATING(15U))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(0U) |          \
                                     PIN_ODR_HIGH(1U) |             \
                                     PIN_ODR_HIGH(2U) |           \
                                     PIN_ODR_HIGH(3U) |           \
                                     PIN_ODR_HIGH(4U) |          \
                                     PIN_ODR_HIGH(5U) |          \
                                     PIN_ODR_HIGH(6U) |          \
                                     PIN_ODR_HIGH(7U) |          \
                                     PIN_ODR_HIGH(8U) |          \
                                     PIN_ODR_HIGH(9U) |         \
                                     PIN_ODR_HIGH(10U) |           \
                                     PIN_ODR_HIGH(11U) |           \
                                     PIN_ODR_HIGH(12U) |           \
                                     PIN_ODR_HIGH(13U) |            \
                                     PIN_ODR_HIGH(14U) |            \
                                     PIN_ODR_HIGH(15U))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(0U, 0U) |        \
                                     PIN_AFIO_AF(1U, 0U) |           \
                                     PIN_AFIO_AF(2U, 0U) |         \
                                     PIN_AFIO_AF(3U, 0U) |         \
                                     PIN_AFIO_AF(4U, 0U) |        \
                                     PIN_AFIO_AF(5U, 0U) |        \
                                     PIN_AFIO_AF(6U, 0U) |        \
                                     PIN_AFIO_AF(7U, 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(8U, 4U) |       \
                                     PIN_AFIO_AF(9U, 4U) |       \
                                     PIN_AFIO_AF(10U, 4U) |        \
                                     PIN_AFIO_AF(11U, 4U) |        \
                                     PIN_AFIO_AF(12U, 0U) |        \
                                     PIN_AFIO_AF(13U, 5U) |          \
                                     PIN_AFIO_AF(14U, 5U) |          \
                                     PIN_AFIO_AF(15U, 5U))



/*
 * GPIOC setup:
*/
/*

 GPIO_USART6_RX_1      (GPIO_ALT|GPIO_AF8|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN7)
 GPIO_USART6_TX_1      (GPIO_ALT|GPIO_AF8|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN6)
 PC8  - SD_D0                     (alternate 12).
 PC9  - SD_D1                     (alternate 12).
 PC10 - SD_D2                     (alternate 12)
 PC11 - SD_D3                     (alternate 12)
 PC12 - SD_CLK                    (alternate 12)
*/
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(0U) |        \
                                     PIN_MODE_INPUT(1U) |           \
                                     PIN_MODE_INPUT(2U) |         \
                                     PIN_MODE_INPUT(3U) |         \
                                     PIN_MODE_INPUT(4U) |        \
                                     PIN_MODE_INPUT(5U) |    \
                                     PIN_MODE_ALTERNATE(6U) |    \
                                     PIN_MODE_ALTERNATE(7U) |    \
                                     PIN_MODE_ALTERNATE(8U) |    \
                                     PIN_MODE_ALTERNATE(9U) |      \
                                     PIN_MODE_ALTERNATE(10U) |     \
                                     PIN_MODE_ALTERNATE(11U) |     \
                                     PIN_MODE_ALTERNATE(12U) |     \
                                     PIN_MODE_INPUT(13U) |      \
                                     PIN_MODE_INPUT(14U) |      \
                                     PIN_MODE_INPUT(15U))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(0U) |    \
                                     PIN_OTYPE_PUSHPULL(1U) |       \
                                     PIN_OTYPE_PUSHPULL(2U) |     \
                                     PIN_OTYPE_PUSHPULL(3U) |     \
                                     PIN_OTYPE_PUSHPULL(4U) |    \
                                     PIN_OTYPE_PUSHPULL(5U) |    \
                                     PIN_OTYPE_PUSHPULL(6U) |    \
                                     PIN_OTYPE_PUSHPULL(7U) |    \
                                     PIN_OTYPE_PUSHPULL(8U) |    \
                                     PIN_OTYPE_PUSHPULL(9U) |   \
                                     PIN_OTYPE_PUSHPULL(10U) |     \
                                     PIN_OTYPE_PUSHPULL(11U) |     \
                                     PIN_OTYPE_PUSHPULL(12U) |     \
                                     PIN_OTYPE_PUSHPULL(13U) |      \
                                     PIN_OTYPE_PUSHPULL(14U) |      \
                                     PIN_OTYPE_PUSHPULL(15U))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(0U) |       \
                                     PIN_OSPEED_HIGH(1U) |       \
                                     PIN_OSPEED_HIGH(2U) |     \
                                     PIN_OSPEED_HIGH(3U) |        \
                                     PIN_OSPEED_HIGH(4U) |       \
                                     PIN_OSPEED_HIGH(5U) |       \
                                     PIN_OSPEED_HIGH(6U) |       \
                                     PIN_OSPEED_HIGH(7U) |       \
                                     PIN_OSPEED_HIGH(8U) |       \
                                     PIN_OSPEED_HIGH(9U) |      \
                                     PIN_OSPEED_HIGH(10U) |        \
                                     PIN_OSPEED_HIGH(11U) |        \
                                     PIN_OSPEED_HIGH(12U) |        \
                                     PIN_OSPEED_HIGH(13U) |         \
                                     PIN_OSPEED_HIGH(14U) |         \
                                     PIN_OSPEED_HIGH(15U))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(0U) |      \
                                     PIN_PUPDR_FLOATING(1U) |         \
                                     PIN_PUPDR_FLOATING(2U) |       \
                                     PIN_PUPDR_FLOATING(3U) |       \
                                     PIN_PUPDR_FLOATING(4U) |      \
                                     PIN_PUPDR_FLOATING(5U) |      \
                                     PIN_PUPDR_PULLUP(6U) |      \
                                     PIN_PUPDR_PULLUP(7U) |      \
                                     PIN_PUPDR_FLOATING(8U) |    \
                                     PIN_PUPDR_FLOATING(9U) |   \
                                     PIN_PUPDR_FLOATING(10U) |     \
                                     PIN_PUPDR_FLOATING(11U) |     \
                                     PIN_PUPDR_FLOATING(12U) |     \
                                     PIN_PUPDR_FLOATING(13U) |      \
                                     PIN_PUPDR_FLOATING(14U) |      \
                                     PIN_PUPDR_FLOATING(15U))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(0U) |          \
                                     PIN_ODR_HIGH(1U) |             \
                                     PIN_ODR_HIGH(2U) |           \
                                     PIN_ODR_HIGH(3U) |           \
                                     PIN_ODR_HIGH(4U) |          \
                                     PIN_ODR_HIGH(5U) |          \
                                     PIN_ODR_HIGH(6U) |          \
                                     PIN_ODR_HIGH(7U) |          \
                                     PIN_ODR_HIGH(8U) |          \
                                     PIN_ODR_HIGH(9U) |         \
                                     PIN_ODR_HIGH(10U) |           \
                                     PIN_ODR_HIGH(11U) |           \
                                     PIN_ODR_HIGH(12U) |           \
                                     PIN_ODR_HIGH(13U) |            \
                                     PIN_ODR_HIGH(14U) |            \
                                     PIN_ODR_HIGH(15U))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(0U, 0U) |        \
                                     PIN_AFIO_AF(1U, 0U) |           \
                                     PIN_AFIO_AF(2U, 0U) |         \
                                     PIN_AFIO_AF(3U, 0U) |         \
                                     PIN_AFIO_AF(4U, 0U) |        \
                                     PIN_AFIO_AF(5U, 0U) |        \
                                     PIN_AFIO_AF(6U, 8U) |        \
                                     PIN_AFIO_AF(7U, 8U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(8U, 12U) |       \
                                     PIN_AFIO_AF(9U, 12U) |       \
                                     PIN_AFIO_AF(10U, 12U) |        \
                                     PIN_AFIO_AF(11U, 12U) |        \
                                     PIN_AFIO_AF(12U, 12U) |        \
                                     PIN_AFIO_AF(13U, 0U) |          \
                                     PIN_AFIO_AF(14U, 0U) |          \
                                     PIN_AFIO_AF(15U, 0U))

/*
 * GPIOA setup:

 GPIO_USART2_RX_2      (GPIO_ALT|GPIO_AF7|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN6)
 GPIO_USART2_TX_2      (GPIO_ALT|GPIO_AF7|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN5)
 GPIO_USART3_RX_3      (GPIO_ALT|GPIO_AF7|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN9)
 GPIO_USART3_TX_3      (GPIO_ALT|GPIO_AF7|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN8)

 PD2  Alternate 12 SD_CMD
 GPIO_TIM4_CH2IN_2     (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_FLOAT|GPIO_PORTD|GPIO_PIN13)

*/
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(0U) |        \
                                     PIN_MODE_INPUT(1U) |           \
                                     PIN_MODE_ALTERNATE(2U) |         \
                                     PIN_MODE_INPUT(3U) |         \
                                     PIN_MODE_INPUT(4U) |        \
                                     PIN_MODE_ALTERNATE(5U) |    \
                                     PIN_MODE_ALTERNATE(6U) |    \
                                     PIN_MODE_INPUT(7U) |    \
                                     PIN_MODE_ALTERNATE(8U) |    \
                                     PIN_MODE_ALTERNATE(9U) |      \
                                     PIN_MODE_INPUT(10U) |     \
                                     PIN_MODE_INPUT(11U) |     \
                                     PIN_MODE_INPUT(12U) |     \
                                     PIN_MODE_ALTERNATE(13U) |      \
                                     PIN_MODE_OUTPUT(14U) |      \
                                     PIN_MODE_INPUT(15U))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(0U) |    \
                                     PIN_OTYPE_PUSHPULL(1U) |       \
                                     PIN_OTYPE_PUSHPULL(2U) |     \
                                     PIN_OTYPE_PUSHPULL(3U) |     \
                                     PIN_OTYPE_PUSHPULL(4U) |    \
                                     PIN_OTYPE_PUSHPULL(5U) |    \
                                     PIN_OTYPE_PUSHPULL(6U) |    \
                                     PIN_OTYPE_PUSHPULL(7U) |    \
                                     PIN_OTYPE_PUSHPULL(8U) |    \
                                     PIN_OTYPE_PUSHPULL(9U) |   \
                                     PIN_OTYPE_PUSHPULL(10U) |     \
                                     PIN_OTYPE_PUSHPULL(11U) |     \
                                     PIN_OTYPE_PUSHPULL(12U) |     \
                                     PIN_OTYPE_PUSHPULL(13U) |      \
                                     PIN_OTYPE_PUSHPULL(14U) |      \
                                     PIN_OTYPE_PUSHPULL(15U))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(0U) |       \
                                     PIN_OSPEED_HIGH(1U) |       \
                                     PIN_OSPEED_HIGH(2U) |     \
                                     PIN_OSPEED_HIGH(3U) |        \
                                     PIN_OSPEED_HIGH(4U) |       \
                                     PIN_OSPEED_HIGH(5U) |       \
                                     PIN_OSPEED_HIGH(6U) |       \
                                     PIN_OSPEED_HIGH(7U) |       \
                                     PIN_OSPEED_HIGH(8U) |       \
                                     PIN_OSPEED_HIGH(9U) |      \
                                     PIN_OSPEED_HIGH(10U) |        \
                                     PIN_OSPEED_HIGH(11U) |        \
                                     PIN_OSPEED_HIGH(12U) |        \
                                     PIN_OSPEED_MEDIUM(13U) |         \
                                     PIN_OSPEED_HIGH(14U) |         \
                                     PIN_OSPEED_HIGH(15U))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(0U) |      \
                                     PIN_PUPDR_FLOATING(1U) |         \
                                     PIN_PUPDR_FLOATING(2U) |       \
                                     PIN_PUPDR_FLOATING(3U) |       \
                                     PIN_PUPDR_FLOATING(4U) |      \
                                     PIN_PUPDR_PULLUP(5U) |      \
                                     PIN_PUPDR_PULLUP(6U) |      \
                                     PIN_PUPDR_FLOATING(7U) |      \
                                     PIN_PUPDR_PULLUP(8U) |    \
                                     PIN_PUPDR_PULLUP(9U) |   \
                                     PIN_PUPDR_FLOATING(10U) |     \
                                     PIN_PUPDR_FLOATING(11U) |     \
                                     PIN_PUPDR_FLOATING(12U) |     \
                                     PIN_PUPDR_PULLDOWN(13U) |      \
                                     PIN_PUPDR_FLOATING(14U) |      \
                                     PIN_PUPDR_FLOATING(15U))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(0U) |          \
                                     PIN_ODR_HIGH(1U) |             \
                                     PIN_ODR_HIGH(2U) |           \
                                     PIN_ODR_HIGH(3U) |           \
                                     PIN_ODR_HIGH(4U) |          \
                                     PIN_ODR_HIGH(5U) |          \
                                     PIN_ODR_HIGH(6U) |          \
                                     PIN_ODR_HIGH(7U) |          \
                                     PIN_ODR_HIGH(8U) |          \
                                     PIN_ODR_HIGH(9U) |         \
                                     PIN_ODR_HIGH(10U) |           \
                                     PIN_ODR_HIGH(11U) |           \
                                     PIN_ODR_HIGH(12U) |           \
                                     PIN_ODR_HIGH(13U) |            \
                                     PIN_ODR_LOW(14U) |            \
                                     PIN_ODR_HIGH(15U))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(0U, 0U) |        \
                                     PIN_AFIO_AF(1U, 0U) |           \
                                     PIN_AFIO_AF(2U, 12U) |         \
                                     PIN_AFIO_AF(3U, 0U) |         \
                                     PIN_AFIO_AF(4U, 0U) |        \
                                     PIN_AFIO_AF(5U, 7U) |        \
                                     PIN_AFIO_AF(6U, 7U) |        \
                                     PIN_AFIO_AF(7U, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(8U, 7U) |       \
                                     PIN_AFIO_AF(9U, 7U) |       \
                                     PIN_AFIO_AF(10U, 0U) |        \
                                     PIN_AFIO_AF(11U, 0U) |        \
                                     PIN_AFIO_AF(12U, 0U) |        \
                                     PIN_AFIO_AF(13U, 2U) |          \
                                     PIN_AFIO_AF(14U, 0U) |          \
                                     PIN_AFIO_AF(15U, 0U))

/*
 * GPIOE setup:
 GPIO_UART7_RX_1     (GPIO_ALT|GPIO_AF8|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN7)
 GPIO_UART7_TX_1     (GPIO_ALT|GPIO_AF8|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN8)

 GPIO_SPI4_MISO_1    (GPIO_ALT|GPIO_AF5|GPIO_SPEED_50MHz|GPIO_PORTE|GPIO_PIN5)
 GPIO_SPI4_MOSI_1    (GPIO_ALT|GPIO_AF5|GPIO_SPEED_50MHz|GPIO_PORTE|GPIO_PIN6)
 GPIO_SPI4_SCK_1     (GPIO_ALT|GPIO_AF5|GPIO_SPEED_50MHz|GPIO_PORTE|GPIO_PIN2)

 GPIO_TIM1_CH1OUT      (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN9)
 GPIO_TIM1_CH2OUT      (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN11)
 GPIO_TIM1_CH3OUT      (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN13)
 GPIO_TIM1_CH4OUT      (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN14)
*/
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(0U) |        \
                                     PIN_MODE_INPUT(1U) |           \
                                     PIN_MODE_ALTERNATE(2U) |         \
                                     PIN_MODE_INPUT(3U) |         \
                                     PIN_MODE_INPUT(4U) |        \
                                     PIN_MODE_ALTERNATE(5U) |    \
                                     PIN_MODE_ALTERNATE(6U) |    \
                                     PIN_MODE_ALTERNATE(7U) |    \
                                     PIN_MODE_ALTERNATE(8U) |    \
                                     PIN_MODE_ALTERNATE(9U) |      \
                                     PIN_MODE_INPUT(10U) |     \
                                     PIN_MODE_ALTERNATE(11U) |     \
                                     PIN_MODE_INPUT(12U) |     \
                                     PIN_MODE_ALTERNATE(13U) |      \
                                     PIN_MODE_ALTERNATE(14U) |      \
                                     PIN_MODE_INPUT(15U))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(0U) |    \
                                     PIN_OTYPE_PUSHPULL(1U) |       \
                                     PIN_OTYPE_PUSHPULL(2U) |     \
                                     PIN_OTYPE_PUSHPULL(3U) |     \
                                     PIN_OTYPE_PUSHPULL(4U) |    \
                                     PIN_OTYPE_PUSHPULL(5U) |    \
                                     PIN_OTYPE_PUSHPULL(6U) |    \
                                     PIN_OTYPE_PUSHPULL(7U) |    \
                                     PIN_OTYPE_PUSHPULL(8U) |    \
                                     PIN_OTYPE_PUSHPULL(9U) |   \
                                     PIN_OTYPE_PUSHPULL(10U) |     \
                                     PIN_OTYPE_PUSHPULL(11U) |     \
                                     PIN_OTYPE_PUSHPULL(12U) |     \
                                     PIN_OTYPE_PUSHPULL(13U) |      \
                                     PIN_OTYPE_PUSHPULL(14U) |      \
                                     PIN_OTYPE_PUSHPULL(15U))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(0U) |       \
                                     PIN_OSPEED_HIGH(1U) |       \
                                     PIN_OSPEED_MEDIUM(2U) |     \
                                     PIN_OSPEED_HIGH(3U) |        \
                                     PIN_OSPEED_HIGH(4U) |       \
                                     PIN_OSPEED_MEDIUM(5U) |       \
                                     PIN_OSPEED_MEDIUM(6U) |       \
                                     PIN_OSPEED_HIGH(7U) |       \
                                     PIN_OSPEED_HIGH(8U) |       \
                                     PIN_OSPEED_HIGH(9U) |      \
                                     PIN_OSPEED_HIGH(10U) |        \
                                     PIN_OSPEED_MEDIUM(11U) |        \
                                     PIN_OSPEED_HIGH(12U) |        \
                                     PIN_OSPEED_MEDIUM(13U) |         \
                                     PIN_OSPEED_MEDIUM(14U) |         \
                                     PIN_OSPEED_HIGH(15U))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(0U) |      \
                                     PIN_PUPDR_FLOATING(1U) |         \
                                     PIN_PUPDR_FLOATING(2U) |       \
                                     PIN_PUPDR_FLOATING(3U) |       \
                                     PIN_PUPDR_FLOATING(4U) |      \
                                     PIN_PUPDR_FLOATING(5U) |      \
                                     PIN_PUPDR_FLOATING(6U) |      \
                                     PIN_PUPDR_PULLUP(7U) |      \
                                     PIN_PUPDR_PULLUP(8U) |    \
                                     PIN_PUPDR_FLOATING(9U) |   \
                                     PIN_PUPDR_FLOATING(10U) |     \
                                     PIN_PUPDR_FLOATING(11U) |     \
                                     PIN_PUPDR_FLOATING(12U) |     \
                                     PIN_PUPDR_FLOATING(13U) |      \
                                     PIN_PUPDR_FLOATING(14U) |      \
                                     PIN_PUPDR_FLOATING(15U))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(0U) |          \
                                     PIN_ODR_HIGH(1U) |             \
                                     PIN_ODR_HIGH(2U) |           \
                                     PIN_ODR_HIGH(3U) |           \
                                     PIN_ODR_HIGH(4U) |          \
                                     PIN_ODR_HIGH(5U) |          \
                                     PIN_ODR_HIGH(6U) |          \
                                     PIN_ODR_HIGH(7U) |          \
                                     PIN_ODR_HIGH(8U) |          \
                                     PIN_ODR_LOW(9U) |         \
                                     PIN_ODR_HIGH(10U) |           \
                                     PIN_ODR_LOW(11U) |           \
                                     PIN_ODR_HIGH(12U) |           \
                                     PIN_ODR_LOW(13U) |            \
                                     PIN_ODR_LOW(14U) |            \
                                     PIN_ODR_HIGH(15U))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(0U, 0U) |        \
                                     PIN_AFIO_AF(1U, 0U) |           \
                                     PIN_AFIO_AF(2U, 5U) |         \
                                     PIN_AFIO_AF(3U, 0U) |         \
                                     PIN_AFIO_AF(4U, 0U) |        \
                                     PIN_AFIO_AF(5U, 5U) |        \
                                     PIN_AFIO_AF(6U, 5U) |        \
                                     PIN_AFIO_AF(7U, 8U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(8U, 8U) |       \
                                     PIN_AFIO_AF(9U, 1U) |       \
                                     PIN_AFIO_AF(10U, 0U) |        \
                                     PIN_AFIO_AF(11U, 1U) |        \
                                     PIN_AFIO_AF(12U, 0U) |        \
                                     PIN_AFIO_AF(13U, 1U) |          \
                                     PIN_AFIO_AF(14U, 1U) |          \
                                     PIN_AFIO_AF(15U, 0U))

/*
 * Port F setup.
 * All input with pull-up.
 */
#define VAL_GPIOF_MODER             0x00000000
#define VAL_GPIOF_OTYPER            0x00000000
#define VAL_GPIOF_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOF_PUPDR             0xFFFFFFFF
#define VAL_GPIOF_ODR               0xFFFFFFFF
#define VAL_GPIOF_AFRL              0x00000000
#define VAL_GPIOF_AFRH              0x00000000

/*
 * Port G setup.
 * All input with pull-up.
 */
#define VAL_GPIOG_MODER             0x00000000
#define VAL_GPIOG_OTYPER            0x00000000
#define VAL_GPIOG_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOG_PUPDR             0xFFFFFFFF
#define VAL_GPIOG_ODR               0xFFFFFFFF
#define VAL_GPIOG_AFRL              0x00000000
#define VAL_GPIOG_AFRH              0x00000000


/*
 * Port H setup.
 * All input with pull-up except:
 * PH0  - GPIOH_OSC_IN          (input floating).
 * PH1  - GPIOH_OSC_OUT         (input floating).
 */
#define VAL_GPIOH_MODER     (PIN_MODE_INPUT(0) |                 \
                             PIN_MODE_INPUT(1) |                \
                             PIN_MODE_INPUT(2) |                            \
                             PIN_MODE_INPUT(3) |                            \
                             PIN_MODE_INPUT(4) |                            \
                             PIN_MODE_INPUT(5) |                            \
                             PIN_MODE_INPUT(6) |                            \
                             PIN_MODE_INPUT(7) |                            \
                             PIN_MODE_INPUT(8) |                            \
                             PIN_MODE_INPUT(9) |                            \
                             PIN_MODE_INPUT(10) |                           \
                             PIN_MODE_INPUT(11) |                           \
                             PIN_MODE_INPUT(12) |                           \
                             PIN_MODE_INPUT(13) |                           \
                             PIN_MODE_INPUT(14) |                           \
                             PIN_MODE_INPUT(15))
#define VAL_GPIOH_OTYPER    0x00000000
#define VAL_GPIOH_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOH_PUPDR     (PIN_PUPDR_FLOATING(0) |              \
                             PIN_PUPDR_FLOATING(1) |             \
                             PIN_PUPDR_PULLUP(2) |                           \
                             PIN_PUPDR_PULLUP(3) |                           \
                             PIN_PUPDR_PULLUP(4) |                           \
                             PIN_PUPDR_PULLUP(5) |                           \
                             PIN_PUPDR_PULLUP(6) |                           \
                             PIN_PUPDR_PULLUP(7) |                           \
                             PIN_PUPDR_PULLUP(8) |                           \
                             PIN_PUPDR_PULLUP(9) |                           \
                             PIN_PUPDR_PULLUP(10) |                          \
                             PIN_PUPDR_PULLUP(11) |                          \
                             PIN_PUPDR_PULLUP(12) |                          \
                             PIN_PUPDR_PULLUP(13) |                          \
                             PIN_PUPDR_PULLUP(14) |                          \
                             PIN_PUPDR_PULLUP(15))
#define VAL_GPIOH_ODR       0xFFFFFFFF
#define VAL_GPIOH_AFRL      0x00000000
#define VAL_GPIOH_AFRH      0x00000000
/*
 * Port I setup.
 * All input with pull-up.
 */
#define VAL_GPIOI_MODER             0x00000000
#define VAL_GPIOI_OTYPER            0x00000000
#define VAL_GPIOI_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOI_PUPDR             0xFFFFFFFF
#define VAL_GPIOI_ODR               0xFFFFFFFF
#define VAL_GPIOI_AFRL              0x00000000
#define VAL_GPIOI_AFRH              0x00000000
#ifndef HAL_BOARD_INIT_HOOK_DEFINE
#define HAL_BOARD_INIT_HOOK_DEFINE
#endif
#ifndef HAL_BOARD_INIT_HOOK_CALL
#define HAL_BOARD_INIT_HOOK_CALL
#endif
#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
  HAL_BOARD_INIT_HOOK_DEFINE
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

