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

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for STMicroelectronics STM32 F412 SkyViper board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_SKYVIPER_F412
#define BOARD_NAME                  "STMicroelectronics STM32 SkyViperF412"
/*
 * APM HW Defines
 */

#define HRT_TIMER GPTD5

#define HAL_STDOUT_SERIAL SD2
#define HAL_STDOUT_BAUDRATE 115200

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

#define STM32_HSE_BYPASS

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F412Rx

/*
 * APM HW Defines
 */
#define PPM_ICU_TIMER  ICUD1
#define PPM_ICU_CHANNEL  ICU_CHANNEL_1

#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U
#define LINE_OSC_IN                 PAL_LINE(GPIOH, 0U)
#define LINE_OSC_OUT                PAL_LINE(GPIOH, 1U)




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
  PA2 -- TXD
  PA3 -- RXD
  PA5 -- RADIO SCK
  PA7 -- RADIO MOSI
  PA11 -- GPS TX
  PA13 -- SWDIO
  PA14 -- SWCLK
  PA9 -- OF NRESET
  PA8 -- OF MOTION
  PA10 -- OF MOSI
  PA12 -- OF MISO
  PA15 -- RADIO nCS
 */

#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(0U) |  \
                                     PIN_MODE_INPUT(1U) |  \
                                     PIN_MODE_ALTERNATE(2U) |      \
                                     PIN_MODE_ALTERNATE(3U) |         \
                                     PIN_MODE_INPUT(4U) |        \
                                     PIN_MODE_ALTERNATE(5U) |    \
                                     PIN_MODE_INPUT(6U) |    \
                                     PIN_MODE_ALTERNATE(7U) |    \
                                     PIN_MODE_INPUT(8U) |    \
                                     PIN_MODE_OUTPUT(9U) |      \
                                     PIN_MODE_ALTERNATE(10U) |     \
                                     PIN_MODE_ALTERNATE(11U) |     \
                                     PIN_MODE_ALTERNATE(12U) |     \
                                     PIN_MODE_ALTERNATE(13U) |      \
                                     PIN_MODE_ALTERNATE(14U) |      \
                                     PIN_MODE_OUTPUT(15U))
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
                                     PIN_OSPEED_HIGH(6U) |       \
                                     PIN_OSPEED_MEDIUM(7U) |       \
                                     PIN_OSPEED_HIGH(8U) |       \
                                     PIN_OSPEED_HIGH(9U) |      \
                                     PIN_OSPEED_HIGH(10U) |        \
                                     PIN_OSPEED_HIGH(11U) |        \
                                     PIN_OSPEED_HIGH(12U) |        \
                                     PIN_OSPEED_HIGH(13U) |         \
                                     PIN_OSPEED_HIGH(14U) |         \
                                     PIN_OSPEED_HIGH(15U))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(0U) |      \
                                     PIN_PUPDR_FLOATING(1U) |         \
                                     PIN_PUPDR_PULLUP(2U) |       \
                                     PIN_PUPDR_PULLUP(3U) |       \
                                     PIN_PUPDR_FLOATING(4U) |      \
                                     PIN_PUPDR_FLOATING(5U) |      \
                                     PIN_PUPDR_FLOATING(6U) |      \
                                     PIN_PUPDR_FLOATING(7U) |      \
                                     PIN_PUPDR_FLOATING(8U) |    \
                                     PIN_PUPDR_PULLUP(9U) |   \
                                     PIN_PUPDR_FLOATING(10U) |     \
                                     PIN_PUPDR_PULLUP(11U) |     \
                                     PIN_PUPDR_FLOATING(12U) |     \
                                     PIN_PUPDR_FLOATING(13U) |      \
                                     PIN_PUPDR_FLOATING(14U) |      \
                                     PIN_PUPDR_PULLUP(15U))
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
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(0U, 0U) |        \
                                     PIN_AFIO_AF(1U, 0U) |           \
                                     PIN_AFIO_AF(2U, 7U) |         \
                                     PIN_AFIO_AF(3U, 7U) |         \
                                     PIN_AFIO_AF(4U, 0U) |        \
                                     PIN_AFIO_AF(5U, 5U) |        \
                                     PIN_AFIO_AF(6U, 0U) |        \
                                     PIN_AFIO_AF(7U, 5U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(8U, 0U) |       \
                                     PIN_AFIO_AF(9U, 0U) |       \
                                     PIN_AFIO_AF(10U, 6U) |        \
                                     PIN_AFIO_AF(11U, 8U) |        \
                                     PIN_AFIO_AF(12U, 6U) |        \
                                     PIN_AFIO_AF(13U, 0U) |          \
                                     PIN_AFIO_AF(14U, 0U) |          \
                                     PIN_AFIO_AF(15U, 0U))

/*
 * GPIOB setup:
   PB9 -- MPU SDA
   PB8 -- MPU SCL
   PB5 -- PWM2
   PB4 -- RADIO MISO
   PB12 -- MPU nCS
   PB13 -- MPU CLK
   PB14 -- MPU MISO
   PB15 -- MPU MOSI
   PB3  -- I2C2 SDA
   PB10 -- I2C2 SCL
   PB0 -- OF SCK
   PB1 -- OF nCS
 */

#define VAL_GPIOB_MODER             (PIN_MODE_ALTERNATE(0U) |  \
                                     PIN_MODE_OUTPUT(1U) |  \
                                     PIN_MODE_INPUT(2U) |      \
                                     PIN_MODE_ALTERNATE(3U) |         \
                                     PIN_MODE_ALTERNATE(4U) |        \
                                     PIN_MODE_ALTERNATE(5U) |    \
                                     PIN_MODE_INPUT(6U) |    \
                                     PIN_MODE_INPUT(7U) |    \
                                     PIN_MODE_ALTERNATE(8U) |    \
                                     PIN_MODE_ALTERNATE(9U) |      \
                                     PIN_MODE_ALTERNATE(10U) |     \
                                     PIN_MODE_INPUT(11U) |     \
                                     PIN_MODE_OUTPUT(12U) |     \
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
                                     PIN_OTYPE_PUSHPULL(10U) |     \
                                     PIN_OTYPE_PUSHPULL(11U) |     \
                                     PIN_OTYPE_PUSHPULL(12U) |     \
                                     PIN_OTYPE_PUSHPULL(13U) |      \
                                     PIN_OTYPE_PUSHPULL(14U) |      \
                                     PIN_OTYPE_PUSHPULL(15U))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(0U) |       \
                                     PIN_OSPEED_HIGH(1U) |       \
                                     PIN_OSPEED_HIGH(2U) |     \
                                     PIN_OSPEED_HIGH(3U) |        \
                                     PIN_OSPEED_MEDIUM(4U) |       \
                                     PIN_OSPEED_MEDIUM(5U) |       \
                                     PIN_OSPEED_HIGH(6U) |       \
                                     PIN_OSPEED_HIGH(7U) |       \
                                     PIN_OSPEED_MEDIUM(8U) |       \
                                     PIN_OSPEED_MEDIUM(9U) |      \
                                     PIN_OSPEED_HIGH(10U) |        \
                                     PIN_OSPEED_HIGH(11U) |        \
                                     PIN_OSPEED_HIGH(12U) |        \
                                     PIN_OSPEED_MEDIUM(13U) |         \
                                     PIN_OSPEED_MEDIUM(14U) |         \
                                     PIN_OSPEED_MEDIUM(15U))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(0U) |      \
                                     PIN_PUPDR_PULLUP(1U) |         \
                                     PIN_PUPDR_FLOATING(2U) |       \
                                     PIN_PUPDR_FLOATING(3U) |       \
                                     PIN_PUPDR_FLOATING(4U) |      \
                                     PIN_PUPDR_PULLDOWN(5U) |      \
                                     PIN_PUPDR_FLOATING(6U) |      \
                                     PIN_PUPDR_FLOATING(7U) |      \
                                     PIN_PUPDR_FLOATING(8U) |    \
                                     PIN_PUPDR_FLOATING(9U) |   \
                                     PIN_PUPDR_FLOATING(10U) |     \
                                     PIN_PUPDR_FLOATING(11U) |     \
                                     PIN_PUPDR_PULLUP(12U) |     \
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
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(0U, 6U) |        \
                                     PIN_AFIO_AF(1U, 0U) |           \
                                     PIN_AFIO_AF(2U, 0U) |         \
                                     PIN_AFIO_AF(3U, 9U) |         \
                                     PIN_AFIO_AF(4U, 5U) |        \
                                     PIN_AFIO_AF(5U, 2U) |        \
                                     PIN_AFIO_AF(6U, 0U) |        \
                                     PIN_AFIO_AF(7U, 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(8U, 4U) |       \
                                     PIN_AFIO_AF(9U, 4U) |       \
                                     PIN_AFIO_AF(10U, 4U) |        \
                                     PIN_AFIO_AF(11U, 0U) |        \
                                     PIN_AFIO_AF(12U, 0U) |        \
                                     PIN_AFIO_AF(13U, 5U) |          \
                                     PIN_AFIO_AF(14U, 5U) |          \
                                     PIN_AFIO_AF(15U, 5U))

/*
 * GPIOC setup:
 *  
    PC6  -- PWM1
    PC7  -- GPS RX
    PC8  -- PWM4
    PC9  -- PWM3
    PC10 -- EXT TX
    PC11 -- EXT RX
    PC4 -- RADIO CE
 */

#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(0U) |  \
                                     PIN_MODE_INPUT(1U) |  \
                                     PIN_MODE_INPUT(2U) |      \
                                     PIN_MODE_INPUT(3U) |         \
                                     PIN_MODE_OUTPUT(4U) |        \
                                     PIN_MODE_INPUT(5U) |    \
                                     PIN_MODE_ALTERNATE(6U) |    \
                                     PIN_MODE_ALTERNATE(7U) |    \
                                     PIN_MODE_ALTERNATE(8U) |    \
                                     PIN_MODE_ALTERNATE(9U) |      \
                                     PIN_MODE_ALTERNATE(10U) |     \
                                     PIN_MODE_ALTERNATE(11U) |     \
                                     PIN_MODE_INPUT(12U) |     \
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
                                     PIN_OSPEED_MEDIUM(6U) |       \
                                     PIN_OSPEED_HIGH(7U) |       \
                                     PIN_OSPEED_MEDIUM(8U) |       \
                                     PIN_OSPEED_MEDIUM(9U) |      \
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
                                     PIN_PUPDR_PULLUP(4U) |      \
                                     PIN_PUPDR_FLOATING(5U) |      \
                                     PIN_PUPDR_PULLDOWN(6U) |      \
                                     PIN_PUPDR_PULLUP(7U) |      \
                                     PIN_PUPDR_PULLDOWN(8U) |    \
                                     PIN_PUPDR_PULLDOWN(9U) |   \
                                     PIN_PUPDR_PULLUP(10U) |     \
                                     PIN_PUPDR_PULLUP(11U) |     \
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
                                     PIN_AFIO_AF(6U, 2U) |        \
                                     PIN_AFIO_AF(7U, 8U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(8U, 2U) |       \
                                     PIN_AFIO_AF(9U, 2U) |       \
                                     PIN_AFIO_AF(10U, 7U) |        \
                                     PIN_AFIO_AF(11U, 7U) |        \
                                     PIN_AFIO_AF(12U, 0U) |        \
                                     PIN_AFIO_AF(13U, 0U) |          \
                                     PIN_AFIO_AF(14U, 0U) |          \
                                     PIN_AFIO_AF(15U, 0U))

/*
 * GPIOD setup:
 *
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(0U) |  \
                                     PIN_MODE_INPUT(1U) |  \
                                     PIN_MODE_INPUT(2U) |      \
                                     PIN_MODE_INPUT(3U) |         \
                                     PIN_MODE_INPUT(4U) |        \
                                     PIN_MODE_INPUT(5U) |    \
                                     PIN_MODE_INPUT(6U) |    \
                                     PIN_MODE_INPUT(7U) |    \
                                     PIN_MODE_INPUT(8U) |    \
                                     PIN_MODE_INPUT(9U) |      \
                                     PIN_MODE_INPUT(10U) |     \
                                     PIN_MODE_INPUT(11U) |     \
                                     PIN_MODE_INPUT(12U) |     \
                                     PIN_MODE_INPUT(13U) |      \
                                     PIN_MODE_INPUT(14U) |      \
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
                                     PIN_OSPEED_HIGH(13U) |         \
                                     PIN_OSPEED_HIGH(14U) |         \
                                     PIN_OSPEED_HIGH(15U))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(0U) |      \
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
                                     PIN_ODR_HIGH(14U) |            \
                                     PIN_ODR_HIGH(15U))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(0U, 0U) |        \
                                     PIN_AFIO_AF(1U, 0U) |           \
                                     PIN_AFIO_AF(2U, 0U) |         \
                                     PIN_AFIO_AF(3U, 0U) |         \
                                     PIN_AFIO_AF(4U, 0U) |        \
                                     PIN_AFIO_AF(5U, 0U) |        \
                                     PIN_AFIO_AF(6U, 0U) |        \
                                     PIN_AFIO_AF(7U, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(8U, 0U) |       \
                                     PIN_AFIO_AF(9U, 0U) |       \
                                     PIN_AFIO_AF(10U, 0U) |        \
                                     PIN_AFIO_AF(11U, 0U) |        \
                                     PIN_AFIO_AF(12U, 0U) |        \
                                     PIN_AFIO_AF(13U, 0U) |          \
                                     PIN_AFIO_AF(14U, 0U) |          \
                                     PIN_AFIO_AF(15U, 0U))

/*
 * GPIOE setup:
 *
 */
#define VAL_GPIOE_MODER             0x0
#define VAL_GPIOE_OTYPER            0x0
#define VAL_GPIOE_OSPEEDR           0x0
#define VAL_GPIOE_PUPDR             0x0
#define VAL_GPIOE_ODR               0x0
#define VAL_GPIOE_AFRL              0x0
#define VAL_GPIOE_AFRH              0x0

/*
 * GPIOF setup:
 */
#define VAL_GPIOF_MODER             0x0
#define VAL_GPIOF_OTYPER            0x0
#define VAL_GPIOF_OSPEEDR           0x0
#define VAL_GPIOF_PUPDR             0x0
#define VAL_GPIOF_ODR               0x0
#define VAL_GPIOF_AFRL              0x0
#define VAL_GPIOF_AFRH              0x0

/*
 * GPIOG setup:
 *
 */

#define VAL_GPIOG_MODER             0x0
#define VAL_GPIOG_OTYPER            0x0
#define VAL_GPIOG_OSPEEDR           0x0
#define VAL_GPIOG_PUPDR             0x0
#define VAL_GPIOG_ODR               0x0
#define VAL_GPIOG_AFRL              0x0
#define VAL_GPIOG_AFRH              0x0
/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOH_OSC_OUT))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0) |         \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0))
#define VAL_GPIOH_AFRH              0x0

/*
 * GPIOI setup:
 *
 */
#define VAL_GPIOI_MODER             0x0
#define VAL_GPIOI_OTYPER            0x0
#define VAL_GPIOI_OSPEEDR           0x0
#define VAL_GPIOI_PUPDR             0x0
#define VAL_GPIOI_ODR               0x0
#define VAL_GPIOI_AFRL              0x0
#define VAL_GPIOI_AFRH              0x0
/*
 * GPIOJ setup:
 *
 */
#define VAL_GPIOJ_MODER             0x0
#define VAL_GPIOJ_OTYPER            0x0
#define VAL_GPIOJ_OSPEEDR           0x0
#define VAL_GPIOJ_PUPDR             0x0
#define VAL_GPIOJ_ODR               0x0
#define VAL_GPIOJ_AFRL              0x0
#define VAL_GPIOJ_AFRH              0x0
/*
 * GPIOK setup:
 *
 */
#define VAL_GPIOK_MODER             0x0
#define VAL_GPIOK_OTYPER            0x0
#define VAL_GPIOK_OSPEEDR           0x0
#define VAL_GPIOK_PUPDR             0x0
#define VAL_GPIOK_ODR               0x0
#define VAL_GPIOK_AFRL              0x0
#define VAL_GPIOK_AFRH              0x0

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
