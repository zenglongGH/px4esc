/****************************************************************************
*
*   Copyright (C) 2016  Zubax Robotics  <info@zubax.com>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/*
 * Based on ChibiOS demo application for STM32F051.
 */

#pragma once

#define STM32F446xx

/*
 * Pin definitions
 */
/*
 * Port A
 */
#define GPIOA_SENS_A                    0
#define GPIOA_SENS_B                    1
#define GPIOA_SENS_C                    2

#define GPIOA_RCPWM                     3

#define GPIOA_OC_ADJ                    4
#define GPIOA_EN_GATE                   5
#define GPIOA_DC_CAL                    6

#define GPIOA_MOTOR_PWM_1N              7
#define GPIOA_MOTOR_PWM_1P              8
#define GPIOA_MOTOR_PWM_2P              9
#define GPIOA_MOTOR_PWM_3P              10

#define GPIOA_USB_DM                    11
#define GPIOA_USB_DP                    12

#define GPIOA_SWDIO                     13
#define GPIOA_SWCLK                     14

#define GPIOA_USB_EN                    15

/*
 * Port B
 */
#define GPIOB_MOTOR_PWM_2N              0
#define GPIOB_MOTOR_PWM_3N              1

#define GPIOB_GAIN                      2

#define GPIOB_TEST_2                    3
#define GPIOB_TEST_3                    4

#define GPIOB_CAN2_RX                   5
#define GPIOB_CAN2_TX                   6

#define GPIOB_REQUIRE_GET_NODE_INFO     7

#define GPIOB_CAN1_RX                   8
#define GPIOB_CAN1_TX                   9

#define GPIOB_UART_TX                   10

/*
 * Port C
 */
#define GPIOC_TEMPERATURE_SENS          0
#define GPIOC_VBAT_SENS                 1
#define GPIOC_CURRENT_2_SENS            2
#define GPIOC_CURRENT_1_SENS            3

#define GPIOC_UART_RX                   5

#define GPIOC_RPM_PULSE_FEEDBACK        6

#define GPIOC_RGB_LED_RED               7
#define GPIOC_RGB_LED_GREEN             8
#define GPIOC_RGB_LED_BLUE              9

#define GPIOC_TEST_4                    12

#define GPIOC_POWER_GOOD                13
#define GPIOC_OVER_TEMP_WARNING_INVERSE 14
#define GPIOC_DRIVER_FAULT_INVERSE      15

/*
 * Port D
 */
#define PORTD_TEST_1                    2

/*
 * Port E
 */
// Not present

/*
 * Port F
 */
// Not present

/*
 * Port G
 */
// Not present

/*
 * Port H
 */
#define GPIOH_OSC_IN                    0
#define GPIOH_OSC_OUT                   1

/*
 * I/O ports initial setup, this configuration is established soon after reset in the initialization code.
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
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2U))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2U))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_SENS_A) |\
                                     PIN_MODE_INPUT(GPIOA_SENS_B) |\
                                     PIN_MODE_INPUT(GPIOA_SENS_C) |\
                                     PIN_MODE_INPUT(GPIOA_RCPWM) |\
                                     PIN_MODE_INPUT(GPIOA_OC_ADJ) |\
                                     PIN_MODE_INPUT(GPIOA_EN_GATE) |\
                                     PIN_MODE_INPUT(GPIOA_DC_CAL) |\
                                     PIN_MODE_INPUT(GPIOA_MOTOR_PWM_1N) |\
                                     PIN_MODE_INPUT(GPIOA_MOTOR_PWM_1P) |\
                                     PIN_MODE_INPUT(GPIOA_MOTOR_PWM_2P) |\
                                     PIN_MODE_INPUT(GPIOA_MOTOR_PWM_3P) |\
                                     PIN_MODE_INPUT(GPIOA_USB_DM) |\
                                     PIN_MODE_INPUT(GPIOA_USB_DP) |\
                                     PIN_MODE_INPUT(GPIOA_SWDIO) |\
                                     PIN_MODE_INPUT(GPIOA_SWCLK) |\
                                     PIN_MODE_INPUT(GPIOA_USB_EN))

#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_SENS_A) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SENS_B) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SENS_C) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_RCPWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_OC_ADJ) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_EN_GATE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_DC_CAL) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_MOTOR_PWM_1N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_MOTOR_PWM_1P) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_MOTOR_PWM_2P) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_MOTOR_PWM_3P) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DP) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_EN))

#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_2M(GPIOA_SENS_A) |\
                                     PIN_OSPEED_2M(GPIOA_SENS_B) |\
                                     PIN_OSPEED_2M(GPIOA_SENS_C) |\
                                     PIN_OSPEED_2M(GPIOA_RCPWM) |\
                                     PIN_OSPEED_2M(GPIOA_OC_ADJ) |\
                                     PIN_OSPEED_2M(GPIOA_EN_GATE) |\
                                     PIN_OSPEED_2M(GPIOA_DC_CAL) |\
                                     PIN_OSPEED_2M(GPIOA_MOTOR_PWM_1N) |\
                                     PIN_OSPEED_2M(GPIOA_MOTOR_PWM_1P) |\
                                     PIN_OSPEED_2M(GPIOA_MOTOR_PWM_2P) |\
                                     PIN_OSPEED_2M(GPIOA_MOTOR_PWM_3P) |\
                                     PIN_OSPEED_2M(GPIOA_USB_DM) |\
                                     PIN_OSPEED_2M(GPIOA_USB_DP) |\
                                     PIN_OSPEED_2M(GPIOA_SWDIO) |\
                                     PIN_OSPEED_2M(GPIOA_SWCLK) |\
                                     PIN_OSPEED_2M(GPIOA_USB_EN))

#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOA_SENS_A) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_SENS_B) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_SENS_C) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_RCPWM) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_OC_ADJ) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_EN_GATE) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_DC_CAL) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_MOTOR_PWM_1N) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_MOTOR_PWM_1P) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_MOTOR_PWM_2P) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_MOTOR_PWM_3P) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_USB_DM) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_USB_DP) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWDIO) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_USB_EN))

#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_SENS_A) |\
                                     PIN_ODR_LOW(GPIOA_SENS_B) |\
                                     PIN_ODR_LOW(GPIOA_SENS_C) |\
                                     PIN_ODR_LOW(GPIOA_RCPWM) |\
                                     PIN_ODR_LOW(GPIOA_OC_ADJ) |\
                                     PIN_ODR_LOW(GPIOA_EN_GATE) |\
                                     PIN_ODR_LOW(GPIOA_DC_CAL) |\
                                     PIN_ODR_LOW(GPIOA_MOTOR_PWM_1N) |\
                                     PIN_ODR_LOW(GPIOA_MOTOR_PWM_1P) |\
                                     PIN_ODR_LOW(GPIOA_MOTOR_PWM_2P) |\
                                     PIN_ODR_LOW(GPIOA_MOTOR_PWM_3P) |\
                                     PIN_ODR_LOW(GPIOA_USB_DM) |\
                                     PIN_ODR_LOW(GPIOA_USB_DP) |\
                                     PIN_ODR_LOW(GPIOA_SWDIO) |\
                                     PIN_ODR_LOW(GPIOA_SWCLK) |\
                                     PIN_ODR_LOW(GPIOA_USB_EN))

#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_SENS_A,  0) |\
                                     PIN_AFIO_AF(GPIOA_SENS_B,  0) |\
                                     PIN_AFIO_AF(GPIOA_SENS_C,  0) |\
                                     PIN_AFIO_AF(GPIOA_RCPWM,  0) |\
                                     PIN_AFIO_AF(GPIOA_OC_ADJ,  0) |\
                                     PIN_AFIO_AF(GPIOA_EN_GATE,  0) |\
                                     PIN_AFIO_AF(GPIOA_DC_CAL,  0) |\
                                     PIN_AFIO_AF(GPIOA_MOTOR_PWM_1N,  0) )
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_MOTOR_PWM_1P,  0) |\
                                     PIN_AFIO_AF(GPIOA_MOTOR_PWM_2P,  0) |\
                                     PIN_AFIO_AF(GPIOA_MOTOR_PWM_3P, 0) |\
                                     PIN_AFIO_AF(GPIOA_USB_DM, 0) |\
                                     PIN_AFIO_AF(GPIOA_USB_DP, 0) |\
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0) |\
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0) |\
                                     PIN_AFIO_AF(GPIOA_USB_EN, 0))

/*
 * GPIOB
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_MOTOR_PWM_2N) |\
                                     PIN_MODE_INPUT(GPIOB_MOTOR_PWM_3N) |\
                                     PIN_MODE_INPUT(GPIOB_GAIN) |\
                                     PIN_MODE_INPUT(GPIOB_TEST_2) |\
                                     PIN_MODE_INPUT(GPIOB_TEST_3) |\
                                     PIN_MODE_INPUT(GPIOB_CAN2_RX) |\
                                     PIN_MODE_INPUT(GPIOB_CAN2_TX) |\
                                     PIN_MODE_INPUT(GPIOB_REQUIRE_GET_NODE_INFO) |\
                                     PIN_MODE_INPUT(GPIOB_CAN1_RX) |\
                                     PIN_MODE_INPUT(GPIOB_CAN1_TX) |\
                                     PIN_MODE_INPUT(GPIOB_UART_TX) |\
                                     PIN_MODE_INPUT(11) |\
                                     PIN_MODE_INPUT(12) |\
                                     PIN_MODE_INPUT(13) |\
                                     PIN_MODE_INPUT(14) |\
                                     PIN_MODE_INPUT(15))

#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_MOTOR_PWM_2N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_MOTOR_PWM_3N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_GAIN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_TEST_2) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_TEST_3) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN2_RX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN2_TX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_REQUIRE_GET_NODE_INFO) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN1_RX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN1_TX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_UART_TX) |\
                                     PIN_OTYPE_PUSHPULL(11) |\
                                     PIN_OTYPE_PUSHPULL(12) |\
                                     PIN_OTYPE_PUSHPULL(13) |\
                                     PIN_OTYPE_PUSHPULL(14) |\
                                     PIN_OTYPE_PUSHPULL(15))

#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_2M(GPIOB_MOTOR_PWM_2N) |\
                                     PIN_OSPEED_2M(GPIOB_MOTOR_PWM_3N) |\
                                     PIN_OSPEED_2M(GPIOB_GAIN) |\
                                     PIN_OSPEED_2M(GPIOB_TEST_2) |\
                                     PIN_OSPEED_2M(GPIOB_TEST_3) |\
                                     PIN_OSPEED_2M(GPIOB_CAN2_RX) |\
                                     PIN_OSPEED_2M(GPIOB_CAN2_TX) |\
                                     PIN_OSPEED_2M(GPIOB_REQUIRE_GET_NODE_INFO) |\
                                     PIN_OSPEED_2M(GPIOB_CAN1_RX) |\
                                     PIN_OSPEED_2M(GPIOB_CAN1_TX) |\
                                     PIN_OSPEED_2M(GPIOB_UART_TX) |\
                                     PIN_OSPEED_2M(11) |\
                                     PIN_OSPEED_2M(12) |\
                                     PIN_OSPEED_2M(13) |\
                                     PIN_OSPEED_2M(14) |\
                                     PIN_OSPEED_2M(15))

#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOB_MOTOR_PWM_2N) |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_MOTOR_PWM_3N) |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_GAIN) |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_TEST_2) |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_TEST_3) |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_CAN2_RX) |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_CAN2_TX) |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_REQUIRE_GET_NODE_INFO) |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_CAN1_RX) |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_CAN1_TX) |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_UART_TX) |\
                                     PIN_PUPDR_PULLDOWN(11) |\
                                     PIN_PUPDR_PULLDOWN(12) |\
                                     PIN_PUPDR_PULLDOWN(13) |\
                                     PIN_PUPDR_PULLDOWN(14) |\
                                     PIN_PUPDR_PULLDOWN(15))

#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_MOTOR_PWM_2N) |\
                                     PIN_ODR_LOW(GPIOB_MOTOR_PWM_3N) |\
                                     PIN_ODR_LOW(GPIOB_GAIN) |\
                                     PIN_ODR_LOW(GPIOB_TEST_2) |\
                                     PIN_ODR_LOW(GPIOB_TEST_3) |\
                                     PIN_ODR_LOW(GPIOB_CAN2_RX) |\
                                     PIN_ODR_LOW(GPIOB_CAN2_TX) |\
                                     PIN_ODR_LOW(GPIOB_REQUIRE_GET_NODE_INFO) |\
                                     PIN_ODR_LOW(GPIOB_CAN1_RX) |\
                                     PIN_ODR_LOW(GPIOB_CAN1_TX) |\
                                     PIN_ODR_LOW(GPIOB_UART_TX) |\
                                     PIN_ODR_LOW(11) |\
                                     PIN_ODR_LOW(12) |\
                                     PIN_ODR_LOW(13) |\
                                     PIN_ODR_LOW(14) |\
                                     PIN_ODR_LOW(15))

#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_MOTOR_PWM_2N,  0) |\
                                     PIN_AFIO_AF(GPIOB_MOTOR_PWM_3N,  0) |\
                                     PIN_AFIO_AF(GPIOB_GAIN,  0) |\
                                     PIN_AFIO_AF(GPIOB_TEST_2,  0) |\
                                     PIN_AFIO_AF(GPIOB_TEST_3,  0) |\
                                     PIN_AFIO_AF(GPIOB_CAN2_RX,  0) |\
                                     PIN_AFIO_AF(GPIOB_CAN2_TX,  0) |\
                                     PIN_AFIO_AF(GPIOB_REQUIRE_GET_NODE_INFO,  0) )
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_CAN1_RX,  0) |\
                                     PIN_AFIO_AF(GPIOB_CAN1_TX,  0) |\
                                     PIN_AFIO_AF(GPIOB_UART_TX, 0) |\
                                     PIN_AFIO_AF(11, 0) |\
                                     PIN_AFIO_AF(12, 0) |\
                                     PIN_AFIO_AF(13, 0) |\
                                     PIN_AFIO_AF(14, 0) |\
                                     PIN_AFIO_AF(15, 0))

/*
 * GPIOC
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_TEMPERATURE_SENS) |\
                                     PIN_MODE_INPUT(GPIOC_VBAT_SENS) |\
                                     PIN_MODE_INPUT(GPIOC_CURRENT_2_SENS) |\
                                     PIN_MODE_INPUT(GPIOC_CURRENT_1_SENS) |\
                                     PIN_MODE_INPUT(4) |\
                                     PIN_MODE_INPUT(GPIOC_UART_RX) |\
                                     PIN_MODE_INPUT(GPIOC_RPM_PULSE_FEEDBACK) |\
                                     PIN_MODE_INPUT(GPIOC_RGB_LED_RED) |\
                                     PIN_MODE_INPUT(GPIOC_RGB_LED_GREEN) |\
                                     PIN_MODE_INPUT(GPIOC_RGB_LED_BLUE) |\
                                     PIN_MODE_INPUT(10) |\
                                     PIN_MODE_INPUT(11) |\
                                     PIN_MODE_INPUT(GPIOC_TEST_4) |\
                                     PIN_MODE_INPUT(GPIOC_POWER_GOOD) |\
                                     PIN_MODE_INPUT(GPIOC_OVER_TEMP_WARNING_INVERSE) |\
                                     PIN_MODE_INPUT(GPIOC_DRIVER_FAULT_INVERSE))

#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_TEMPERATURE_SENS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_VBAT_SENS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_CURRENT_2_SENS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_CURRENT_1_SENS) |\
                                     PIN_OTYPE_PUSHPULL(4) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_UART_RX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_RPM_PULSE_FEEDBACK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_RGB_LED_RED) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_RGB_LED_GREEN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_RGB_LED_BLUE) |\
                                     PIN_OTYPE_PUSHPULL(10) |\
                                     PIN_OTYPE_PUSHPULL(11) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_TEST_4) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_POWER_GOOD) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_OVER_TEMP_WARNING_INVERSE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_DRIVER_FAULT_INVERSE))

#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_2M(GPIOC_TEMPERATURE_SENS) |\
                                     PIN_OSPEED_2M(GPIOC_VBAT_SENS) |\
                                     PIN_OSPEED_2M(GPIOC_CURRENT_2_SENS) |\
                                     PIN_OSPEED_2M(GPIOC_CURRENT_1_SENS) |\
                                     PIN_OSPEED_2M(4) |\
                                     PIN_OSPEED_2M(GPIOC_UART_RX) |\
                                     PIN_OSPEED_2M(GPIOC_RPM_PULSE_FEEDBACK) |\
                                     PIN_OSPEED_2M(GPIOC_RGB_LED_RED) |\
                                     PIN_OSPEED_2M(GPIOC_RGB_LED_GREEN) |\
                                     PIN_OSPEED_2M(GPIOC_RGB_LED_BLUE) |\
                                     PIN_OSPEED_2M(10) |\
                                     PIN_OSPEED_2M(11) |\
                                     PIN_OSPEED_2M(GPIOC_TEST_4) |\
                                     PIN_OSPEED_2M(GPIOC_POWER_GOOD) |\
                                     PIN_OSPEED_2M(GPIOC_OVER_TEMP_WARNING_INVERSE) |\
                                     PIN_OSPEED_2M(GPIOC_DRIVER_FAULT_INVERSE))

#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOC_TEMPERATURE_SENS) |\
                                     PIN_PUPDR_PULLDOWN(GPIOC_VBAT_SENS) |\
                                     PIN_PUPDR_PULLDOWN(GPIOC_CURRENT_2_SENS) |\
                                     PIN_PUPDR_PULLDOWN(GPIOC_CURRENT_1_SENS) |\
                                     PIN_PUPDR_PULLDOWN(4) |\
                                     PIN_PUPDR_PULLDOWN(GPIOC_UART_RX) |\
                                     PIN_PUPDR_PULLDOWN(GPIOC_RPM_PULSE_FEEDBACK) |\
                                     PIN_PUPDR_PULLDOWN(GPIOC_RGB_LED_RED) |\
                                     PIN_PUPDR_PULLDOWN(GPIOC_RGB_LED_GREEN) |\
                                     PIN_PUPDR_PULLDOWN(GPIOC_RGB_LED_BLUE) |\
                                     PIN_PUPDR_PULLDOWN(10) |\
                                     PIN_PUPDR_PULLDOWN(11) |\
                                     PIN_PUPDR_PULLDOWN(GPIOC_TEST_4) |\
                                     PIN_PUPDR_PULLDOWN(GPIOC_POWER_GOOD) |\
                                     PIN_PUPDR_PULLDOWN(GPIOC_OVER_TEMP_WARNING_INVERSE) |\
                                     PIN_PUPDR_PULLDOWN(GPIOC_DRIVER_FAULT_INVERSE))

#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_TEMPERATURE_SENS) |\
                                     PIN_ODR_LOW(GPIOC_VBAT_SENS) |\
                                     PIN_ODR_LOW(GPIOC_CURRENT_2_SENS) |\
                                     PIN_ODR_LOW(GPIOC_CURRENT_1_SENS) |\
                                     PIN_ODR_LOW(4) |\
                                     PIN_ODR_LOW(GPIOC_UART_RX) |\
                                     PIN_ODR_LOW(GPIOC_RPM_PULSE_FEEDBACK) |\
                                     PIN_ODR_LOW(GPIOC_RGB_LED_RED) |\
                                     PIN_ODR_LOW(GPIOC_RGB_LED_GREEN) |\
                                     PIN_ODR_LOW(GPIOC_RGB_LED_BLUE) |\
                                     PIN_ODR_LOW(10) |\
                                     PIN_ODR_LOW(11) |\
                                     PIN_ODR_LOW(GPIOC_TEST_4) |\
                                     PIN_ODR_LOW(GPIOC_POWER_GOOD) |\
                                     PIN_ODR_LOW(GPIOC_OVER_TEMP_WARNING_INVERSE) |\
                                     PIN_ODR_LOW(GPIOC_DRIVER_FAULT_INVERSE))

#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_TEMPERATURE_SENS,  0) |\
                                     PIN_AFIO_AF(GPIOC_VBAT_SENS,  0) |\
                                     PIN_AFIO_AF(GPIOC_CURRENT_2_SENS,  0) |\
                                     PIN_AFIO_AF(GPIOC_CURRENT_1_SENS,  0) |\
                                     PIN_AFIO_AF(4,  0) |\
                                     PIN_AFIO_AF(GPIOC_UART_RX,  0) |\
                                     PIN_AFIO_AF(GPIOC_RPM_PULSE_FEEDBACK,  0) |\
                                     PIN_AFIO_AF(GPIOC_RGB_LED_RED,  0) )
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_RGB_LED_GREEN,  0) |\
                                     PIN_AFIO_AF(GPIOC_RGB_LED_BLUE,  0) |\
                                     PIN_AFIO_AF(10, 0) |\
                                     PIN_AFIO_AF(11, 0) |\
                                     PIN_AFIO_AF(GPIOC_TEST_4, 0) |\
                                     PIN_AFIO_AF(GPIOC_POWER_GOOD, 0) |\
                                     PIN_AFIO_AF(GPIOC_OVER_TEMP_WARNING_INVERSE, 0) |\
                                     PIN_AFIO_AF(GPIOC_DRIVER_FAULT_INVERSE, 0))

/*
 * GPIOD
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(0) |\
                                     PIN_MODE_INPUT(1) |\
                                     PIN_MODE_INPUT(PORTD_TEST_1) |\
                                     PIN_MODE_INPUT(3) |\
                                     PIN_MODE_INPUT(4) |\
                                     PIN_MODE_INPUT(5) |\
                                     PIN_MODE_INPUT(6) |\
                                     PIN_MODE_INPUT(7) |\
                                     PIN_MODE_INPUT(8) |\
                                     PIN_MODE_INPUT(9) |\
                                     PIN_MODE_INPUT(10) |\
                                     PIN_MODE_INPUT(11) |\
                                     PIN_MODE_INPUT(12) |\
                                     PIN_MODE_INPUT(13) |\
                                     PIN_MODE_INPUT(14) |\
                                     PIN_MODE_INPUT(15))

#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(0) |\
                                     PIN_OTYPE_PUSHPULL(1) |\
                                     PIN_OTYPE_PUSHPULL(PORTD_TEST_1) |\
                                     PIN_OTYPE_PUSHPULL(3) |\
                                     PIN_OTYPE_PUSHPULL(4) |\
                                     PIN_OTYPE_PUSHPULL(5) |\
                                     PIN_OTYPE_PUSHPULL(6) |\
                                     PIN_OTYPE_PUSHPULL(7) |\
                                     PIN_OTYPE_PUSHPULL(8) |\
                                     PIN_OTYPE_PUSHPULL(9) |\
                                     PIN_OTYPE_PUSHPULL(10) |\
                                     PIN_OTYPE_PUSHPULL(11) |\
                                     PIN_OTYPE_PUSHPULL(12) |\
                                     PIN_OTYPE_PUSHPULL(13) |\
                                     PIN_OTYPE_PUSHPULL(14) |\
                                     PIN_OTYPE_PUSHPULL(15))

#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_2M(0) |\
                                     PIN_OSPEED_2M(1) |\
                                     PIN_OSPEED_2M(PORTD_TEST_1) |\
                                     PIN_OSPEED_2M(3) |\
                                     PIN_OSPEED_2M(4) |\
                                     PIN_OSPEED_2M(5) |\
                                     PIN_OSPEED_2M(6) |\
                                     PIN_OSPEED_2M(7) |\
                                     PIN_OSPEED_2M(8) |\
                                     PIN_OSPEED_2M(9) |\
                                     PIN_OSPEED_2M(10) |\
                                     PIN_OSPEED_2M(11) |\
                                     PIN_OSPEED_2M(12) |\
                                     PIN_OSPEED_2M(13) |\
                                     PIN_OSPEED_2M(14) |\
                                     PIN_OSPEED_2M(15))

#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLDOWN(0) |\
                                     PIN_PUPDR_PULLDOWN(1) |\
                                     PIN_PUPDR_PULLDOWN(PORTD_TEST_1) |\
                                     PIN_PUPDR_PULLDOWN(3) |\
                                     PIN_PUPDR_PULLDOWN(4) |\
                                     PIN_PUPDR_PULLDOWN(5) |\
                                     PIN_PUPDR_PULLDOWN(6) |\
                                     PIN_PUPDR_PULLDOWN(7) |\
                                     PIN_PUPDR_PULLDOWN(8) |\
                                     PIN_PUPDR_PULLDOWN(9) |\
                                     PIN_PUPDR_PULLDOWN(10) |\
                                     PIN_PUPDR_PULLDOWN(11) |\
                                     PIN_PUPDR_PULLDOWN(12) |\
                                     PIN_PUPDR_PULLDOWN(13) |\
                                     PIN_PUPDR_PULLDOWN(14) |\
                                     PIN_PUPDR_PULLDOWN(15))

#define VAL_GPIOD_ODR               (PIN_ODR_LOW(0) |\
                                     PIN_ODR_LOW(1) |\
                                     PIN_ODR_LOW(PORTD_TEST_1) |\
                                     PIN_ODR_LOW(3) |\
                                     PIN_ODR_LOW(4) |\
                                     PIN_ODR_LOW(5) |\
                                     PIN_ODR_LOW(6) |\
                                     PIN_ODR_LOW(7) |\
                                     PIN_ODR_LOW(8) |\
                                     PIN_ODR_LOW(9) |\
                                     PIN_ODR_LOW(10) |\
                                     PIN_ODR_LOW(11) |\
                                     PIN_ODR_LOW(12) |\
                                     PIN_ODR_LOW(13) |\
                                     PIN_ODR_LOW(14) |\
                                     PIN_ODR_LOW(15))

#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(0,  0) |\
                                     PIN_AFIO_AF(1,  0) |\
                                     PIN_AFIO_AF(PORTD_TEST_1,  0) |\
                                     PIN_AFIO_AF(3,  0) |\
                                     PIN_AFIO_AF(4,  0) |\
                                     PIN_AFIO_AF(5,  0) |\
                                     PIN_AFIO_AF(6,  0) |\
                                     PIN_AFIO_AF(7,  0) )
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(8,  0) |\
                                     PIN_AFIO_AF(9,  0) |\
                                     PIN_AFIO_AF(10, 0) |\
                                     PIN_AFIO_AF(11, 0) |\
                                     PIN_AFIO_AF(12, 0) |\
                                     PIN_AFIO_AF(13, 0) |\
                                     PIN_AFIO_AF(14, 0) |\
                                     PIN_AFIO_AF(15, 0))

/*
 * GPIOE
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(0) |\
                                     PIN_MODE_INPUT(1) |\
                                     PIN_MODE_INPUT(2) |\
                                     PIN_MODE_INPUT(3) |\
                                     PIN_MODE_INPUT(4) |\
                                     PIN_MODE_INPUT(5) |\
                                     PIN_MODE_INPUT(6) |\
                                     PIN_MODE_INPUT(7) |\
                                     PIN_MODE_INPUT(8) |\
                                     PIN_MODE_INPUT(9) |\
                                     PIN_MODE_INPUT(10) |\
                                     PIN_MODE_INPUT(11) |\
                                     PIN_MODE_INPUT(12) |\
                                     PIN_MODE_INPUT(13) |\
                                     PIN_MODE_INPUT(14) |\
                                     PIN_MODE_INPUT(15))

#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(0) |\
                                     PIN_OTYPE_PUSHPULL(1) |\
                                     PIN_OTYPE_PUSHPULL(2) |\
                                     PIN_OTYPE_PUSHPULL(3) |\
                                     PIN_OTYPE_PUSHPULL(4) |\
                                     PIN_OTYPE_PUSHPULL(5) |\
                                     PIN_OTYPE_PUSHPULL(6) |\
                                     PIN_OTYPE_PUSHPULL(7) |\
                                     PIN_OTYPE_PUSHPULL(8) |\
                                     PIN_OTYPE_PUSHPULL(9) |\
                                     PIN_OTYPE_PUSHPULL(10) |\
                                     PIN_OTYPE_PUSHPULL(11) |\
                                     PIN_OTYPE_PUSHPULL(12) |\
                                     PIN_OTYPE_PUSHPULL(13) |\
                                     PIN_OTYPE_PUSHPULL(14) |\
                                     PIN_OTYPE_PUSHPULL(15))

#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_2M(0) |\
                                     PIN_OSPEED_2M(1) |\
                                     PIN_OSPEED_2M(2) |\
                                     PIN_OSPEED_2M(3) |\
                                     PIN_OSPEED_2M(4) |\
                                     PIN_OSPEED_2M(5) |\
                                     PIN_OSPEED_2M(6) |\
                                     PIN_OSPEED_2M(7) |\
                                     PIN_OSPEED_2M(8) |\
                                     PIN_OSPEED_2M(9) |\
                                     PIN_OSPEED_2M(10) |\
                                     PIN_OSPEED_2M(11) |\
                                     PIN_OSPEED_2M(12) |\
                                     PIN_OSPEED_2M(13) |\
                                     PIN_OSPEED_2M(14) |\
                                     PIN_OSPEED_2M(15))

#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLDOWN(0) |\
                                     PIN_PUPDR_PULLDOWN(1) |\
                                     PIN_PUPDR_PULLDOWN(2) |\
                                     PIN_PUPDR_PULLDOWN(3) |\
                                     PIN_PUPDR_PULLDOWN(4) |\
                                     PIN_PUPDR_PULLDOWN(5) |\
                                     PIN_PUPDR_PULLDOWN(6) |\
                                     PIN_PUPDR_PULLDOWN(7) |\
                                     PIN_PUPDR_PULLDOWN(8) |\
                                     PIN_PUPDR_PULLDOWN(9) |\
                                     PIN_PUPDR_PULLDOWN(10) |\
                                     PIN_PUPDR_PULLDOWN(11) |\
                                     PIN_PUPDR_PULLDOWN(12) |\
                                     PIN_PUPDR_PULLDOWN(13) |\
                                     PIN_PUPDR_PULLDOWN(14) |\
                                     PIN_PUPDR_PULLDOWN(15))

#define VAL_GPIOE_ODR               (PIN_ODR_LOW(0) |\
                                     PIN_ODR_LOW(1) |\
                                     PIN_ODR_LOW(2) |\
                                     PIN_ODR_LOW(3) |\
                                     PIN_ODR_LOW(4) |\
                                     PIN_ODR_LOW(5) |\
                                     PIN_ODR_LOW(6) |\
                                     PIN_ODR_LOW(7) |\
                                     PIN_ODR_LOW(8) |\
                                     PIN_ODR_LOW(9) |\
                                     PIN_ODR_LOW(10) |\
                                     PIN_ODR_LOW(11) |\
                                     PIN_ODR_LOW(12) |\
                                     PIN_ODR_LOW(13) |\
                                     PIN_ODR_LOW(14) |\
                                     PIN_ODR_LOW(15))

#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(0,  0) |\
                                     PIN_AFIO_AF(1,  0) |\
                                     PIN_AFIO_AF(2,  0) |\
                                     PIN_AFIO_AF(3,  0) |\
                                     PIN_AFIO_AF(4,  0) |\
                                     PIN_AFIO_AF(5,  0) |\
                                     PIN_AFIO_AF(6,  0) |\
                                     PIN_AFIO_AF(7,  0) )
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(8,  0) |\
                                     PIN_AFIO_AF(9,  0) |\
                                     PIN_AFIO_AF(10, 0) |\
                                     PIN_AFIO_AF(11, 0) |\
                                     PIN_AFIO_AF(12, 0) |\
                                     PIN_AFIO_AF(13, 0) |\
                                     PIN_AFIO_AF(14, 0) |\
                                     PIN_AFIO_AF(15, 0))

/*
 * GPIOF
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(0) |\
                                     PIN_MODE_INPUT(1) |\
                                     PIN_MODE_INPUT(2) |\
                                     PIN_MODE_INPUT(3) |\
                                     PIN_MODE_INPUT(4) |\
                                     PIN_MODE_INPUT(5) |\
                                     PIN_MODE_INPUT(6) |\
                                     PIN_MODE_INPUT(7) |\
                                     PIN_MODE_INPUT(8) |\
                                     PIN_MODE_INPUT(9) |\
                                     PIN_MODE_INPUT(10) |\
                                     PIN_MODE_INPUT(11) |\
                                     PIN_MODE_INPUT(12) |\
                                     PIN_MODE_INPUT(13) |\
                                     PIN_MODE_INPUT(14) |\
                                     PIN_MODE_INPUT(15))

#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(0) |\
                                     PIN_OTYPE_PUSHPULL(1) |\
                                     PIN_OTYPE_PUSHPULL(2) |\
                                     PIN_OTYPE_PUSHPULL(3) |\
                                     PIN_OTYPE_PUSHPULL(4) |\
                                     PIN_OTYPE_PUSHPULL(5) |\
                                     PIN_OTYPE_PUSHPULL(6) |\
                                     PIN_OTYPE_PUSHPULL(7) |\
                                     PIN_OTYPE_PUSHPULL(8) |\
                                     PIN_OTYPE_PUSHPULL(9) |\
                                     PIN_OTYPE_PUSHPULL(10) |\
                                     PIN_OTYPE_PUSHPULL(11) |\
                                     PIN_OTYPE_PUSHPULL(12) |\
                                     PIN_OTYPE_PUSHPULL(13) |\
                                     PIN_OTYPE_PUSHPULL(14) |\
                                     PIN_OTYPE_PUSHPULL(15))

#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_2M(0) |\
                                     PIN_OSPEED_2M(1) |\
                                     PIN_OSPEED_2M(2) |\
                                     PIN_OSPEED_2M(3) |\
                                     PIN_OSPEED_2M(4) |\
                                     PIN_OSPEED_2M(5) |\
                                     PIN_OSPEED_2M(6) |\
                                     PIN_OSPEED_2M(7) |\
                                     PIN_OSPEED_2M(8) |\
                                     PIN_OSPEED_2M(9) |\
                                     PIN_OSPEED_2M(10) |\
                                     PIN_OSPEED_2M(11) |\
                                     PIN_OSPEED_2M(12) |\
                                     PIN_OSPEED_2M(13) |\
                                     PIN_OSPEED_2M(14) |\
                                     PIN_OSPEED_2M(15))

#define VAL_GPIOF_PUPDR             (PIN_PUPDR_PULLDOWN(0) |\
                                     PIN_PUPDR_PULLDOWN(1) |\
                                     PIN_PUPDR_PULLDOWN(2) |\
                                     PIN_PUPDR_PULLDOWN(3) |\
                                     PIN_PUPDR_PULLDOWN(4) |\
                                     PIN_PUPDR_PULLDOWN(5) |\
                                     PIN_PUPDR_PULLDOWN(6) |\
                                     PIN_PUPDR_PULLDOWN(7) |\
                                     PIN_PUPDR_PULLDOWN(8) |\
                                     PIN_PUPDR_PULLDOWN(9) |\
                                     PIN_PUPDR_PULLDOWN(10) |\
                                     PIN_PUPDR_PULLDOWN(11) |\
                                     PIN_PUPDR_PULLDOWN(12) |\
                                     PIN_PUPDR_PULLDOWN(13) |\
                                     PIN_PUPDR_PULLDOWN(14) |\
                                     PIN_PUPDR_PULLDOWN(15))

#define VAL_GPIOF_ODR               (PIN_ODR_LOW(0) |\
                                     PIN_ODR_LOW(1) |\
                                     PIN_ODR_LOW(2) |\
                                     PIN_ODR_LOW(3) |\
                                     PIN_ODR_LOW(4) |\
                                     PIN_ODR_LOW(5) |\
                                     PIN_ODR_LOW(6) |\
                                     PIN_ODR_LOW(7) |\
                                     PIN_ODR_LOW(8) |\
                                     PIN_ODR_LOW(9) |\
                                     PIN_ODR_LOW(10) |\
                                     PIN_ODR_LOW(11) |\
                                     PIN_ODR_LOW(12) |\
                                     PIN_ODR_LOW(13) |\
                                     PIN_ODR_LOW(14) |\
                                     PIN_ODR_LOW(15))

#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(0,  0) |\
                                     PIN_AFIO_AF(1,  0) |\
                                     PIN_AFIO_AF(2,  0) |\
                                     PIN_AFIO_AF(3,  0) |\
                                     PIN_AFIO_AF(4,  0) |\
                                     PIN_AFIO_AF(5,  0) |\
                                     PIN_AFIO_AF(6,  0) |\
                                     PIN_AFIO_AF(7,  0) )
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(8,  0) |\
                                     PIN_AFIO_AF(9,  0) |\
                                     PIN_AFIO_AF(10, 0) |\
                                     PIN_AFIO_AF(11, 0) |\
                                     PIN_AFIO_AF(12, 0) |\
                                     PIN_AFIO_AF(13, 0) |\
                                     PIN_AFIO_AF(14, 0) |\
                                     PIN_AFIO_AF(15, 0))

/*
 * GPIOG
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(0) |\
                                     PIN_MODE_INPUT(1) |\
                                     PIN_MODE_INPUT(2) |\
                                     PIN_MODE_INPUT(3) |\
                                     PIN_MODE_INPUT(4) |\
                                     PIN_MODE_INPUT(5) |\
                                     PIN_MODE_INPUT(6) |\
                                     PIN_MODE_INPUT(7) |\
                                     PIN_MODE_INPUT(8) |\
                                     PIN_MODE_INPUT(9) |\
                                     PIN_MODE_INPUT(10) |\
                                     PIN_MODE_INPUT(11) |\
                                     PIN_MODE_INPUT(12) |\
                                     PIN_MODE_INPUT(13) |\
                                     PIN_MODE_INPUT(14) |\
                                     PIN_MODE_INPUT(15))

#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(0) |\
                                     PIN_OTYPE_PUSHPULL(1) |\
                                     PIN_OTYPE_PUSHPULL(2) |\
                                     PIN_OTYPE_PUSHPULL(3) |\
                                     PIN_OTYPE_PUSHPULL(4) |\
                                     PIN_OTYPE_PUSHPULL(5) |\
                                     PIN_OTYPE_PUSHPULL(6) |\
                                     PIN_OTYPE_PUSHPULL(7) |\
                                     PIN_OTYPE_PUSHPULL(8) |\
                                     PIN_OTYPE_PUSHPULL(9) |\
                                     PIN_OTYPE_PUSHPULL(10) |\
                                     PIN_OTYPE_PUSHPULL(11) |\
                                     PIN_OTYPE_PUSHPULL(12) |\
                                     PIN_OTYPE_PUSHPULL(13) |\
                                     PIN_OTYPE_PUSHPULL(14) |\
                                     PIN_OTYPE_PUSHPULL(15))

#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_2M(0) |\
                                     PIN_OSPEED_2M(1) |\
                                     PIN_OSPEED_2M(2) |\
                                     PIN_OSPEED_2M(3) |\
                                     PIN_OSPEED_2M(4) |\
                                     PIN_OSPEED_2M(5) |\
                                     PIN_OSPEED_2M(6) |\
                                     PIN_OSPEED_2M(7) |\
                                     PIN_OSPEED_2M(8) |\
                                     PIN_OSPEED_2M(9) |\
                                     PIN_OSPEED_2M(10) |\
                                     PIN_OSPEED_2M(11) |\
                                     PIN_OSPEED_2M(12) |\
                                     PIN_OSPEED_2M(13) |\
                                     PIN_OSPEED_2M(14) |\
                                     PIN_OSPEED_2M(15))

#define VAL_GPIOG_PUPDR             (PIN_PUPDR_PULLDOWN(0) |\
                                     PIN_PUPDR_PULLDOWN(1) |\
                                     PIN_PUPDR_PULLDOWN(2) |\
                                     PIN_PUPDR_PULLDOWN(3) |\
                                     PIN_PUPDR_PULLDOWN(4) |\
                                     PIN_PUPDR_PULLDOWN(5) |\
                                     PIN_PUPDR_PULLDOWN(6) |\
                                     PIN_PUPDR_PULLDOWN(7) |\
                                     PIN_PUPDR_PULLDOWN(8) |\
                                     PIN_PUPDR_PULLDOWN(9) |\
                                     PIN_PUPDR_PULLDOWN(10) |\
                                     PIN_PUPDR_PULLDOWN(11) |\
                                     PIN_PUPDR_PULLDOWN(12) |\
                                     PIN_PUPDR_PULLDOWN(13) |\
                                     PIN_PUPDR_PULLDOWN(14) |\
                                     PIN_PUPDR_PULLDOWN(15))

#define VAL_GPIOG_ODR               (PIN_ODR_LOW(0) |\
                                     PIN_ODR_LOW(1) |\
                                     PIN_ODR_LOW(2) |\
                                     PIN_ODR_LOW(3) |\
                                     PIN_ODR_LOW(4) |\
                                     PIN_ODR_LOW(5) |\
                                     PIN_ODR_LOW(6) |\
                                     PIN_ODR_LOW(7) |\
                                     PIN_ODR_LOW(8) |\
                                     PIN_ODR_LOW(9) |\
                                     PIN_ODR_LOW(10) |\
                                     PIN_ODR_LOW(11) |\
                                     PIN_ODR_LOW(12) |\
                                     PIN_ODR_LOW(13) |\
                                     PIN_ODR_LOW(14) |\
                                     PIN_ODR_LOW(15))

#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(0,  0) |\
                                     PIN_AFIO_AF(1,  0) |\
                                     PIN_AFIO_AF(2,  0) |\
                                     PIN_AFIO_AF(3,  0) |\
                                     PIN_AFIO_AF(4,  0) |\
                                     PIN_AFIO_AF(5,  0) |\
                                     PIN_AFIO_AF(6,  0) |\
                                     PIN_AFIO_AF(7,  0) )
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(8,  0) |\
                                     PIN_AFIO_AF(9,  0) |\
                                     PIN_AFIO_AF(10, 0) |\
                                     PIN_AFIO_AF(11, 0) |\
                                     PIN_AFIO_AF(12, 0) |\
                                     PIN_AFIO_AF(13, 0) |\
                                     PIN_AFIO_AF(14, 0) |\
                                     PIN_AFIO_AF(15, 0))

/*
 * GPIOH
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |\
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |\
                                     PIN_MODE_INPUT(2) |\
                                     PIN_MODE_INPUT(3) |\
                                     PIN_MODE_INPUT(4) |\
                                     PIN_MODE_INPUT(5) |\
                                     PIN_MODE_INPUT(6) |\
                                     PIN_MODE_INPUT(7) |\
                                     PIN_MODE_INPUT(8) |\
                                     PIN_MODE_INPUT(9) |\
                                     PIN_MODE_INPUT(10) |\
                                     PIN_MODE_INPUT(11) |\
                                     PIN_MODE_INPUT(12) |\
                                     PIN_MODE_INPUT(13) |\
                                     PIN_MODE_INPUT(14) |\
                                     PIN_MODE_INPUT(15))

#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |\
                                     PIN_OTYPE_PUSHPULL(2) |\
                                     PIN_OTYPE_PUSHPULL(3) |\
                                     PIN_OTYPE_PUSHPULL(4) |\
                                     PIN_OTYPE_PUSHPULL(5) |\
                                     PIN_OTYPE_PUSHPULL(6) |\
                                     PIN_OTYPE_PUSHPULL(7) |\
                                     PIN_OTYPE_PUSHPULL(8) |\
                                     PIN_OTYPE_PUSHPULL(9) |\
                                     PIN_OTYPE_PUSHPULL(10) |\
                                     PIN_OTYPE_PUSHPULL(11) |\
                                     PIN_OTYPE_PUSHPULL(12) |\
                                     PIN_OTYPE_PUSHPULL(13) |\
                                     PIN_OTYPE_PUSHPULL(14) |\
                                     PIN_OTYPE_PUSHPULL(15))

#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_2M(GPIOH_OSC_IN) |\
                                     PIN_OSPEED_2M(GPIOH_OSC_OUT) |\
                                     PIN_OSPEED_2M(2) |\
                                     PIN_OSPEED_2M(3) |\
                                     PIN_OSPEED_2M(4) |\
                                     PIN_OSPEED_2M(5) |\
                                     PIN_OSPEED_2M(6) |\
                                     PIN_OSPEED_2M(7) |\
                                     PIN_OSPEED_2M(8) |\
                                     PIN_OSPEED_2M(9) |\
                                     PIN_OSPEED_2M(10) |\
                                     PIN_OSPEED_2M(11) |\
                                     PIN_OSPEED_2M(12) |\
                                     PIN_OSPEED_2M(13) |\
                                     PIN_OSPEED_2M(14) |\
                                     PIN_OSPEED_2M(15))

#define VAL_GPIOH_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOH_OSC_IN) |\
                                     PIN_PUPDR_PULLDOWN(GPIOH_OSC_OUT) |\
                                     PIN_PUPDR_PULLDOWN(2) |\
                                     PIN_PUPDR_PULLDOWN(3) |\
                                     PIN_PUPDR_PULLDOWN(4) |\
                                     PIN_PUPDR_PULLDOWN(5) |\
                                     PIN_PUPDR_PULLDOWN(6) |\
                                     PIN_PUPDR_PULLDOWN(7) |\
                                     PIN_PUPDR_PULLDOWN(8) |\
                                     PIN_PUPDR_PULLDOWN(9) |\
                                     PIN_PUPDR_PULLDOWN(10) |\
                                     PIN_PUPDR_PULLDOWN(11) |\
                                     PIN_PUPDR_PULLDOWN(12) |\
                                     PIN_PUPDR_PULLDOWN(13) |\
                                     PIN_PUPDR_PULLDOWN(14) |\
                                     PIN_PUPDR_PULLDOWN(15))

#define VAL_GPIOH_ODR               (PIN_ODR_LOW(GPIOH_OSC_IN) |\
                                     PIN_ODR_LOW(GPIOH_OSC_OUT) |\
                                     PIN_ODR_LOW(2) |\
                                     PIN_ODR_LOW(3) |\
                                     PIN_ODR_LOW(4) |\
                                     PIN_ODR_LOW(5) |\
                                     PIN_ODR_LOW(6) |\
                                     PIN_ODR_LOW(7) |\
                                     PIN_ODR_LOW(8) |\
                                     PIN_ODR_LOW(9) |\
                                     PIN_ODR_LOW(10) |\
                                     PIN_ODR_LOW(11) |\
                                     PIN_ODR_LOW(12) |\
                                     PIN_ODR_LOW(13) |\
                                     PIN_ODR_LOW(14) |\
                                     PIN_ODR_LOW(15))

#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN,  0) |\
                                     PIN_AFIO_AF(GPIOH_OSC_OUT,  0) |\
                                     PIN_AFIO_AF(2,  0) |\
                                     PIN_AFIO_AF(3,  0) |\
                                     PIN_AFIO_AF(4,  0) |\
                                     PIN_AFIO_AF(5,  0) |\
                                     PIN_AFIO_AF(6,  0) |\
                                     PIN_AFIO_AF(7,  0) )
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(8,  0) |\
                                     PIN_AFIO_AF(9,  0) |\
                                     PIN_AFIO_AF(10, 0) |\
                                     PIN_AFIO_AF(11, 0) |\
                                     PIN_AFIO_AF(12, 0) |\
                                     PIN_AFIO_AF(13, 0) |\
                                     PIN_AFIO_AF(14, 0) |\
                                     PIN_AFIO_AF(15, 0))


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
    void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */
