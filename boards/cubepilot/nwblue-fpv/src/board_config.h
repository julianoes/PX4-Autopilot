/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

/**
 * @file board_config.h
 *
 * CubePilot NW Blue FPV internal definitions.
 * STM32H747XI on the CubePilot CubeNode (CM1) module on this carrier.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

/* If NuttX is built without support for SMPS it can brick the hardware. */
#include "hardware/stm32h7x3xx_pwr.h"
#if STM32_PWR_CR3_SMPSEXTHP != (1 << 3)
#  error "No SMPS support in NuttX submodule");
#endif

#define BOARD_HAS_USB_VALID            1
#define BOARD_HAS_NBAT_V               1
#define BOARD_HAS_NBAT_I               1

/* LEDs (active-low: anode to +3.3V, cathode through resistor to GPIO) */

#define GPIO_nLED_RED       /* PF12 */ (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTF | GPIO_PIN12)
#define GPIO_nLED_GREEN     /* PF13 */ (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTF | GPIO_PIN13)
#define GPIO_nLED_BLUE      /* PA5  */ (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN5)

/* Status LED (active-low) */
#define GPIO_LED_STATUS     /* PC0  */ (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN0)

#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED            LED_RED

/* ADC channels (all on ADC3, per schematic):
 *   PC2_C / ADC3_INP0   CURRENT_SENSE   (ESC connector pin 2, R28 = 1k shunt)
 *   PC3_C / ADC3_INP1   3V3_SENSE       (R18 10k pull-down)
 *   PF5   / ADC3_INP4   10V_SENSE       (R13/R14 30k:10k divider, ~4.0x)
 *   PF3   / ADC3_INP5   SYSVIN_SENSE    (R19/R20/R21 200k:5k divider, ~41x)
 *   PF4   / ADC3_INP9   5V_SENSE        (R15/R16/R17 divider)
 *
 * PC2_C and PC3_C are direct (analog-only) channels - they don't have a GPIO
 * configuration in the digital sense, but PX4 still wants their GPIO_ADC3_INPx
 * macros listed in PX4_ADC_GPIO and the channel numbers in ADC_CHANNELS.
 */

/* All carrier sense channels live on ADC3, so make ADC3 the primary ADC. */
#define SYSTEM_ADC_BASE             STM32_ADC3_BASE

/* The 3.3 V sense pin (PC3_C) is tied directly to the 3.3 V rail with only a
 * 10 k pull-down to ground, so the ADC sees the full rail voltage (no divider).
 */
#define ADC_3V3_SCALE               (1.0f)

#define ADC1_CH(n)                  (n)
#define ADC3_CH(n)                  (n)

/* PC2_C (ADC3_INP0) and PC3_C (ADC3_INP1) are direct analog channels and
 * don't go through GPIO config (no GPIO_PIN2_C/PIN3_C macros in the H7
 * pinmap). They're enabled via the ADC_CHANNELS mask alone.
 */
#define PX4_ADC_GPIO  \
	/* PF5   */ GPIO_ADC3_INP4,  \
	/* PF3   */ GPIO_ADC3_INP5,  \
	/* PF4   */ GPIO_ADC3_INP9

#define ADC_BATTERY_VOLTAGE_CHANNEL          /* PF3   */ ADC3_CH(5)   /* SYSVIN  */
#define ADC_BATTERY_CURRENT_CHANNEL          /* PC2_C */ ADC3_CH(0)   /* CURRENT */
#define ADC_SCALED_V5_CHANNEL                /* PF4   */ ADC3_CH(9)   /* 5V      */
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL   /* PC3_C */ ADC3_CH(1)   /* 3V3     */

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)        | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL)        | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL))

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS  9
#define BOARD_NUM_IO_TIMERS         4

/* Tone alarm output - GPIO mode (no timer); BUZZER on PC8 */
#define GPIO_TONE_ALARM_IDLE    /* PC8 */ (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN8)
#define GPIO_TONE_ALARM_GPIO    /* PC8 */ (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET   | GPIO_PORTC | GPIO_PIN8)

/* USB OTG FS - PA9 OTG_FS_VBUS sensing */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT | GPIO_PULLDOWN | GPIO_SPEED_100MHz | GPIO_PORTA | GPIO_PIN9)

/* High-resolution timer - TIM8 internal time-base, no GPIO export */
#define HRT_TIMER               8
#define HRT_TIMER_CHANNEL       3

/* SDIO */
#define SDIO_SLOTNO             0
#define SDIO_MINOR              0

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_SERVO_VALID   (1)
#define BOARD_ADC_BRICK1_VALID  (1)

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define BOARD_HAS_STATIC_MANIFEST 1

#define FLASH_BASED_PARAMS

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,        \
		GPIO_TONE_ALARM_IDLE,\
		GPIO_OTGFS_VBUS,     \
		GPIO_CAN1_TX,        \
		GPIO_CAN1_RX,        \
		GPIO_CAN_SHUTDOWN,   \
		GPIO_CAN_SLEEP,      \
	}

__BEGIN_DECLS
#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);
extern void board_peripheral_reset(int ms);

#ifdef CONFIG_USBHOST
extern int stm32_usbhost_initialize(void);
#endif

#include <px4_platform_common/board_common.h>
#endif /* __ASSEMBLY__ */
__END_DECLS
