/****************************************************************************
 *
 * Copyright 2017 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * configs/lm3s6965-ek/src/up_leds.c
 * arch/arm/src/board/up_leds.c
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "tiva_gpio.h"
#include "lm3s6965ek_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* CONFIG_DEBUG_LEDS enables debug output from this file (needs CONFIG_DEBUG
 * with CONFIG_DEBUG_VERBOSE too)
 */

#ifdef CONFIG_DEBUG_LEDS
#define leddbg  lldbg
#define ledvdbg llvdbg
#else
#define leddbg(...)
#define ledvdbg(...)
#endif

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_LEDS
#define led_dumpgpio(m) tiva_dumpgpio(LED_GPIO, m)
#else
#define led_dumpgpio(m)
#endif

#ifdef CONFIG_ARCH_LEDS
/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_nest;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_led_initialize
 ****************************************************************************/

void board_led_initialize(void)
{
	leddbg("Initializing\n");

	/* Configure Port E, Bit 1 as an output, initial value=OFF */

	led_dumpgpio("board_led_initialize before tiva_configgpio()");
	tiva_configgpio(LED_GPIO);
	led_dumpgpio("board_led_initialize after tiva_configgpio()");
	g_nest = 0;
}

/****************************************************************************
 * Name: board_led_on
 ****************************************************************************/

void board_led_on(int led)
{
	switch (led) {
	case LED_STARTED:
	case LED_HEAPALLOCATE:
	default:
		break;

	case LED_INIRQ:
	case LED_SIGNAL:
	case LED_ASSERTION:
	case LED_PANIC:
		g_nest++;
	case LED_IRQSENABLED:
	case LED_STACKCREATED:
		led_dumpgpio("board_led_on: before tiva_gpiowrite()");
		tiva_gpiowrite(LED_GPIO, false);
		led_dumpgpio("board_led_on: after tiva_gpiowrite()");
		break;
	}
}

/****************************************************************************
 * Name: board_led_off
 ****************************************************************************/

void board_led_off(int led)
{
	switch (led) {
	case LED_IRQSENABLED:
	case LED_STACKCREATED:
	case LED_STARTED:
	case LED_HEAPALLOCATE:
	default:
		break;

	case LED_INIRQ:
	case LED_SIGNAL:
	case LED_ASSERTION:
	case LED_PANIC:
		if (--g_nest <= 0) {
			led_dumpgpio("board_led_off: before tiva_gpiowrite()");
			tiva_gpiowrite(LED_GPIO, true);
			led_dumpgpio("board_led_off: after tiva_gpiowrite()");
		}
		break;
	}
}

#endif							/* CONFIG_ARCH_LEDS */
