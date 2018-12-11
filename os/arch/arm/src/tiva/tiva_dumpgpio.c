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
 * arch/arm/src/tiva/tiva_dumpgpio.c
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <tinyara/arch.h>

#include "up_arch.h"

#include "chip.h"
#include "tiva_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

static const char g_portchar[TIVA_NPORTS] = {
#if TIVA_NPORTS > 0
	'A'
#endif
#if TIVA_NPORTS > 1
	, 'B'
#endif
#if TIVA_NPORTS > 2
	, 'C'
#endif
#if TIVA_NPORTS > 3
	, 'D'
#endif
#if TIVA_NPORTS > 4
	, 'E'
#endif
#if TIVA_NPORTS > 5
	, 'F'
#endif
#if TIVA_NPORTS > 6
	, 'G'
#endif
#if TIVA_NPORTS > 7
	, 'H'
#endif
#if TIVA_NPORTS > 8
	, 'J'
#endif
#if TIVA_NPORTS > 9
	, 'K'
#endif
#if TIVA_NPORTS > 10
	, 'L'
#endif
#if TIVA_NPORTS > 11
	, 'M'
#endif
#if TIVA_NPORTS > 12
	, 'N'
#endif
#if TIVA_NPORTS > 13
	, 'P'
#endif
#if TIVA_NPORTS > 14
	, 'Q'
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_gpioport
 *
 * Description:
 *   Given a GPIO enumeration value, return the base address of the
 *   associated GPIO registers.
 *
 ****************************************************************************/

static inline uint8_t tiva_gpioport(int port)
{
	return port < TIVA_NPORTS ? g_portchar[port] : '?';
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  tiva_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

int tiva_dumpgpio(uint32_t pinset, const char *msg)
{
#ifdef CONFIG_DEBUG
	irqstate_t flags;
	unsigned int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
	uintptr_t base;
#ifdef TIVA_SYSCON_RCGCGPIO
	uint32_t rcgcgpio;
#else
	uint32_t rcgc2;
#endif
	bool enabled;

	/* Get the base address associated with the GPIO port */

	base = tiva_gpiobaseaddress(port);
	DEBUGASSERT(base != 0);

	/* The following requires exclusive access to the GPIO registers */

	flags = irqsave();
#ifdef TIVA_SYSCON_RCGCGPIO
	rcgcgpio = getreg32(TIVA_SYSCON_RCGCGPIO);
	enabled = ((rcgcgpio & SYSCON_RCGCGPIO(port)) != 0);
#else
	rcgc2 = getreg32(TIVA_SYSCON_RCGC2);
	enabled = ((rcgc2 & SYSCON_RCGC2_GPIO(port)) != 0);
#endif

	lldbg("GPIO%c pinset: %08x base: %08x -- %s\n", tiva_gpioport(port), pinset, base, msg);
#ifdef TIVA_SYSCON_RCGCGPIO
	lldbg("RCGCGPIO: %08x (%s)\n", rcgcgpio, enabled ? "enabled" : "disabled");
#else
	lldbg("   RCGC2: %08x (%s)\n", rcgc2, enabled ? "enabled" : "disabled");
#endif

	/* Don't bother with the rest unless the port is enabled */

	if (enabled) {
		lldbg("   AFSEL: %02x DEN: %02x DIR: %02x DATA: %02x\n", getreg32(base + TIVA_GPIO_AFSEL_OFFSET), getreg32(base + TIVA_GPIO_DEN_OFFSET), getreg32(base + TIVA_GPIO_DIR_OFFSET), getreg32(base + TIVA_GPIO_DATA_OFFSET + 0x3fc));
		lldbg("      IS:  %02x IBE: %02x IEV: %02x IM:  %02x RIS: %08x MIS: %08x\n", getreg32(base + TIVA_GPIO_IEV_OFFSET), getreg32(base + TIVA_GPIO_IM_OFFSET), getreg32(base + TIVA_GPIO_RIS_OFFSET), getreg32(base + TIVA_GPIO_MIS_OFFSET));
		lldbg("     2MA:  %02x 4MA: %02x 8MA: %02x ODR: %02x PUR %02x PDR: %02x SLR: %02x\n", getreg32(base + TIVA_GPIO_DR2R_OFFSET), getreg32(base + TIVA_GPIO_DR4R_OFFSET), getreg32(base + TIVA_GPIO_DR8R_OFFSET), getreg32(base + TIVA_GPIO_ODR_OFFSET), getreg32(base + TIVA_GPIO_PUR_OFFSET), getreg32(base + TIVA_GPIO_PDR_OFFSET), getreg32(base + TIVA_GPIO_SLR_OFFSET));
	}

	irqrestore(flags);
#endif							/* CONFIG_DEBUG */

	return OK;
}
