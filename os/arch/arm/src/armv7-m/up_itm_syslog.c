/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
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
 * arch/arm/src/armv7-m/up_itm_syslog.c
 *
 *   Copyright (C) 2014 Pierre-noel Bouteville . All rights reserved.
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Pierre-noel Bouteville <pnb990@gmail.com>
 *            Gregory Nutt <gnutt@nuttx.org>
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

#include <stdio.h>

#include <tinyara/syslog/syslog.h>

#include "nvic.h"
#include "itm.h"
#include "tpi.h"
#include "dwt.h"
#include "up_arch.h"
#include "itm_syslog.h"

#if defined(CONFIG_SYSLOG) || defined(CONFIG_ARMV7M_ITMSYSLOG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ARMV7M_ITMSYSLOG_SWODIV
#define CONFIG_ARMV7M_ITMSYSLOG_SWODIV 15
#endif

#if CONFIG_ARMV7M_ITMSYSLOG_SWODIV < 0
#error CONFIG_ARMV7M_ITMSYSLOG_SWODIV should be at least equal to 1
#endif

/* Use Port #0 at default */

#ifndef CONFIG_ARMV7M_ITMSYSLOG_PORT
#define CONFIG_ARMV7M_ITMSYSLOG_PORT 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: itm_syslog_initialize
 *
 * Description:
 *   Performs ARM-specific initialize for the ITM SYSLOG functions.
 *   Additional, board specific logic may be required to:
 *
 *   - Enable/configured serial wire output pins
 *   - Enable debug clocking.
 *
 *   Those operations must be performed by MCU-specific logic before this
 *   function is called.
 *
 ****************************************************************************/

void itm_syslog_initialize(void)
{
	uint32_t regval;

	/* Enable trace in core debug */

	regval = getreg32(NVIC_DEMCR);
	regval |= NVIC_DEMCR_TRCENA;
	putreg32(regval, NVIC_DEMCR);

	putreg32(0xc5acce55, ITM_LAR);
	putreg32(0, ITM_TER);
	putreg32(0, ITM_TCR);
	putreg32(2, TPI_SPPR);		/* Pin protocol: 2=> Manchester (USART) */

	/* Default 880kbps */

	regval = CONFIG_ARMV7M_ITMSYSLOG_SWODIV - 1;
	putreg32(regval, TPI_ACPR);	/* TRACECLKIN/(ACPR+1) SWO speed */

	putreg32(0, ITM_TPR);
	putreg32(0x400003fe, DWT_CTRL);
	putreg32(0x0001000d, ITM_TCR);
	putreg32(0x00000100, TPI_FFCR);
	putreg32(0xffffffff, ITM_TER);	/* Enable 32 Ports */
}

/****************************************************************************
 * Name: syslog_putc
 *
 * Description:
 *   This is the low-level system logging interface.  The debugging/syslogging
 *   interfaces are syslog() and lowsyslog().  The difference is that
 *   the syslog() internface writes to fd=1 (stdout) whereas lowsyslog() uses
 *   a lower level interface that works from interrupt handlers.  This
 *   function is the low-level interface used to implement lowsyslog().
 *
 ****************************************************************************/

int syslog_putc(int ch)
{
	/* ITM enabled */

	if ((getreg32(ITM_TCR) & ITM_TCR_ITMENA_Msk) == 0) {
		return EOF;
	}

	/* ITM Port "CONFIG_ARMV7M_ITMSYSLOG_PORT" enabled */

	if (getreg32(ITM_TER) & (1 << CONFIG_ARMV7M_ITMSYSLOG_PORT)) {
		while (getreg32(ITM_PORT(CONFIG_ARMV7M_ITMSYSLOG_PORT)) == 0) ;
		putreg8((uint8_t)ch, ITM_PORT(CONFIG_ARMV7M_ITMSYSLOG_PORT));
	}

	return ch;
}

#endif							/* CONFIG_SYSLOG && CONFIG_ARMV7M_ITMSYSLOG */
