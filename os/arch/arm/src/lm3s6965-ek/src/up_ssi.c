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
/************************************************************************************
 * configs/lm3s6965-ek/src/up_ssi.c
 * arch/arm/src/board/up_ssi.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <tinyara/spi/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "tiva_gpio.h"
#include "lm3s6965ek_internal.h"

/* The LM3S6965 Eval Kit microSD CS is on SSI0 */

#if defined(CONFIG_TIVA_SSI0)	/* || defined(CONFIG_TIVA_SSI1) */

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#undef SSI_DEBUG				/* Define to enable debug */
#undef SSI_VERBOSE				/* Define to enable verbose debug */

#ifdef SSI_DEBUG
#define ssidbg  lldbg
#ifdef SSI_VERBOSE
#define ssivdbg lldbg
#else
#define ssivdbg(...)
#endif
#else
#undef SSI_VERBOSE
#define ssidbg(...)
#define ssivdbg(...)
#endif

/* Dump GPIO registers */

#ifdef SSI_VERBOSE
#define ssi_dumpgpio(m) tiva_dumpgpio(SDCCS_GPIO, m)
#else
#define ssi_dumpgpio(m)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: tiva_ssiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LM3S6965 Eval Kit.
 *
 ************************************************************************************/

void weak_function lm_ssiinitialize(void)
{
	/* Configure the SPI-based microSD CS GPIO */

	ssi_dumpgpio("lm_ssiinitialize() Entry)");
	tiva_configgpio(SDCCS_GPIO);
#ifdef CONFIG_NX_LCDDRIVER
	tiva_configgpio(OLEDCS_GPIO);
#endif
	ssi_dumpgpio("lm_ssiinitialize() Exit");
}

/****************************************************************************
 * The external functions, tiva_spiselect and tiva_spistatus must be provided
 * by board-specific logic.  The are implementations of the select and status
 * methods SPI interface defined by struct spi_ops_s (see include/tinyara/spi/spi.h).
 * All othermethods (including up_spiinitialize()) are provided by common
 * logic.  To use this common SPI logic on your board:
 *
 *   1. Provide tiva_spiselect() and tiva_spistatus() functions in your
 *      board-specific logic.  This function will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   2. Add a call to up_spiinitialize() in your low level initialization
 *      logic
 *   3. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

void tiva_spiselect(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	ssidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
	ssi_dumpgpio("tiva_spiselect() Entry");

	if (devid == SPIDEV_MMCSD) {
		/* Assert the CS pin to the card */

		tiva_gpiowrite(SDCCS_GPIO, !selected);
	}
#ifdef CONFIG_NX_LCDDRIVER
	else if (devid == SPIDEV_DISPLAY) {
		/* Assert the CS pin to the display */

		tiva_gpiowrite(OLEDCS_GPIO, !selected);
	}
#endif
	ssi_dumpgpio("tiva_spiselect() Exit");
}

uint8_t tiva_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	ssidbg("Returning SPI_STATUS_PRESENT\n");
	return SPI_STATUS_PRESENT;
}

#endif							/* CONFIG_TIVA_SSI0 || CONFIG_TIVA_SSI1 */
