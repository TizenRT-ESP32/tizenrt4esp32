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
 * arch/arm/irq/up_ramvec_attach.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <errno.h>
#include <debug.h>

#include <tinyara/irq.h>
#include <tinyara/arch.h>

#include "ram_vectors.h"

#ifdef CONFIG_ARCH_RAMVECTORS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing the interrupt
 * config.  NOTE: that only lldbg types are used so that the output is
 * immediately available.
 */

#ifdef CONFIG_DEBUG_IRQ
#define intdbg    lldbg
#define intvdbg   llvdbg
#else
#define intdbg(...)
#define intvdbg(...)
#endif

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Common exception entrypoint */

void exception_common(void);

/****************************************************************************
 * Name: up_ramvec_attach
 *
 * Description:
 *   Configure the ram vector table so that IRQ number 'irq' will be
 *   dispatched by hardware to 'vector'
 *
 ****************************************************************************/

int up_ramvec_attach(int irq, up_vector_t vector)
{
	int ret = -EINVAL;

	intvdbg("%s IRQ%d\n", vector ? "Attaching" : "Detaching", irq);

	if ((unsigned)irq < NR_VECTORS) {
		irqstate_t flags;

		/* If the new vector is NULL, then the vector is being detached. In
		 * this case, disable the itnerrupt and direct any interrupts to the
		 * common exception handler.
		 */

		flags = irqsave();
		if (vector == NULL) {
			/* Disable the interrupt if we can before detaching it.  We might
			 * not be able to do this for all interrupts.
			 */

			up_disable_irq(irq);

			/* Detaching the vector really means re-attaching it to the
			 * common exception handler.
			 */

			vector = exception_common;
		}

		/* Save the new vector in the vector table */

		g_ram_vectors[irq] = vector;
		irqrestore(flags);
		ret = OK;
	}

	return ret;
}

#endif							/* !CONFIG_ARCH_RAMVECTORS */
