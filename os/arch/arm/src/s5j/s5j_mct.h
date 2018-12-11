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
 * arch/arm/src/s5j/s5j_mct.h
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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
#ifndef __ARCH_ARM_SRC_S5J_S5J_MCT_H
#define __ARCH_ARM_SRC_S5J_S5J_MCT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>

#include <stdbool.h>

#include <tinyara/irq.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Helpers ******************************************************************/
#ifdef CONFIG_S5J_MCT
#  define CONFIG_S5J_MCT_NUM    4
#else
#  define CONFIG_S5J_MCT_NUM    0
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/
#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

enum s5j_mct_channel_e {
	S5J_MCT_CHANNEL0,
	S5J_MCT_CHANNEL1,
	S5J_MCT_CHANNEL2,
	S5J_MCT_CHANNEL3,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
struct s5j_mct_priv_s;

void s5j_mct_ack_irq(FAR struct s5j_mct_priv_s *priv);
void s5j_mct_setmode(FAR struct s5j_mct_priv_s *priv, bool oneshot);
void s5j_mct_enable(FAR struct s5j_mct_priv_s *priv);
void s5j_mct_disable(FAR struct s5j_mct_priv_s *priv);
int s5j_mct_setisr(FAR struct s5j_mct_priv_s *priv, xcpt_t handler, void *arg);
void s5j_mct_setperiod(FAR struct s5j_mct_priv_s *priv, uint32_t period);
void s5j_mct_enableint(FAR struct s5j_mct_priv_s *priv);
void s5j_mct_disableint(FAR struct s5j_mct_priv_s *priv);

/* Power-up timer and get its structure */
FAR struct s5j_mct_priv_s *s5j_mct_init(int timer);

/****************************************************************************
 * Name: s5j_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the timer device. This should be of the form
 *             /dev/timer0
 *   timer - the timer number.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/
#ifdef CONFIG_TIMER
int s5j_timer_initialize(FAR const char *devpath, int timer);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_S5J_S5J_MCT_H */
