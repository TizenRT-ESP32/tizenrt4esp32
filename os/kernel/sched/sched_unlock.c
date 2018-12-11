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
/************************************************************************
 * kernel/sched/sched_unlock.c
 *
 *   Copyright (C) 2007, 2009, 2014 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <tinyara/config.h>

#include <tinyara/clock.h>
#include <tinyara/arch.h>

#include "sched/sched.h"

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Global Variables
 ************************************************************************/

/************************************************************************
 * Private Variables
 ************************************************************************/

/************************************************************************
 * Private Function Prototypes
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name:  sched_unlock
 *
 * Description:
 *   This function decrements the preemption lock count.  Typically this
 *   is paired with sched_lock() and concludes a critical section of
 *   code.  Preemption will not be unlocked until sched_unlock() has
 *   been called as many times as sched_lock().  When the lockcount is
 *   decremented to zero, any tasks that were eligible to preempt the
 *   current task will execute.
 *
 ************************************************************************/

int sched_unlock(void)
{
	FAR struct tcb_s *rtcb = this_task();

	/* Check for some special cases:  (1) rtcb may be NULL only during
	 * early boot-up phases, and (2) sched_unlock() should have no
	 * effect if called from the interrupt level.
	 */

	if (rtcb && !up_interrupt_context()) {
		/* Prevent context switches throughout the following */

		irqstate_t flags = irqsave();

		/* Decrement the preemption lock counter */

		if (rtcb->lockcount) {
			rtcb->lockcount--;
		}

		/* Check if the lock counter has decremented to zero.  If so,
		 * then pre-emption has been re-enabled.
		 */

		if (rtcb->lockcount <= 0) {
			rtcb->lockcount = 0;

			/* Release any ready-to-run tasks that have collected in
			 * g_pendingtasks.
			 */

			if (g_pendingtasks.head) {
				up_release_pending();
			}
#if CONFIG_RR_INTERVAL > 0
			/* If (1) the task that was running supported round-robin
			 * scheduling and (2) if its time slice has already expired, but
			 * (3) it could not slice out because pre-emption was disabled,
			 * then we need to swap the task out now and reassess the interval
			 * timer for the next time slice.
			 */

			if ((rtcb->flags & TCB_FLAG_ROUND_ROBIN) != 0 && rtcb->timeslice == 0) {
				/* Yes.. that is the situation.  But one more thing.  The call
				 * to up_release_pending() above may have actually replaced
				 * the task at the head of the read-to-run list.  In that case,
				 * we need only to reset the timeslice value back to the
				 * maximum.
				 */

				if (rtcb != this_task()) {
					rtcb->timeslice = MSEC2TICK(CONFIG_RR_INTERVAL);
				}
#ifdef CONFIG_SCHED_TICKLESS
				else {
					sched_timer_reassess();
				}
#endif
			}
#endif
		}

		irqrestore(flags);
	}

	return OK;
}
