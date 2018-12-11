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
 * kernel/semaphore/sem_destroy.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <semaphore.h>
#include <errno.h>

#include "semaphore/semaphore.h"

#ifdef CONFIG_SEMAPHORE_HISTORY
#include "sched/sched.h"
#include <tinyara/debug/sysdbg.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

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

/****************************************************************************
 * Name: sem_destroy
 *
 * Description:
 *   This function is used to destroy the un-named semaphore indicated by
 *   'sem'.  Only a semaphore that was created using sem_init() may be
 *   destroyed using sem_destroy(); the effect of calling sem_destroy() with
 *   a named semaphore is undefined.  The effect of subsequent use of the
 *   semaphore sem is undefined until sem is re-initialized by another call
 *   to sem_init().
 *
 *   The effect of destroying a semaphore upon which other processes are
 *   currently blocked is undefined.
 *
 * Parameters:
 *   sem - Semaphore to be destroyed.
 *
 * Return Value:
 *   0 (OK), or -1 (ERROR) if unsuccessful.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sem_destroy(FAR sem_t *sem)
{
	/* Assure a valid semaphore is specified */

	if (sem) {
		/* Already destroyed? */

		if ((sem->flags & FLAGS_INITIALIZED) == 0) {
			return OK;
		}

		/* There is really no particular action that we need
		 * take to destroy a semaphore.  We will just reset
		 * the count to some reasonable value (1) and release
		 * ownership.
		 *
		 * Check if other threads are waiting on the semaphore.
		 * In this case, the behavior is undefined.  We will:
		 * leave the count unchanged but still return OK.
		 */

		if (sem->semcount >= 0) {
			sem->semcount = 1;
		}
#ifdef CONFIG_SEMAPHORE_HISTORY
		save_semaphore_history(sem, (void *)this_task(), SEM_DESTROY);
#endif
		/* Release holders of the semaphore */

		sem_destroyholder(sem);

		sem->flags &= ~FLAGS_INITIALIZED;
		return OK;
	} else {
		set_errno(EINVAL);
		return ERROR;
	}
}
