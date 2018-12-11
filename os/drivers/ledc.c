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
 * drivers/ledc.c
 *
 *   Copyright (C) 2011-2013, 2016 Gregory Nutt. All rights reserved.
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
 * Compilation Switches
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/arch.h>
#include <tinyara/kmalloc.h>
#include <tinyara/semaphore.h>
#include <tinyara/fs/fs.h>
#include <tinyara/ledc.h>

#include <arch/irq.h>

#ifdef CONFIG_LEDC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing LEDC */

#define LEDC_PREVENT_MULTIPLE_OPEN   0

#ifdef CONFIG_DEBUG_LEDC_INFO
#define ledcdbg    dbg
#define ledcvdbg   vdbg
#define ledclldbg  lldbg
#define ledcllvdbg llvdbg
#else
#define ledcdbg(...)
#define ledcvdbg(...)
#define ledclldbg(...)
#define ledcllvdbg(...)
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct ledc_upperhalf_s {
	uint8_t crefs;				/* The number of times the device has been opened */
	volatile bool started;		/* True: pulsed output is being generated */
	sem_t exclsem;				/* Supports mutual exclusion */
	struct ledc_info_s info;		/* Pulsed output characteristics */
	FAR struct ledc_lowerhalf_s *dev;	/* lower-half state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ledc_open(FAR struct file *filep);
static int ledc_close(FAR struct file *filep);
static ssize_t ledc_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t ledc_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int ledc_start(FAR struct ledc_upperhalf_s *upper, unsigned int oflags);
static int ledc_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ledcops = {
	ledc_open,					/* open */
	ledc_close,					/* close */
	ledc_read,					/* read */
	ledc_write,					/* write */
	0,							/* seek */
	ledc_ioctl					/* ioctl */
#ifndef CONFIG_DISABLE_POLL
	, 0						/* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ledc_open
 *
 * Description:
 *   This function is called whenever the LEDC device is opened.
 *
 ****************************************************************************/

static int ledc_open(FAR struct file *filep)
{
	FAR struct inode *inode = filep->f_inode;
	FAR struct ledc_upperhalf_s *upper = inode->i_private;
	uint8_t tmp;
	int ret;

	ledcvdbg("crefs: %d\n", upper->crefs);

	/* Get exclusive access to the device structures */

	ret = sem_wait(&upper->exclsem);
	if (ret < 0) {
		ret = -get_errno();
		goto errout;
	}

	/* Increment the count of references to the device.  If this the first
	 * time that the driver has been opened for this device, then initialize
	 * the device.
	 */

	tmp = upper->crefs + 1;
	if (tmp == 0) {
		/* More than 255 opens; uint8_t overflows to zero */

		ret = -EMFILE;
		goto errout_with_sem;
	}

	/* Check if this is the first time that the driver has been opened. */

	if (tmp == 1) {
		FAR struct ledc_lowerhalf_s *lower = upper->dev;

		/* Yes.. perform one time hardware initialization. */

		DEBUGASSERT(lower->ops->setup != NULL);
		ledcvdbg("calling setup\n");

		ret = lower->ops->setup(lower);
		if (ret < 0) {
			goto errout_with_sem;
		}
	}
#if LEDC_PREVENT_MULTIPLE_OPEN == 1
	else if (tmp > 1) {
		ret = -EBUSY;
		goto errout_with_sem;
	}
#endif

	/* Save the new open count on success */

	upper->crefs = tmp;
	ret = OK;

errout_with_sem:
	sem_post(&upper->exclsem);

errout:
	return ret;
}

/****************************************************************************
 * Name: ledc_close
 *
 * Description:
 *   This function is called when the LEDC device is closed.
 *
 ****************************************************************************/

static int ledc_close(FAR struct file *filep)
{
	FAR struct inode *inode = filep->f_inode;
	FAR struct ledc_upperhalf_s *upper = inode->i_private;
	int ret;

	ledcvdbg("crefs: %d\n", upper->crefs);

	/* Get exclusive access to the device structures */

	ret = sem_wait(&upper->exclsem);
	if (ret < 0) {
		ret = -get_errno();
		goto errout;
	}

	/* Decrement the references to the driver.  If the reference count will
	 * decrement to 0, then uninitialize the driver.
	 */

	if (upper->crefs > 1) {
		upper->crefs--;
	} else {
		FAR struct ledc_lowerhalf_s *lower = upper->dev;

		/* There are no more references to the port */

		upper->crefs = 0;

		/* Disable the LEDC device */

		DEBUGASSERT(lower->ops->shutdown != NULL);
		ledcvdbg("calling shutdown: %d\n");

		lower->ops->shutdown(lower);
	}
	ret = OK;

//errout_with_sem:
	sem_post(&upper->exclsem);

errout:
	return ret;
}

/****************************************************************************
 * Name: ledc_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t ledc_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
	/* Return zero -- usually meaning end-of-file */

	return 0;
}

/****************************************************************************
 * Name: ledc_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t ledc_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
	return 0;
}

/****************************************************************************
 * Name: ledc_start
 *
 * Description:
 *   Handle the LEDCIOC_START ioctl command
 *
 ****************************************************************************/

static int ledc_start(FAR struct ledc_upperhalf_s *upper, unsigned int oflags)
{
	FAR struct ledc_lowerhalf_s *lower = upper->dev;
	int ret = OK;

	DEBUGASSERT(upper != NULL && lower->ops->start != NULL);

	/* Verify that the LEDC is not already running */

	if (!upper->started) {
		/* Invoke the bottom half method to start the pulse train */

		ret = lower->ops->start(lower, &upper->info);

		/* A return value of zero means that the pulse train was started
		 * successfully.
		 */

		if (ret == OK) {
			/* Indicate that the pulse train has started */

			upper->started = true;
		}
	}

	return ret;
}

/****************************************************************************
 * Name: ledc_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the LEDC work is done.
 *
 ****************************************************************************/

static int ledc_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
	FAR struct inode *inode = filep->f_inode;
	FAR struct ledc_upperhalf_s *upper = inode->i_private;
	FAR struct ledc_lowerhalf_s *lower = upper->dev;
	int ret;

	ledcvdbg("cmd: %d arg: %ld\n", cmd, arg);

	/* Get exclusive access to the device structures */

	ret = sem_wait(&upper->exclsem);
	if (ret < 0) {
		return ret;
	}

	/* Handle built-in ioctl commands */

	switch (cmd) {
	/* LEDCIOC_SETCHARACTERISTICS - Set the characteristics of the next pulsed
	 *   output.  This command will neither start nor stop the pulsed output.
	 *   It will either setup the configuration that will be used when the
	 *   output is started; or it will change the characteristics of the pulsed
	 *   output on the fly if the timer is already started.
	 *
	 *   ioctl argument:  A read-only reference to struct ledc_info_s that provides
	 *   the characteristics of the pulsed output.
	 */

	case LEDCIOC_SETCHARACTERISTICS: {
		FAR const struct ledc_info_s *info = (FAR const struct ledc_info_s *)((uintptr_t)arg);
		DEBUGASSERT(info != NULL && lower->ops->start != NULL);

		/* Save the pulse train characteristics */

		memcpy(&upper->info, info, sizeof(struct ledc_info_s));

		/* If LEDC is already running, then re-start it with the new characteristics */

		if (upper->started) {
			ret = lower->ops->start(lower, &upper->info);
		}
	}
	break;

	/* LEDCIOC_GETCHARACTERISTICS - Get the currently selected characteristics of
	 *   the pulsed output (independent of whether the output is start or stopped).
	 *
	 *   ioctl argument:  A reference to struct ledc_info_s to receive the
	 *   characteristics of the pulsed output.
	 */

	case LEDCIOC_GETCHARACTERISTICS: {
		FAR struct ledc_info_s *info = (FAR struct ledc_info_s *)((uintptr_t)arg);
		DEBUGASSERT(info != NULL);

		memcpy(info, &upper->info, sizeof(struct ledc_info_s));

	}
	break;

	/* LEDCIOC_START - Start the pulsed output.  The LEDCIOC_SETCHARACTERISTICS
	 *   command must have previously been sent.
	 *
	 *   ioctl argument:  None
	 */

	case LEDCIOC_START: {
    	DEBUGASSERT(lower->ops->start != NULL);

		/* Start the pulse train */

		ret = ledc_start(upper, filep->f_oflags);
	}
	break;

	/* LEDCIOC_STOP - Stop the pulsed output.
	 *
	 *   ioctl argument:  None
	 */

	case LEDCIOC_STOP: {
		ledcvdbg("LEDCIOC_STOP: started: %d\n", upper->started);
		DEBUGASSERT(lower->ops->stop != NULL);

		if (upper->started) {
			ret = lower->ops->stop(lower);
			upper->started = false;
		}
	}
	break;

	/* Any unrecognized IOCTL commands might be platform-specific ioctl commands */

	default: {
		ledcvdbg("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);
		DEBUGASSERT(lower->ops->ioctl != NULL);
		ret = lower->ops->ioctl(lower, cmd, arg);
	}
	break;
	}

	sem_post(&upper->exclsem);
	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ledc_register
 *
 * Description:
 *   This function binds an instance of a "lower half" timer driver with the
 *   "upper half" LEDC device and registers that device so that can be used
 *   by application code.
 *
 *   When this function is called, the "lower half" driver should be in the
 *   reset state (as if the shutdown() method had already been called).
 *
 * Input parameters:
 *   path - The full path to the driver to be registered in the TinyAra pseudo-
 *     filesystem.  The recommended convention is to name all LEDC drivers
 *     as "/dev/ledc0", "/dev/ledc1", etc.  where the driver path differs only
 *     in the "minor" number at the end of the device name.
 *   dev - A pointer to an instance of lower half timer driver.  This instance
 *     is bound to the LEDC driver and must persists as long as the driver
 *     persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ledc_register(FAR const char *path, FAR struct ledc_lowerhalf_s *dev)
{
	FAR struct ledc_upperhalf_s *upper;

	/* Allocate the upper-half data structure */

	upper = (FAR struct ledc_upperhalf_s *)kmm_zalloc(sizeof(struct ledc_upperhalf_s));
	if (!upper) {
		ledcdbg("Allocation failed\n");
		return -ENOMEM;
	}

	/* Initialize the LEDC device structure (it was already zeroed by kmm_zalloc()) */

	sem_init(&upper->exclsem, 0, 1);
	upper->dev = dev;

	/* Register the LEDC device */

	return register_driver(path, &g_ledcops, 0666, upper);
}

/****************************************************************************
 * Name: ledc_expired
 *
 * Description:
 *   If CONFIG_LEDC_PULSECOUNT is defined and the pulse count was configured
 *   to a non-zero value, then the "upper half" driver will wait for the
 *   pulse count to expire.  The sequence of expected events is as follows:
 *
 *   1. The upper half driver calls the start method, providing the lower
 *      half driver with the pulse train characteristics.  If a fixed
 *      number of pulses is required, the 'count' value will be nonzero.
 *   2. The lower half driver's start() methoc must verify that it can
 *      support the request pulse train (frequency, duty, AND pulse count).
 *      If it cannot, it should return an error.  If the pulse count is
 *      non-zero, it should set up the hardware for that number of pulses
 *      and return success.  NOTE:  That is CONFIG_LEDC_PULSECOUNT is
 *      defined, the start() method receives an additional parameter
 *      that must be used in this callback.
 *   3. When the start() method returns success, the upper half driver
 *      will "sleep" until the ledc_expired method is called.
 *   4. When the lower half detects that the pulse count has expired
 *      (probably through an interrupt), it must call the ledc_expired
 *      interface using the handle that was previously passed to the
 *      start() method
 *
 * Input parameters:
 *   handle - This is the handle that was provided to the lower-half
 *     start() method.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/


#endif /* CONFIG_LEDC */
