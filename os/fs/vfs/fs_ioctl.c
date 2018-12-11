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
 * fs/vfs/fs_ioctl.c
 *
 *   Copyright (C) 2007-2010, 2012-2014 Gregory Nutt. All rights reserved.
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

#include <sys/ioctl.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>

#include <net/if.h>

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
#include <tinyara/net/net.h>
#endif

#include "inode/inode.h"

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ioctl/fs_ioctl
 *
 * Description:
 *   Perform device specific operations.
 *
 * Parameters:
 *   fd       File/socket descriptor of device
 *   req      The ioctl command
 *   arg      The argument of the ioctl cmd
 *
 * Return:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   -1 on failure with errno set properly:
 *
 *   EBADF
 *     'fd' is not a valid descriptor.
 *   EFAULT
 *     'arg' references an inaccessible memory area.
 *   EINVAL
 *     'cmd' or 'arg' is not valid.
 *   ENOTTY
 *     'fd' is not associated with a character special device.
 *   ENOTTY
 *      The specified request does not apply to the kind of object that the
 *      descriptor 'fd' references.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_IOCTL_VARIADIC
int fs_ioctl(int fd, int req, unsigned long arg)
#else
int ioctl(int fd, int req, unsigned long arg)
#endif
{
	int err;
#if CONFIG_NFILE_DESCRIPTORS > 0
	FAR struct file *filep;
	FAR struct inode *inode;
	int ret = OK;

	/* Did we get a valid file descriptor? */

	if ((unsigned int)fd >= CONFIG_NFILE_DESCRIPTORS)
#endif
	{
		/* Perform the socket ioctl */

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
		if ((unsigned int)fd < (CONFIG_NFILE_DESCRIPTORS + CONFIG_NSOCKET_DESCRIPTORS)) {
			return netdev_ioctl(fd, req, arg);
		} else
#endif
		{
			err = EBADF;
			goto errout;
		}
	}
#if CONFIG_NFILE_DESCRIPTORS > 0
	/* Get the file structure corresponding to the file descriptor. */

	filep = fs_getfilep(fd);
	if (!filep) {
		/* The errno value has already been set */

		return ERROR;
	}

	/* Is a driver registered? Does it support the ioctl method? */

	inode = filep->f_inode;
	if (inode && inode->u.i_ops && inode->u.i_ops->ioctl) {
		/* Yes, then let it perform the ioctl */

		ret = (int)inode->u.i_ops->ioctl(filep, req, arg);
		if (ret < 0) {
			err = -ret;
			goto errout;
		}
	} else {
		err = ENOTTY;
		goto errout;
	}

	return ret;
#endif

errout:
	set_errno(err);
	return ERROR;
}
