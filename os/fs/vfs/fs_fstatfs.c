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
 * fs/vfs/fs_fstatfs.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <sys/statfs.h>
#include <string.h>
#include <limits.h>
#include <sched.h>
#include <errno.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fstatfs
 *
 * Return: Zero on success; -1 on failure with errno set:
 *
 *   EACCES  Search permission is denied for one of the directories in the
 *           path prefix of path.
 *   EFAULT  Bad address.
 *   ENOENT  A component of the path path does not exist, or the path is an
 *           empty string.
 *   ENOMEM  Out of memory
 *   ENOTDIR A component of the path is not a directory.
 *   ENOSYS  The file system does not support this call.
 *
 ****************************************************************************/

int fstatfs(int fd, FAR struct statfs *buf)
{
	FAR struct file *filep;
	FAR struct inode *inode;
	int ret;

	DEBUGASSERT(buf != NULL);

	/* First, get the file structure.
     * Note that on failure, fs_getfilep() will set the errno variable.
	 */
	filep = fs_getfilep(fd);
	if (filep == NULL) {
		/* The errno value has already been set */
		return ERROR;
	}

	/* Get the inode from the file structure */
	inode = filep->f_inode;
	DEBUGASSERT(inode != NULL);

	/* Check if the file is open */
	if (inode == NULL) {
		/* The descriptor does not refer to an open file. */
		ret = -EBADF;
	} else
#ifndef CONFIG_DISABLE_MOUNTPOINT
	/* The way we handle the stat depends on the type of inode that we
	 * are dealing with. */
	if (INODE_IS_MOUNTPT(inode)) {
		/* The node is a file system mointpoint. Verify that the mountpoint
		 * supports the statfs() method */
		ret = -ENOSYS;
		if (inode->u.i_mops && inode->u.i_mops->statfs) {
			/* Perform the statfs() operation */
			ret = inode->u.i_mops->statfs(inode, buf);
		}
	} else
#endif
	{
		/* The node is part of the root pseudo file system */
		memset(buf, 0, sizeof(struct statfs));
		buf->f_type = PROC_SUPER_MAGIC;
		buf->f_namelen = NAME_MAX;
		ret = OK;
	}

	/* Check if the fstat operation was successful */
	if (ret < 0) {
		set_errno(-ret);
		return ERROR;
	}

	/* Successfully statfs'ed the file */
	return OK;
}
