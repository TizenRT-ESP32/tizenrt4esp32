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
 * libc/stdio/lib_ftell.c
 *
 *   Copyright (C) 2008, 2011, 2013 Gregory Nutt. All rights reserved.
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
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "lib_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Constant Data
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_getrdoffset
 *
 * Description:
 *   It is insufficient to simply use the file offset; we must also account
 *   for the data offset in the any buffered data.  This function calculates
 *   that offset.
 *
 * Returned Value:
 *   The file position offset due to buffered data.
 *
 ****************************************************************************/

#if CONFIG_STDIO_BUFFER_SIZE > 0
static off_t lib_getrdoffset(FAR FILE *stream)
{
	off_t rdoffset = 0;
	lib_take_semaphore(stream);

	if (stream->fs_bufread != stream->fs_bufstart) {
#if CONFIG_NUNGET_CHARS > 0
		rdoffset = stream->fs_bufread - stream->fs_bufpos + stream->fs_nungotten;
#else
		rdoffset = stream->fs_bufread - stream->fs_bufpos;
#endif
	}

	lib_give_semaphore(stream);
	return rdoffset;
}
#else
#define lib_getrdoffset(stream) (0)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftell
 *
 * Description:
 *   ftell() returns the current value of the file position indicator for the
 *   stream pointed to by stream.
 *
 * Returned Value:
 *   Zero on success; -1 on failure with errno set appropriately.
 *
 ****************************************************************************/

long ftell(FAR FILE *stream)
{
	off_t position;

	/* Verify that we were provided with a stream */

	if (!stream) {
		set_errno(EBADF);
		return ERROR;
	}

	/* Perform the lseek to the current position.  This will not move the
	 * file pointer, but will return its current setting
	 */

	position = lseek(stream->fs_fd, 0, SEEK_CUR);
	if (position != (off_t)-1) {
		return (long)(position - lib_getrdoffset(stream));
	} else {
		return ERROR;
	}
}
