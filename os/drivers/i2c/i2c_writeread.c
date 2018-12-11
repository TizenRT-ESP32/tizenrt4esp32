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
 * drivers/i2c/i2c_writeread.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <assert.h>

#include <tinyara/i2c.h>

#if defined(CONFIG_I2C_TRANSFER)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_writeread
 *
 * Description:
 *   Send a block of data on I2C using the previously, followed by restarted
 *   read access.  This provides a convenient wrapper to the transfer function.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   config  - Described the I2C configuration
 *   wbuffer - A pointer to the read-only buffer of data to be written to device
 *   wbuflen - The number of bytes to send from the buffer
 *   rbuffer - A pointer to a buffer of data to receive the data from the device
 *   rbuflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int i2c_writeread(FAR struct i2c_dev_s *dev, FAR const struct i2c_config_s *config, FAR const uint8_t *wbuffer, int wbuflen, FAR uint8_t *rbuffer, int rbuflen)
{
	struct i2c_msg_s msg[2];
	unsigned int flags;
	int ret = -1;

	/* 7- or 10-bit address? */

	DEBUGASSERT(config->addrlen == 10 || config->addrlen == 7);
	flags = (config->addrlen == 10) ? I2C_M_TEN : 0;

	/* Format two messages: The first is a write */

	msg[0].addr = config->address;
	msg[0].flags = flags;
	msg[0].buffer = (FAR uint8_t *)wbuffer;	/* Override const */
	msg[0].length = wbuflen;

	/* The second is either a read (rbuflen > 0) or a write (rbuflen < 0) with
	 * no restart.
	 */

	if (rbuflen > 0) {
		msg[1].flags = (flags | I2C_M_READ);
	} else {
		msg[1].flags = (flags | I2C_M_NORESTART);
		rbuflen = -rbuflen;
	}

	msg[1].addr = config->address;
	msg[1].buffer = rbuffer;
	msg[1].length = rbuflen;

	/* Then perform the transfer
	 *
	 * REVISIT:  The following two operations must become atomic in order to
	 * assure thread safety.
	 */

	if (dev != 0x0) {
		I2C_SETFREQUENCY(dev, config->frequency);
		ret = I2C_TRANSFER(dev, msg, 2);
	}

	return ret;
}

#endif							/* CONFIG_I2C_TRANSFER */
