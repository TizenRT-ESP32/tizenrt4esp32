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
 * include/tinyara/i2c.h
 *
 *   Copyright(C) 2009-2012, 2016 Gregory Nutt. All rights reserved.
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_TINYARA_I2C_H
#define __INCLUDE_TINYARA_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* If a dynamic timeout is selected, then a non-negative, non-zero micro-
 * seconds per byte value must be provided as well.
 */

#ifdef CONFIG_STM32_I2C_DYNTIMEO
#if CONFIG_STM32_I2C_DYNTIMEO_USECPERBYTE < 1
#warning "Ignoring CONFIG_STM32_I2C_DYNTIMEO because of CONFIG_STM32_I2C_DYNTIMEO_USECPERBYTE"
#undef CONFIG_STM32_I2C_DYNTIMEO
#endif
#endif

/* I2C address calculation.  Convert 7- and 10-bit address to 8-bit and
 * 16-bit read/write address
 */

#define I2C_READBIT          0x01

/* Convert 7- to 8-bit address */

#define I2C_ADDR8(a)         ((a) << 1)
#define I2C_WRITEADDR8(a)    I2C_ADDR8(a)
#define I2C_READADDR8(a)     (I2C_ADDR8(a) | I2C_READBIT)

/* Convert 10- to 16-bit address */

#define I2C_ADDR10H(a)       (0xf0 | (((a) >> 7) & 0x06))
#define I2C_ADDR10L(a)       ((a) & 0xff)

#define I2C_WRITEADDR10H(a)  I2C_ADDR10H(a)
#define I2C_WRITEADDR10L(a)  I2C_ADDR10L(a)

#define I2C_READADDR10H(a)   (I2C_ADDR10H(a) | I2C_READBIT)
#define I2C_READADDR10L(a)   I2C_ADDR10L(a)

/* Bit definitions for the flags field in struct i2c_msg_s */
/*flags field*/
#define I2C_M_READ           0x0001	/* Read data, from slave to master */
#define I2C_M_TEN            0x0002	/* Ten bit address */
#define I2C_M_NORESTART      0x0080	/* Message should not begin with
									 * (re-)start of transfer */
#define I2C_M_IGNORE_NAK     0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NOSTART        0x4000	/* if I2C_FUNC_NOSTART */

/*io_ctrl command type*/
#define I2C_SLAVE            0x0703	/* Use this slave address */
#define I2C_SLAVE_FORCE      0x0706	/* Use this slave address, even if it
									 * is already in use by a driver! */
#define I2C_TENBIT           0x0704	/* 0 for 7 bit addrs, != 0 for 10 bit */
#define I2C_RDWR             0x0707	/* Combined R/W transfer (one STOP only) */

#define I2C_FREQUENCY        0X0801



/* Access macros ************************************************************/

/****************************************************************************
 * Name: I2C_SETFREQUENCY
 *
 * Description:
 *   Set the I2C frequency. This frequency will be retained in the struct
 *   i2c_dev_s instance and will be used with all transfers.  Required.
 *
 * Input Parameters:
 *   dev       - Device-specific state data
 *   frequency - The I2C frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

#define I2C_SETFREQUENCY(d, f) ((d)->ops->setfrequency(d, f))

/****************************************************************************
 * Name: I2C_SETADDRESS
 *
 * Description:
 *   Set the I2C slave address. This frequency will be retained in the struct
 *   i2c_dev_s instance and will be used with all transfers.  Required.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   address - The I2C slave address
 *   nbits   - The number of address bits provided (7 or 10)
 *
 * Returned Value:
 *   Returns OK on success; a negated errno on failure.
 *
 ****************************************************************************/

#define I2C_SETADDRESS(d, a, n) ((d)->ops->setaddress(d, a, n))

/****************************************************************************
 * Name: I2C_SETOWNADDRESS
 *
 * Description:
 *   Set our own I2C address. Calling this function enables Slave mode and
 *   disables Master mode on given instance (note that I2C is a bus, where
 *   multiple masters and slave may be handled by one device driver).
 *
 *   One may register a callback to be notified about reception. During the
 *   slave mode reception, the function READ and WRITE must be used to
 *   to handle reads and writes from a master.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   address - Our own slave address; If it is 0x00, then the device driver
 *             listens to general call
 *   nbits   - The number of address bits provided (7 or 10)
 *
 * Returned Value:
 *   OK on valid address and if the same address has not been assigned
 *   to another instance sharing the same port. Otherwise ERROR is returned.
 *
 ****************************************************************************/

#define I2C_SETOWNADDRESS(d, a, n)  ((d)->ops->setownaddress(d, a, n))

/****************************************************************************
 * Name: I2C_WRITE
 *
 * Description:
 *   Send a block of data on I2C using the previously selected I2C
 *   frequency and slave address. Each write operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this write completes. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to the read-only buffer of data to be written to device
 *   buflen - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define I2C_WRITE(d, b, l) ((d)->ops->write(d, b, l))

/****************************************************************************
 * Name: I2C_READ
 *
 * Description:
 *   Receive a block of data from I2C using the previously selected I2C
 *   frequency and slave address. Each read operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this read completes. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to a buffer of data to receive the data from the device
 *   buflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define I2C_READ(d, b, l) ((d)->ops->read(d, b, l))

/****************************************************************************
 * Name: I2C_WRITEREAD
 *
 * Description:
 *   Send a block of data on I2C using the previously selected I2C
 *   frequency and slave address, followed by restarted read access.
 *   It provides a convenient wrapper to the transfer function.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   wbuffer - A pointer to the read-only buffer of data to be written to device
 *   wbuflen - The number of bytes to send from the buffer
 *   rbuffer - A pointer to a buffer of data to receive the data from the device
 *   rbuflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define I2C_WRITEREAD(d, wb, wl, rb, rl) ((d)->ops->writeread(d, wb, wl, rb, rl))

/****************************************************************************
 * Name: I2C_TRANSFER
 *
 * Description:
 *   Perform a sequence of I2C transfers, each transfer is started with a
 *   START and the final transfer is completed with a STOP. Each sequence
 *   will be an 'atomic'  operation in the sense that any other I2C actions
 *   will be serialized and pend until this read completes. Optional.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   msgs     - A pointer to a set of message descriptors
 *   msgcount - The number of transfers to perform
 *
 * Returned Value:
 *   The number of transfers completed
 *
 ****************************************************************************/

#define I2C_TRANSFER(d, m, c) ((d)->ops->transfer(d, m, c))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The I2C vtable */

struct i2c_dev_s;
struct i2c_msg_s;
struct i2c_ops_s {
	uint32_t (*setfrequency)(FAR struct i2c_dev_s *dev, uint32_t frequency);
	int (*setaddress)(FAR struct i2c_dev_s *dev, int addr, int nbits);
	int (*write)(FAR struct i2c_dev_s *dev, const uint8_t *buffer, int buflen);
	int (*read)(FAR struct i2c_dev_s *dev, uint8_t *buffer, int buflen);
#ifdef CONFIG_I2C_WRITEREAD
	int (*writeread)(FAR struct i2c_dev_s *inst, const uint8_t *wbuffer, int wbuflen, uint8_t *rbuffer, int rbuflen);
#endif

#ifdef CONFIG_I2C_TRANSFER
	int (*transfer)(FAR struct i2c_dev_s *dev, FAR struct i2c_msg_s *msgs, int count);
#endif
#ifdef CONFIG_I2C_SLAVE
	int (*setownaddress)(FAR struct i2c_dev_s *dev, int addr, int nbits);

	int (*registercallback)(FAR struct i2c_dev_s *dev, int (*callback)(void));
#endif
};

/* This structure contains the full state of I2C as needed for a specific
 * transfer.  It is passed to I2C methods so that I2C transfer may be
 * performed in a thread safe manner.
 */

struct i2c_config_s {
	uint32_t frequency;			/* I2C frequency */
	uint16_t address;			/* I2C address (7 or 10 bits) */
	uint8_t addrlen;			/* I2C address length (7 or 10 bits) */
};

/* I2C transaction segment beginning with a START.  A number of these can
 * be transferred together to form an arbitrary sequence of write/read transfer
 * to an I2C slave device.
 */

struct i2c_msg_s {
	uint16_t addr;				/* Slave address */
	uint16_t flags;				/* See I2C_M_* definitions */
#ifdef CONFIG_I2C_USERIO
	uint16_t length;
	uint8_t *buffer;
#else
	uint8_t *buffer;
	int length;
#endif
};

#ifdef CONFIG_I2C_USERIO
/* I2C transaction data. Used in RDWR ioctl */
struct i2c_rdwr_ioctl_data_s {
	struct i2c_msg_s *msgs;
	uint32_t nmsgs;
};
#endif

/* I2C private data.  This structure only defines the initial fields of the
 * structure visible to the I2C client.  The specific implementation may
 * add additional, device specific fields after the vtable.
 */

struct i2c_dev_s {
	const struct i2c_ops_s *ops;	/* I2C vtable */
	FAR void *priv;				/* Used by the arch-specific logic */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: up_i2cinitialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a unique instance of struct
 *   struct i2c_dev_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Input Parameter:
 *   Port number (for hardware that has multiple I2C interfaces)
 *
 * Returned Value:
 *   Valid I2C device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

EXTERN FAR struct i2c_dev_s *up_i2cinitialize(int port);

/****************************************************************************
 * Name: up_i2cuninitialize
 *
 * Description:
 *   De-initialize the selected I2C port, and power down the device.
 *
 * Input Parameter:
 *   Device structure as returned by the up_i2cinitialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/
EXTERN int up_i2cuninitialize(FAR struct i2c_dev_s *dev);

/************************************************************************************
 * Name: up_i2creset
 *
 * Description:
 *   Reset an I2C bus
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_RESET
EXTERN int up_i2creset(FAR struct i2c_dev_s *dev);
#endif

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

#ifdef CONFIG_I2C_TRANSFER
int i2c_writeread(FAR struct i2c_dev_s *dev, FAR const struct i2c_config_s *config, FAR const uint8_t *wbuffer, int wbuflen, FAR uint8_t *rbuffer, int rbuflen);
#endif

/****************************************************************************
 * Name: i2c_write
 *
 * Description:
 *   Send a block of data on I2C. Each write operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this write completes.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   config  - Described the I2C configuration
 *   buffer - A pointer to the read-only buffer of data to be written to device
 *   buflen - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRANSFER
int i2c_write(FAR struct i2c_dev_s *dev, FAR const struct i2c_config_s *config, FAR const uint8_t *buffer, int buflen);
#endif

/****************************************************************************
 * Name: i2c_read
 *
 * Description:
 *   Receive a block of data from I2C using the previously selected I2C
 *   frequency and slave address. Each read operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this read completes. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to a buffer of data to receive the data from the device
 *   buflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRANSFER
int i2c_read(FAR struct i2c_dev_s *dev, FAR const struct i2c_config_s *config, FAR uint8_t *buffer, int buflen);
#endif

#ifdef CONFIG_I2C_USERIO
int i2c_uioregister(FAR const char *path, FAR struct i2c_dev_s *dev);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif							/* __INCLUDE_TINYARA_I2C_H */
