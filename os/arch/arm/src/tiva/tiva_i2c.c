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
/************************************************************************************
 * arch/arm/src/tiva/tiva_i2c.c
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The basic structure of this driver derives in spirit (if nothing more) from the
 * NuttX STM32 I2C driver which has:
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <tinyara/arch.h>
#include <tinyara/irq.h>
#include <tinyara/i2c.h>
#include <tinyara/kmalloc.h>
#include <tinyara/clock.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "tiva_enablepwr.h"
#include "tiva_enableclks.h"
#include "tiva_gpio.h"
#include "chip/tiva_pinmap.h"
#include "chip/tiva_syscontrol.h"
#include "tiva_i2c.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_TIVA_I2C0) || defined(CONFIG_TIVA_I2C1) || \
	defined(CONFIG_TIVA_I2C2) || defined(CONFIG_TIVA_I2C3) || \
	defined(CONFIG_TIVA_I2C4) || defined(CONFIG_TIVA_I2C5) || \
	defined(CONFIG_TIVA_I2C6) || defined(CONFIG_TIVA_I2C7) || \
	defined(CONFIG_TIVA_I2C8) || defined(CONFIG_TIVA_I2C9)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.  Instead,
 * CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_TIVA_I2C_TIMEOSEC) && !defined(CONFIG_TIVA_I2C_TIMEOMS)
#define CONFIG_TIVA_I2C_TIMEOSEC 0
#define CONFIG_TIVA_I2C_TIMEOMS  500	/* Default is 500 milliseconds */
#elif !defined(CONFIG_TIVA_I2C_TIMEOSEC)
#define CONFIG_TIVA_I2C_TIMEOSEC 0	/* User provided milliseconds */
#elif !defined(CONFIG_TIVA_I2C_TIMEOMS)
#define CONFIG_TIVA_I2C_TIMEOMS  0	/* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_TIVA_I2C_TIMEOTICKS
#define CONFIG_TIVA_I2C_TIMEOTICKS \
	(SEC2TICK(CONFIG_TIVA_I2C_TIMEOSEC) + MSEC2TICK(CONFIG_TIVA_I2C_TIMEOMS))
#endif

#ifndef CONFIG_TIVA_I2C_DYNTIMEO_STARTSTOP
#define CONFIG_TIVA_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_TIVA_I2C_TIMEOTICKS)
#endif

/* GPIO pins ************************************************************************/
/* Macros to convert a I2C pin to a GPIO output */

#define I2C_INPUT  (GPIO_FUNC_INPUT)
#define I2C_OUTPUT (GPIO_FUNC_ODOUTPUT | GPIO_PADTYPE_OD | GPIO_VALUE_ONE)

#define MKI2C_INPUT(p)  (((p) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | I2C_INPUT)
#define MKI2C_OUTPUT(p) (((p) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | I2C_OUTPUT)

/* Debug ****************************************************************************/
/* CONFIG_DEBUG_I2C + CONFIG_DEBUG enables general I2C debug output. */

#ifdef CONFIG_DEBUG_I2C
#define i2cdbg dbg
#define i2cvdbg vdbg
#else
#define i2cdbg(...)
#define i2cvdbg(...)
#endif

#ifndef CONFIG_DEBUG
#undef CONFIG_TIVA_I2C_REGDEBUG
#endif

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard, low-level
 * debug interface syslog() but does not require that any other debug
 * is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#define tiva_i2c_tracereset(p)
#define tiva_i2c_tracenew(p, s)
#define tiva_i2c_traceevent(p, e, a)
#define tiva_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#define CONFIG_I2C_NTRACE 32
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/
/* Interrupt state */

enum tiva_intstate_e {
	INTSTATE_IDLE = 0,			/* No I2C activity */
	INTSTATE_WAITING,			/* Waiting for data transfer to complete */
	INTSTATE_DONE				/* Interrupt activity complete */
};

/* Trace events */

enum tiva_trace_e {
	I2CEVENT_NONE = 0,			/* No events have occurred with this status */
	I2CEVENT_SENDADDRESS,		/* Address sent, param = address */
	I2CEVENT_ERROR,				/* Error occurred, param = MCS */
	I2CEVENT_BUSY,				/* Still busy, param = MCS */
	I2CEVENT_XFRDONE,			/* Transfer completed without error, param = mcnt */
	I2CEVENT_RECVSETUP,			/* Setup to receive the next byte, param = mcnt */
	I2CEVENT_SENDBYTE,			/* Send byte, param = mcnt */
	I2CEVENT_SPURIOUS,			/* Spurious interrupt received, param = msgc */
	I2CEVENT_NEXTMSG,			/* Starting next message, param = msgc */
	I2CEVENT_TIMEOUT,			/* Software detected timeout, param = RIS */
	I2CEVENT_DONE				/* All messages transferred, param = intstate */
};

/* Trace data */

struct tiva_trace_s {
	uint32_t status;			/* I2C 32-bit SR2|SR1 status */
	uint32_t count;				/* Interrupt count when status change */
	enum tiva_trace_e event;	/* Last event that occurred with this status */
	uint32_t parm;				/* Parameter associated with the event */
	uint32_t time;				/* First of event or first status */
};

/* I2C Device hardware configuration */

struct tiva_i2c_config_s {
	uintptr_t base;				/* I2C base address */
#ifndef TIVA_SYSCON_RCGCI2C
	uint32_t rcgbit;			/* Bit in the RCG1 register to enable clocking */
#endif
#ifndef TIVA_SYSCON_SRI2C
	uint32_t rstbit;			/* Bit in the SRCR1 register to reset I2C */
#endif
	uint32_t scl_pin;			/* GPIO configuration for SCL as SCL */
	uint32_t sda_pin;			/* GPIO configuration for SDA as SDA */
#ifndef CONFIG_I2C_POLLED
	int (*isr)(int, void *);	/* Interrupt handler */
	uint8_t irq;				/* IRQ number */
#endif
	uint8_t devno;				/* I2Cn where n = devno */
};

/* I2C Device Private Data */

struct tiva_i2c_priv_s {
	const struct tiva_i2c_config_s *config;	/* Port configuration */
	sem_t exclsem;				/* Mutual exclusion semaphore */
#ifndef CONFIG_I2C_POLLED
	sem_t waitsem;				/* Interrupt wait semaphore */
#endif
	uint8_t refs;				/* Reference count */
	volatile uint8_t intstate;	/* Interrupt handshake (see enum tiva_intstate_e) */

	uint8_t msgc;				/* Message count */
	struct i2c_msg_s *msgv;		/* Message list */
	uint8_t *mptr;				/* Current message buffer */
	int mcnt;					/* Current message length */
	uint16_t mflags;			/* Current message flags */
	uint32_t mstatus;			/* MCS register at the end of the transfer */

#ifdef CONFIG_TIVA_I2C_REGDEBUG
	/* Register level debug */

	bool wrlast;				/* Last was a write */
	uintptr_t addrlast;			/* Last address */
	uint32_t vallast;			/* Last value */
	int ntimes;					/* Number of times */
#endif

#ifdef CONFIG_I2C_TRACE
	/* I2C trace support */

	int tndx;					/* Trace array index */
	int tcount;					/* Number of events with this status */
	uint32_t ttime;				/* Time when the trace was started */
	uint32_t tstatus;			/* Last status read */

	/* The actual trace data */

	struct tiva_trace_s trace[CONFIG_I2C_NTRACE];
#endif
};

/* I2C Device, Instance */

struct tiva_i2c_inst_s {
	const struct i2c_ops_s *ops;	/* Standard I2C operations */
	struct tiva_i2c_priv_s *priv;	/* Common driver private data structure */

	uint32_t frequency;			/* Frequency used in this instantiation */
	uint16_t address;			/* Address used in this instantiation */
	uint16_t flags;				/* Flags used in this instantiation */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

#ifdef CONFIG_TIVA_I2C_REGDEBUG
static bool tiva_i2c_checkreg(struct tiva_i2c_priv_s *priv, bool wr, uint32_t regval, uintptr_t regaddr);
static uint32_t tiva_i2c_getreg(struct tiva_i2c_priv_s *priv, unsigned int offset);
static void tiva_i2c_putreg(struct tiva_i2c_priv_s *priv, unsigned int offset, uint32_t value);
#else
static inline uint32_t tiva_i2c_getreg(struct tiva_i2c_priv_s *priv, unsigned int offset);
static inline void tiva_i2c_putreg(struct tiva_i2c_priv_s *priv, unsigned int offset, uint32_t value);
#endif
static inline void tiva_i2c_sem_wait(struct i2c_dev_s *dev);

#ifdef CONFIG_TIVA_I2C_DYNTIMEO
static useconds_t tiva_i2c_tousecs(int msgc, struct i2c_msg_s *msgv);
#endif							/* CONFIG_TIVA_I2C_DYNTIMEO */

static inline int tiva_i2c_sem_waitdone(struct tiva_i2c_priv_s *priv);
static inline void tiva_i2c_sem_post(struct i2c_dev_s *dev);
static inline void tiva_i2c_sem_init(struct i2c_dev_s *dev);
static inline void tiva_i2c_sem_destroy(struct i2c_dev_s *dev);

#ifdef CONFIG_I2C_TRACE
static void tiva_i2c_tracereset(struct tiva_i2c_priv_s *priv);
static void tiva_i2c_tracenew(struct tiva_i2c_priv_s *priv, uint32_t status);
static void tiva_i2c_traceevent(struct tiva_i2c_priv_s *priv, enum tiva_trace_e event, uint32_t parm);
static void tiva_i2c_tracedump(struct tiva_i2c_priv_s *priv);
#endif							/* CONFIG_I2C_TRACE */

static void tiva_i2c_startxfr(struct tiva_i2c_priv_s *priv);
static void tiva_i2c_nextxfr(struct tiva_i2c_priv_s *priv, uint32_t cmd);
static int tiva_i2c_interrupt(struct tiva_i2c_priv_s *priv, uint32_t status);

#ifndef CONFIG_I2C_POLLED
#ifdef CONFIG_TIVA_I2C0
static int tiva_i2c0_interrupt(int irq, void *context, FAR void *arg);
#endif
#ifdef CONFIG_TIVA_I2C1
static int tiva_i2c1_interrupt(int irq, void *context, FAR void *arg);
#endif
#ifdef CONFIG_TIVA_I2C2
static int tiva_i2c2_interrupt(int irq, void *context, FAR void *arg);
#endif
#ifdef CONFIG_TIVA_I2C3
static int tiva_i2c3_interrupt(int irq, void *context, FAR void *arg);
#endif
#ifdef CONFIG_TIVA_I2C4
static int tiva_i2c4_interrupt(int irq, void *context, FAR void *arg);
#endif
#ifdef CONFIG_TIVA_I2C5
static int tiva_i2c5_interrupt(int irq, void *context, FAR void *arg);
#endif
#ifdef CONFIG_TIVA_I2C6
static int tiva_i2c6_interrupt(int irq, void *context, FAR void *arg);
#endif
#ifdef CONFIG_TIVA_I2C7
static int tiva_i2c7_interrupt(int irq, void *context, FAR void *arg);
#endif
#ifdef CONFIG_TIVA_I2C8
static int tiva_i2c8_interrupt(int irq, void *context, FAR void *arg);
#endif
#ifdef CONFIG_TIVA_I2C9
static int tiva_i2c9_interrupt(int irq, void *context, FAR void *arg);
#endif
#endif							/* !CONFIG_I2C_POLLED */

static int tiva_i2c_initialize(struct tiva_i2c_priv_s *priv, uint32_t frequency);
static int tiva_i2c_uninitialize(struct tiva_i2c_priv_s *priv);
static uint32_t tiva_i2c_setclock(struct tiva_i2c_priv_s *priv, uint32_t frequency);
static uint32_t tiva_i2c_setfrequency(struct i2c_dev_s *dev, uint32_t frequency);
static int tiva_i2c_setaddress(struct i2c_dev_s *dev, int addr, int nbits);
static int tiva_i2c_process(struct i2c_dev_s *dev, struct i2c_msg_s *msgv, int msgc);
static int tiva_i2c_write(struct i2c_dev_s *dev, const uint8_t *buffer, int buflen);
static int tiva_i2c_read(struct i2c_dev_s *dev, uint8_t *buffer, int buflen);

#ifdef CONFIG_I2C_WRITEREAD
static int tiva_i2c_writeread(struct i2c_dev_s *dev, const uint8_t *wbuffer, int wbuflen, uint8_t *buffer, int buflen);
#endif							/* CONFIG_I2C_WRITEREAD */

#ifdef CONFIG_I2C_TRANSFER
static int tiva_i2c_transfer(struct i2c_dev_s *dev, struct i2c_msg_s *msgv, int msgc);
#endif							/* CONFIG_I2C_TRANSFER */

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_TIVA_I2C0
static const struct tiva_i2c_config_s tiva_i2c0_config = {
	.base = TIVA_I2C0_BASE,
#ifndef TIVA_SYSCON_RCGCI2C
	.rcgbit = SYSCON_RCGC1_I2C0,
#endif
#ifndef TIVA_SYSCON_SRI2C
	.rstbit = SYSCON_SRCR1_I2C0,
#endif
	.scl_pin = GPIO_I2C0_SCL,
	.sda_pin = GPIO_I2C0_SDA,
#ifndef CONFIG_I2C_POLLED
	.isr = tiva_i2c0_interrupt,
	.irq = TIVA_IRQ_I2C0,
#endif
	.devno = 0,
};

static struct tiva_i2c_priv_s tiva_i2c0_priv;
#endif

#ifdef CONFIG_TIVA_I2C1
static const struct tiva_i2c_config_s tiva_i2c1_config = {
	.base = TIVA_I2C1_BASE,
#ifndef TIVA_SYSCON_RCGCI2C
	.rcgbit = SYSCON_RCGC1_I2C1,
#endif
#ifndef TIVA_SYSCON_SRI2C
	.rstbit = SYSCON_SRCR1_I2C1,
#endif
	.scl_pin = GPIO_I2C1_SCL,
	.sda_pin = GPIO_I2C1_SDA,
#ifndef CONFIG_I2C_POLLED
	.isr = tiva_i2c1_interrupt,
	.irq = TIVA_IRQ_I2C1,
#endif
	.devno = 1,
};

static struct tiva_i2c_priv_s tiva_i2c1_priv;
#endif

#ifdef CONFIG_TIVA_I2C2
static const struct tiva_i2c_config_s tiva_i2c2_config = {
	.base = TIVA_I2C2_BASE,
#ifndef TIVA_SYSCON_RCGCI2C
	.rcgbit = SYSCON_RCGC1_I2C2,
#endif
#ifndef TIVA_SYSCON_SRI2C
	.rstbit = SYSCON_SRCR1_I2C2,
#endif
	.scl_pin = GPIO_I2C2_SCL,
	.sda_pin = GPIO_I2C2_SDA,
#ifndef CONFIG_I2C_POLLED
	.isr = tiva_i2c2_interrupt,
	.irq = TIVA_IRQ_I2C2,
#endif
	.devno = 2,
};

static struct tiva_i2c_priv_s tiva_i2c2_priv;
#endif

#ifdef CONFIG_TIVA_I2C3
static const struct tiva_i2c_config_s tiva_i2c3_config = {
	.base = TIVA_I2C3_BASE,
#ifndef TIVA_SYSCON_RCGCI2C
	.rcgbit = SYSCON_RCGC1_I2C3,
#endif
#ifndef TIVA_SYSCON_SRI2C
	.rstbit = SYSCON_SRCR1_I2C3,
#endif
	.scl_pin = GPIO_I2C3_SCL,
	.sda_pin = GPIO_I2C3_SDA,
#ifndef CONFIG_I2C_POLLED
	.isr = tiva_i2c3_interrupt,
	.irq = TIVA_IRQ_I2C3,
#endif
	.devno = 3,
};

static struct tiva_i2c_priv_s tiva_i2c3_priv;
#endif

#ifdef CONFIG_TIVA_I2C4
static const struct tiva_i2c_config_s tiva_i2c4_config = {
	.base = TIVA_I2C4_BASE,
#ifndef TIVA_SYSCON_RCGCI2C
	.rcgbit = SYSCON_RCGC1_I2C4,
#endif
#ifndef TIVA_SYSCON_SRI2C
	.rstbit = SYSCON_SRCR1_I2C4,
#endif
	.scl_pin = GPIO_I2C4_SCL,
	.sda_pin = GPIO_I2C4_SDA,
#ifndef CONFIG_I2C_POLLED
	.isr = tiva_i2c4_interrupt,
	.irq = TIVA_IRQ_I2C4,
#endif
	.devno = 4,
};

static struct tiva_i2c_priv_s tiva_i2c4_priv;
#endif

#ifdef CONFIG_TIVA_I2C5
static const struct tiva_i2c_config_s tiva_i2c5_config = {
	.base = TIVA_I2C5_BASE,
#ifndef TIVA_SYSCON_RCGCI2C
	.rcgbit = SYSCON_RCGC1_I2C5,
#endif
#ifndef TIVA_SYSCON_SRI2C
	.rstbit = SYSCON_SRCR1_I2C5,
#endif
	.scl_pin = GPIO_I2C5_SCL,
	.sda_pin = GPIO_I2C5_SDA,
#ifndef CONFIG_I2C_POLLED
	.isr = tiva_i2c5_interrupt,
	.irq = TIVA_IRQ_I2C5,
#endif
	.devno = 5,
};

static struct tiva_i2c_priv_s tiva_i2c5_priv;
#endif

#ifdef CONFIG_TIVA_I2C6
static const struct tiva_i2c_config_s tiva_i2c6_config = {
	.base = TIVA_I2C6_BASE,
#ifndef TIVA_SYSCON_RCGCI2C
	.rcgbit = SYSCON_RCGC1_I2C6,
#endif
#ifndef TIVA_SYSCON_SRI2C
	.rstbit = SYSCON_SRCR1_I2C6,
#endif
	.scl_pin = GPIO_I2C6_SCL,
	.sda_pin = GPIO_I2C6_SDA,
#ifndef CONFIG_I2C_POLLED
	.isr = tiva_i2c6_interrupt,
	.irq = TIVA_IRQ_I2C6,
#endif
	.devno = 6,
};

static struct tiva_i2c_priv_s tiva_i2c6_priv;
#endif

#ifdef CONFIG_TIVA_I2C7
static const struct tiva_i2c_config_s tiva_i2c7_config = {
	.base = TIVA_I2C7_BASE,
#ifndef TIVA_SYSCON_RCGCI2C
	.rcgbit = SYSCON_RCGC1_I2C7,
#endif
#ifndef TIVA_SYSCON_SRI2C
	.rstbit = SYSCON_SRCR1_I2C7,
#endif
	.scl_pin = GPIO_I2C7_SCL,
	.sda_pin = GPIO_I2C7_SDA,
#ifndef CONFIG_I2C_POLLED
	.isr = tiva_i2c7_interrupt,
	.irq = TIVA_IRQ_I2C7,
#endif
	.devno = 7,
};

static struct tiva_i2c_priv_s tiva_i2c7_priv;
#endif

#ifdef CONFIG_TIVA_I2C8
static const struct tiva_i2c_config_s tiva_i2c8_config = {
	.base = TIVA_I2C8_BASE,
#ifndef TIVA_SYSCON_RCGCI2C
	.rcgbit = SYSCON_RCGC1_I2C8,
#endif
#ifndef TIVA_SYSCON_SRI2C
	.rstbit = SYSCON_SRCR1_I2C8,
#endif
	.scl_pin = GPIO_I2C8_SCL,
	.sda_pin = GPIO_I2C8_SDA,
#ifndef CONFIG_I2C_POLLED
	.isr = tiva_i2c8_interrupt,
	.irq = TIVA_IRQ_I2C8,
#endif
	.devno = 8,
};

static struct tiva_i2c_priv_s tiva_i2c8_priv;
#endif

#ifdef CONFIG_TIVA_I2C9
static const struct tiva_i2c_config_s tiva_i2c9_config = {
	.base = TIVA_I2C9_BASE,
#ifndef TIVA_SYSCON_RCGCI2C
	.rcgbit = SYSCON_RCGC1_I2C9,
#endif
#ifndef TIVA_SYSCON_SRI2C
	.rstbit = SYSCON_SRCR1_I2C9,
#endif
	.scl_pin = GPIO_I2C9_SCL,
	.sda_pin = GPIO_I2C9_SDA,
#ifndef CONFIG_I2C_POLLED
	.isr = tiva_i2c9_interrupt,
	.irq = TIVA_IRQ_I2C9,
#endif
	.devno = 9,
};

static struct tiva_i2c_priv_s tiva_i2c9_priv;
#endif

/* Device Structures, Instantiation */

static const struct i2c_ops_s tiva_i2c_ops = {
	.setfrequency = tiva_i2c_setfrequency,
	.setaddress = tiva_i2c_setaddress,
	.write = tiva_i2c_write,
	.read = tiva_i2c_read
#ifdef CONFIG_I2C_WRITEREAD
	, .writeread = tiva_i2c_writeread
#endif
#ifdef CONFIG_I2C_TRANSFER
	, .transfer = tiva_i2c_transfer
#endif
#ifdef CONFIG_I2C_SLAVE
	, .setownaddress = tiva_i2c_setownaddress,
	.registercallback = tiva_i2c_registercallback
#endif
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   regval  - The value to be written
 *   regaddr - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ************************************************************************************/

#ifdef CONFIG_TIVA_I2C_REGDEBUG
static bool tiva_i2c_checkreg(struct tiva_i2c_priv_s *priv, bool wr, uint32_t regval, uintptr_t regaddr)
{
	if (wr == priv->wrlast &&	/* Same kind of access? */
		regval == priv->vallast &&	/* Same value? */
		regaddr == priv->addrlast) {	/* Same address? */
		/* Yes, then just keep a count of the number of times we did this. */

		priv->ntimes++;
		return false;
	} else {
		/* Did we do the previous operation more than once? */

		if (priv->ntimes > 0) {
			/* Yes... show how many times we did it */

			lldbg("...[Repeats %d times]...\n", priv->ntimes);
		}

		/* Save information about the new access */

		priv->wrlast = wr;
		priv->vallast = regval;
		priv->addrlast = regaddr;
		priv->ntimes = 0;
	}

	/* Return true if this is the first time that we have done this operation */

	return true;
}
#endif

/************************************************************************************
 * Name: tiva_i2c_getreg
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ************************************************************************************/

#ifdef CONFIG_TIVA_I2C_REGDEBUG
static uint32_t tiva_i2c_getreg(struct tiva_i2c_priv_s *priv, unsigned int offset)
{
	uintptr_t regaddr = priv->config->base + offset;
	uint32_t regval = getreg32(regaddr);

	if (tiva_i2c_checkreg(priv, false, regval, regaddr)) {
		lldbg("%08x->%08x\n", regaddr, regval);
	}

	return regval;
}
#else
static inline uint32_t tiva_i2c_getreg(struct tiva_i2c_priv_s *priv, unsigned int offset)
{
	return getreg32(priv->config->base + offset);
}
#endif

/************************************************************************************
 * Name: tiva_i2c_putreg
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ************************************************************************************/

#ifdef CONFIG_TIVA_I2C_REGDEBUG
static void tiva_i2c_putreg(struct tiva_i2c_priv_s *priv, unsigned int offset, uint32_t regval)
{
	uintptr_t regaddr = priv->config->base + offset;

	if (tiva_i2c_checkreg(priv, true, regval, regaddr)) {
		lldbg("%08x<-%08x\n", regaddr, regval);
	}

	putreg32(regval, regaddr);
}
#else
static inline void tiva_i2c_putreg(struct tiva_i2c_priv_s *priv, unsigned int offset, uint32_t regval)
{
	putreg32(regval, priv->config->base + offset);
}
#endif

/************************************************************************************
 * Name: tiva_i2c_sem_wait
 *
 * Description:
 *   Take the exclusive access, waiting as necessary
 *
 ************************************************************************************/

static inline void tiva_i2c_sem_wait(struct i2c_dev_s *dev)
{
	struct tiva_i2c_inst_s *inst = (struct tiva_i2c_inst_s *)dev;

	while (sem_wait(&inst->priv->exclsem) != 0) {
		ASSERT(errno == EINTR);
	}
}

/************************************************************************************
 * Name: tiva_i2c_tousecs
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be processed.
 *
 ************************************************************************************/

#ifdef CONFIG_TIVA_I2C_DYNTIMEO
static useconds_t tiva_i2c_tousecs(int msgc, struct i2c_msg_s *msgv)
{
	size_t bytecount = 0;
	int i;

	/* Count the number of bytes left to process */

	for (i = 0; i < msgc; i++) {
		bytecount += msgv[i].length;
	}

	/* Then return a number of microseconds based on a user provided scaling
	 * factor.
	 */

	return (useconds_t)(CONFIG_TIVA_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/************************************************************************************
 * Name: tiva_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ************************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int tiva_i2c_sem_waitdone(struct tiva_i2c_priv_s *priv)
{
	struct timespec abstime;
	irqstate_t flags;
	int ret;

	flags = irqsave();

	/* Enable the master interrupt.  The I2C master module generates an interrupt when
	 * a transaction completes (either transmit or receive), when arbitration is lost,
	 * or when an error occurs during a transaction.
	 */

	tiva_i2c_putreg(priv, TIVA_I2CM_IMR_OFFSET, I2CM_IMR_MIM);

	/* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
	 * are currently disabled but will be temporarily re-enabled below when
	 * sem_timedwait() sleeps.
	 */

	do {
		/* Get the current time */

		(void)clock_gettime(CLOCK_REALTIME, &abstime);

		/* Calculate a time in the future */

#if CONFIG_TIVA_I2C_TIMEOSEC > 0
		abstime.tv_sec += CONFIG_TIVA_I2C_TIMEOSEC;
#endif

		/* Add a value proportional to the number of bytes in the transfer */

#ifdef CONFIG_TIVA_I2C_DYNTIMEO
		abstime.tv_nsec += 1000 * tiva_i2c_tousecs(priv->msgc, priv->msgv);
		if (abstime.tv_nsec >= 1000 * 1000 * 1000) {
			abstime.tv_sec++;
			abstime.tv_nsec -= 1000 * 1000 * 1000;
		}
#elif CONFIG_TIVA_I2C_TIMEOMS > 0
		abstime.tv_nsec += CONFIG_TIVA_I2C_TIMEOMS * 1000 * 1000;
		if (abstime.tv_nsec >= 1000 * 1000 * 1000) {
			abstime.tv_sec++;
			abstime.tv_nsec -= 1000 * 1000 * 1000;
		}
#endif

		/* Wait until either the transfer is complete or the timeout expires */

		ret = sem_timedwait(&priv->waitsem, &abstime);
		if (ret != OK && errno != EINTR) {
			/* Break out of the loop on irrecoverable errors.  This would
			 * include timeouts and mystery errors reported by sem_timedwait.
			 * NOTE that we try again if we are awakened by a signal (EINTR).
			 */

			tiva_i2c_traceevent(priv, I2CEVENT_TIMEOUT, tiva_i2c_getreg(priv, TIVA_I2CM_RIS_OFFSET));
			break;
		}
	}

	/* Loop until the interrupt level transfer is complete. */

	while (priv->intstate != INTSTATE_DONE);

	/* Set the interrupt state back to IDLE */

	priv->intstate = INTSTATE_IDLE;

	/* Disable I2C interrupts */

	tiva_i2c_putreg(priv, TIVA_I2CM_IMR_OFFSET, 0);

	irqrestore(flags);
	return ret;
}
#else
static inline int tiva_i2c_sem_waitdone(struct tiva_i2c_priv_s *priv)
{
	uint32_t timeout;
	uint32_t start;
	uint32_t elapsed;
	uint32_t status;
	int ret;

	/* Get the timeout value */

#ifdef CONFIG_TIVA_I2C_DYNTIMEO
	timeout = USEC2TICK(tiva_i2c_tousecs(priv->msgc, priv->msgv));
#else
	timeout = CONFIG_TIVA_I2C_TIMEOTICKS;
#endif

	/* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
	 * are currently disabled but will be temporarily re-enabled below when
	 * sem_timedwait() sleeps.
	 */

	start = clock_systimer();

	do {
		/* Read the raw interrupt status */

		status = tiva_i2c_getreg(priv, TIVA_I2CM_RIS_OFFSET);

		/* Poll by simply calling the timer interrupt handler with the raw
		 * interrupt status until it reports that it is done.
		 */

		tiva_i2c_interrupt(priv, status);

		/* Calculate the elapsed time */

		elapsed = clock_systimer() - start;
	}

	/* Loop until the transfer is complete. */

	while (priv->intstate != INTSTATE_DONE && elapsed < timeout);

	i2cvdbg("intstate: %d elapsed: %d threshold: %d status: %08x\n", priv->intstate, elapsed, timeout, status);

	/* Set the interrupt state back to IDLE */

	ret = priv->intstate == INTSTATE_DONE ? OK : -ETIMEDOUT;
	priv->intstate = INTSTATE_IDLE;
	return ret;
}
#endif

/************************************************************************************
 * Name: tiva_i2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ************************************************************************************/

static inline void tiva_i2c_sem_post(struct i2c_dev_s *dev)
{
	struct tiva_i2c_inst_s *inst = (struct tiva_i2c_inst_s *)dev;

	sem_post(&inst->priv->exclsem);
}

/************************************************************************************
 * Name: tiva_i2c_sem_init
 *
 * Description:
 *   Initialize semaphores
 *
 ************************************************************************************/

static inline void tiva_i2c_sem_init(struct i2c_dev_s *dev)
{
	struct tiva_i2c_inst_s *inst = (struct tiva_i2c_inst_s *)dev;

	sem_init(&inst->priv->exclsem, 0, 1);
#ifndef CONFIG_I2C_POLLED
	sem_init(&inst->priv->waitsem, 0, 0);
#endif
}

/************************************************************************************
 * Name: tiva_i2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ************************************************************************************/

static inline void tiva_i2c_sem_destroy(struct i2c_dev_s *dev)
{
	struct tiva_i2c_inst_s *inst = (struct tiva_i2c_inst_s *)dev;

	sem_destroy(&inst->priv->exclsem);
#ifndef CONFIG_I2C_POLLED
	sem_destroy(&inst->priv->waitsem);
#endif
}

/************************************************************************************
 * Name: tiva_i2c_trace
 *
 * Description:
 *   I2C trace instrumentation
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void tiva_i2c_traceclear(struct tiva_i2c_priv_s *priv)
{
	struct tiva_trace_s *trace = &priv->trace[priv->tndx];

	trace->status = 0;			/* I2C 32-bit SR2|SR1 status */
	trace->count = 0;			/* Interrupt count when status change */
	trace->event = I2CEVENT_NONE;	/* Last event that occurred with this status */
	trace->parm = 0;			/* Parameter associated with the event */
	trace->time = 0;			/* Time of first status or event */
}

static void tiva_i2c_tracereset(struct tiva_i2c_priv_s *priv)
{
	/* Reset the trace info for a new data collection */

	priv->tndx = 0;
	priv->tcount = 0;
	priv->ttime = clock_systimer();
	priv->tstatus = 0;
	tiva_i2c_traceclear(priv);
}

static void tiva_i2c_tracenew(struct tiva_i2c_priv_s *priv, uint32_t status)
{
	struct tiva_trace_s *trace = &priv->trace[priv->tndx];

	/* Is the current entry uninitialized?  Has the status changed? */

	if (trace->count == 0 || status != trace->status) {
		/* Yes.. Was it the status changed?  */

		if (trace->count != 0) {
			/* Yes.. bump up the trace index (unless we are out of trace entries) */

			if (priv->tndx >= (CONFIG_I2C_NTRACE - 1)) {
				i2cdbg("I2C%d: ERROR: Trace table overflow\n", priv->config->devno);
				return;
			}

			priv->tndx++;
			trace = &priv->trace[priv->tndx];
		}

		/* Initialize the new trace entry */

		tiva_i2c_traceclear(priv);
		trace->status = status;
		trace->count = 1;
		trace->time = clock_systimer();

		/* Save the status and reset the count */

		priv->tstatus = status;
		priv->tcount = 1;
	} else {
		/* Just increment the count of times that we have seen this status */

		trace->count++;
	}
}

static void tiva_i2c_traceevent(struct tiva_i2c_priv_s *priv, enum tiva_trace_e event, uint32_t parm)
{
	struct tiva_trace_s *trace;

	if (event != I2CEVENT_NONE) {
		trace = &priv->trace[priv->tndx];
		if (trace->event != event) {
			/* Initialize the new trace entry */

			trace->event = event;
			trace->parm = parm;
			trace->count = priv->tcount;
			trace->time = clock_systimer();

			/* Bump up the trace index (unless we are out of trace entries) */

			if (priv->tndx >= (CONFIG_I2C_NTRACE - 1)) {
				i2cdbg("I2C%d: ERROR: Trace table overflow\n", priv->config->devno);
				return;
			}

			priv->tndx++;
			priv->tcount++;
			tiva_i2c_traceclear(priv);

			trace = &priv->trace[priv->tndx];
			trace->status = priv->tstatus;
			trace->count = priv->tcount;
			trace->time = clock_systimer();
		} else {
			priv->tcount++;
		}
	}
}

static void tiva_i2c_tracedump(struct tiva_i2c_priv_s *priv)
{
	struct tiva_trace_s *trace;
	int i;

	syslog(LOG_DEBUG, "Elapsed time: %d\n", clock_systimer() - priv->ttime);

	for (i = 0; i <= priv->tndx; i++) {
		trace = &priv->trace[i];
		syslog(LOG_DEBUG, "%2d. STATUS: %08x COUNT: %3d EVENT: %2d PARM: %08x TIME: %d\n", i + 1, trace->status, trace->count, trace->event, trace->parm, trace->time - priv->ttime);
	}
}
#endif							/* CONFIG_I2C_TRACE */

/************************************************************************************
 * Name: tiva_i2c_startxfr
 *
 * Description:
 *   Send the START conditions/force Master mode
 *
 ************************************************************************************/

static void tiva_i2c_startxfr(struct tiva_i2c_priv_s *priv)
{
	struct i2c_msg_s *msg;
	uint32_t regval;

	DEBUGASSERT(priv && priv->msgc > 0);

	/* Get run-time data for the next message */

	msg = priv->msgv;
	priv->mptr = msg->buffer;
	priv->mcnt = msg->length;
	priv->mflags = msg->flags;

	/* Set the Master Slave Address */

	regval = (uint32_t)msg->addr << I2CM_SA_SA_SHIFT;
	if ((msg->flags & I2C_M_READ) != 0) {
		regval |= I2CM_SA_RS;
	}

	tiva_i2c_putreg(priv, TIVA_I2CM_SA_OFFSET, regval);
	tiva_i2c_traceevent(priv, I2CEVENT_SENDADDRESS, msg->addr);

	/* Then initiate the transfer */

	tiva_i2c_nextxfr(priv, I2CM_CS_START);
}

/************************************************************************************
 * Name: tiva_i2c_nextxfr
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ************************************************************************************/

static void tiva_i2c_nextxfr(struct tiva_i2c_priv_s *priv, uint32_t cmd)
{
	/* Set up the basic command.  The STOP bit should be set on the last byte transfer.
	 *
	 * - CASE 1: If this is the last message in the sequence, then the stop bit should
	 *   always be set.
	 * - CASE 2.1.1: The next message may be another read or write of the SAME
	 *   direction (read or write) and to the SAME address WITHOUT repeated start, in
	 *   which case this is really just a continuation of the message.  No STOP is
	 *   needed.
	 * - CASE 2.x.2: The next message may be to the SAME address WITH repeated start.
	 *   Because the repeated start, a direction change is possible.  This is still
	 *   a continuation of the same message sequence and so no STOP is needed.
	 * - CASE 2.2.x: The next message may be a DIFFERENT address WITHOUT repeated
	 *   start.  This would be an error; The STOP will be sent, the next message will
	 *   fail.
	 */

	cmd |= I2CM_CS_RUN;
	if (priv->mcnt < 2) {
		/* Are there more messages in this sequence? */

		if (priv->msgc < 2) {
			/* No.. send the STOP */

			cmd |= I2CM_CS_STOP;
		} else {
			/* Yes.. peek at the next message */

			struct i2c_msg_s *curr = priv->msgv;
			struct i2c_msg_s *next = curr + 1;

			/* Same address as the current message? */

			if (curr->addr != next->addr) {
				/* No.. send the STOP */

				cmd |= I2CM_CS_STOP;
			}
		}
	}

	/* Set up to transfer the next byte.  Are we sending or receiving? */

	if ((priv->mflags & I2C_M_READ) != 0) {
		/* We are receiving data.  We need to ACK UNLESS we are going to send
		 * STOP.
		 */

		if ((cmd & I2CM_CS_STOP) == 0) {
			cmd |= I2CM_CS_ACK;
		}

		/* Write the command to the control register to receive the next byte. */

		tiva_i2c_putreg(priv, TIVA_I2CM_CS_OFFSET, cmd);
		tiva_i2c_traceevent(priv, I2CEVENT_RECVSETUP, priv->mcnt);
	} else {
		uint32_t dr;

		/* We are sending data.  Write the data to be sent to the DR register. */

		dr = (uint32_t)*priv->mptr++;
		tiva_i2c_putreg(priv, TIVA_I2CM_DR_OFFSET, dr << I2CM_DR_SHIFT);

		/* Write the command to the control register to send the byte in the DR
		 * register.
		 */

		tiva_i2c_putreg(priv, TIVA_I2CM_CS_OFFSET, cmd);
		tiva_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->mcnt);
	}

	priv->intstate = INTSTATE_WAITING;
}

/************************************************************************************
 * Name: tiva_i2c_interrupt
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ************************************************************************************/

static int tiva_i2c_interrupt(struct tiva_i2c_priv_s *priv, uint32_t status)
{
	/* Check for new trace setup */

	tiva_i2c_tracenew(priv, status);

	/* Check for a master interrupt?  The I2C master module generates an interrupt when
	 * a transaction completes (either transmit or receive), when arbitration is lost,
	 * or when an error occurs during a transaction.
	 */

	if ((status & I2CM_RIS_MRIS) != 0) {
		uint32_t mcs;

#ifndef CONFIG_I2C_POLLED
		/* Clear the pending master interrupt */

		tiva_i2c_putreg(priv, TIVA_I2CM_ICR_OFFSET, I2CM_ICR_MIC);
		status &= ~I2CM_RIS_MRIS;

		/* Workaround for I2C master interrupt clear errata for rev B Tiva
		 * devices.  For later devices, this write is ignored and therefore
		 * harmless (other than the slight performance hit).
		 */

		(void)tiva_i2c_getreg(priv, TIVA_I2CM_MIS_OFFSET);
#endif

		/* We need look at the Master Control/Status register to determine the cause
		 * of the master interrupt.
		 */

		mcs = tiva_i2c_getreg(priv, TIVA_I2CM_CS_OFFSET);

		/* If the busy bit is set, then the other bits are not valid */

		if ((mcs & I2CM_CS_BUSY) != 0) {
			tiva_i2c_traceevent(priv, I2CEVENT_BUSY, mcs);
		}

		/* Check for errors, in which case, stop the transfer and return. */

#if 0							/* I2CM_CS_CLKTO */
		else if ((mcs & (I2CM_CS_ERROR | I2CM_CS_ARBLST | I2CM_CS_CLKTO)) != 0)
#else
		else if ((mcs & (I2CM_CS_ERROR | I2CM_CS_ARBLST)) != 0)
#endif
		{
			tiva_i2c_traceevent(priv, I2CEVENT_ERROR, mcs);

			/* Disable further interrupts */

			tiva_i2c_putreg(priv, TIVA_I2CM_IMR_OFFSET, 0);

#ifndef CONFIG_I2C_POLLED
			/* Is there a thread waiting for this event (there should be) */

			if (priv->intstate != INTSTATE_IDLE && priv->intstate != INTSTATE_DONE) {
				/* Yes.. inform the thread that the transfer is complete
				 * and wake it up.
				 */

				sem_post(&priv->waitsem);
				priv->mstatus = mcs;
				priv->intstate = INTSTATE_DONE;
			}
#else
			priv->mstatus = mcs;
			priv->intstate = INTSTATE_DONE;
#endif
		}

		/* Otherwise, the last transfer must have completed successfully */

		else {
			int dr;

			tiva_i2c_traceevent(priv, I2CEVENT_XFRDONE, priv->mcnt);

			/* Read from the DR register */

			dr = tiva_i2c_getreg(priv, TIVA_I2CM_DR_OFFSET);

			/* We check for msgc > 0 here as an unexpected interrupt with
			 * due to noise on the I2C cable can otherwise cause msgc to
			 * wrap causing memory overwrite
			 */

			if (priv->msgc > 0 && priv->msgv != NULL) {
				/* Was this the completion of an address or of the data portion
				 * of the transfer?
				 */

				DEBUGASSERT(priv->mcnt > 0);

				if (priv->intstate == INTSTATE_WAITING) {
					/* Data transfer completed.  Are we sending or receiving data? */

					if ((priv->mflags & I2C_M_READ) != 0) {
						/* We are receiving data.  Copy the received data to
						 * the user buffer
						 */

						*priv->mptr++ = (uint8_t)dr;
					}

					/* Decrement the count of bytes remaining to be sent */

					priv->mcnt--;
				}

				/* Was that the last byte of this message? */

				if (priv->mcnt > 0) {
					/* Send the next byte */

					tiva_i2c_nextxfr(priv, 0);
				} else {
					/* Increment to next pointer and decrement message count */

					priv->msgv++;
					priv->msgc--;

					/* Is there another message to be sent? */

					if (priv->msgc > 0) {
						/* Do we need to terminate or restart after this byte?
						 * If there are more messages to send, then we may
						 * continue with or without the (repeated) start bit.
						 */

						tiva_i2c_traceevent(priv, I2CEVENT_NEXTMSG, priv->msgc);
						if ((priv->msgv->flags & I2C_M_NORESTART) != 0) {
							/* Just continue transferring data.  In this case,
							 * no STOP was sent at the end of the last message
							 * and the there is no new address.
							 *
							 * REVISIT: In this case, the address or the
							 * direction of the transfer cannot be permitted to
							 * change
							 */

							tiva_i2c_nextxfr(priv, 0);
						} else {
							/* Set the repeated start.  No STOP was sent at the
							 * end of the previous message.
							 */

							tiva_i2c_startxfr(priv);
						}
					} else {
						/* No.. then we are finished */

						tiva_i2c_traceevent(priv, I2CEVENT_DONE, priv->intstate);

						/* Disable further interrupts */

						tiva_i2c_putreg(priv, TIVA_I2CM_IMR_OFFSET, 0);

#ifndef CONFIG_I2C_POLLED
						/* Is there a thread waiting for this event (there
						 * should be)
						 */

						if (priv->intstate != INTSTATE_IDLE && priv->intstate != INTSTATE_DONE) {
							/* Yes.. inform the thread that the transfer is
							 * complete and wake it up.
							 */

							sem_post(&priv->waitsem);
							priv->mstatus = 0;
							priv->intstate = INTSTATE_DONE;
						}
#else
						priv->mstatus = 0;
						priv->intstate = INTSTATE_DONE;
#endif
					}
				}
			}

			/* There is no pending message? Must be a spurious interrupt */

			else {
				tiva_i2c_traceevent(priv, I2CEVENT_SPURIOUS, priv->msgc);
			}
		}
	}

	/* Make sure that all pending interrupts were handled */

#ifndef CONFIG_I2C_POLLED
	DEBUGASSERT(status == 0);
#endif
	return OK;
}

/************************************************************************************
 * Name: tiva_i2c0_interrupt
 *
 * Description:
 *   I2C0 interrupt service routine
 *
 ************************************************************************************/

#if !defined(CONFIG_I2C_POLLED) && defined(CONFIG_TIVA_I2C0)
static int tiva_i2c0_interrupt(int irq, void *context, FAR void *arg)
{
	struct tiva_i2c_priv_s *priv;
	uint32_t status;

	/* Read the masked interrupt status */

	priv = &tiva_i2c0_priv;
	status = tiva_i2c_getreg(priv, TIVA_I2CM_MIS_OFFSET);

	/* Let the common interrupt handler do the rest of the work */

	return tiva_i2c_interrupt(priv, status);
}
#endif

/************************************************************************************
 * Name: tiva_i2c1_interrupt
 *
 * Description:
 *   I2C1 interrupt service routine
 *
 ************************************************************************************/

#if !defined(CONFIG_I2C_POLLED) && defined(CONFIG_TIVA_I2C1)
static int tiva_i2c1_interrupt(int irq, void *context, FAR void *arg)
{
	struct tiva_i2c_priv_s *priv;
	uint32_t status;

	/* Read the masked interrupt status */

	priv = &tiva_i2c1_priv;
	status = tiva_i2c_getreg(priv, TIVA_I2CM_MIS_OFFSET);

	/* Let the common interrupt handler do the rest of the work */

	return tiva_i2c_interrupt(priv, status);
}
#endif

/************************************************************************************
 * Name: tiva_i2c2_interrupt
 *
 * Description:
 *   I2C2 interrupt service routine
 *
 ************************************************************************************/

#if !defined(CONFIG_I2C_POLLED) && defined(CONFIG_TIVA_I2C2)
static int tiva_i2c2_interrupt(int irq, void *context, FAR void *arg)
{
	struct tiva_i2c_priv_s *priv;
	uint32_t status;

	/* Read the masked interrupt status */

	priv = &tiva_i2c2_priv;
	status = tiva_i2c_getreg(priv, TIVA_I2CM_MIS_OFFSET);

	/* Let the common interrupt handler do the rest of the work */

	return tiva_i2c_interrupt(priv, status);
}
#endif

/************************************************************************************
 * Name: tiva_i2c3_interrupt
 *
 * Description:
 *   I2C2 interrupt service routine
 *
 ************************************************************************************/

#if !defined(CONFIG_I2C_POLLED) && defined(CONFIG_TIVA_I2C3)
static int tiva_i2c3_interrupt(int irq, void *context, FAR void *arg)
{
	struct tiva_i2c_priv_s *priv;
	uint32_t status;

	/* Read the masked interrupt status */

	priv = &tiva_i2c3_priv;
	status = tiva_i2c_getreg(priv, TIVA_I2CM_MIS_OFFSET);

	/* Let the common interrupt handler do the rest of the work */

	return tiva_i2c_interrupt(priv, status);
}
#endif

/************************************************************************************
 * Name: tiva_i2c4_interrupt
 *
 * Description:
 *   I2C4 interrupt service routine
 *
 ************************************************************************************/

#if !defined(CONFIG_I2C_POLLED) && defined(CONFIG_TIVA_I2C4)
static int tiva_i2c4_interrupt(int irq, void *context, FAR void *arg)
{
	struct tiva_i2c_priv_s *priv;
	uint32_t status;

	/* Read the masked interrupt status */

	priv = &tiva_i2c4_priv;
	status = tiva_i2c_getreg(priv, TIVA_I2CM_MIS_OFFSET);

	/* Let the common interrupt handler do the rest of the work */

	return tiva_i2c_interrupt(priv, status);
}
#endif

/************************************************************************************
 * Name: tiva_i2c5_interrupt
 *
 * Description:
 *   I2C5 interrupt service routine
 *
 ************************************************************************************/

#if !defined(CONFIG_I2C_POLLED) && defined(CONFIG_TIVA_I2C5)
static int tiva_i2c5_interrupt(int irq, void *context, FAR void *arg)
{
	struct tiva_i2c_priv_s *priv;
	uint32_t status;

	/* Read the masked interrupt status */

	priv = &tiva_i2c5_priv;
	status = tiva_i2c_getreg(priv, TIVA_I2CM_MIS_OFFSET);

	/* Let the common interrupt handler do the rest of the work */

	return tiva_i2c_interrupt(priv, status);
}
#endif

/************************************************************************************
 * Name: tiva_i2c6_interrupt
 *
 * Description:
 *   I2C6 interrupt service routine
 *
 ************************************************************************************/

#if !defined(CONFIG_I2C_POLLED) && defined(CONFIG_TIVA_I2C6)
static int tiva_i2c6_interrupt(int irq, void *context, FAR void *arg)
{
	struct tiva_i2c_priv_s *priv;
	uint32_t status;

	/* Read the masked interrupt status */

	priv = &tiva_i2c6_priv;
	status = tiva_i2c_getreg(priv, TIVA_I2CM_MIS_OFFSET);

	/* Let the common interrupt handler do the rest of the work */

	return tiva_i2c_interrupt(priv, status);
}
#endif

/************************************************************************************
 * Name: tiva_i2c7_interrupt
 *
 * Description:
 *   I2C7 interrupt service routine
 *
 ************************************************************************************/

#if !defined(CONFIG_I2C_POLLED) && defined(CONFIG_TIVA_I2C7)
static int tiva_i2c7_interrupt(int irq, void *context, FAR void *arg)
{
	struct tiva_i2c_priv_s *priv;
	uint32_t status;

	/* Read the masked interrupt status */

	priv = &tiva_i2c7_priv;
	status = tiva_i2c_getreg(priv, TIVA_I2CM_MIS_OFFSET);

	/* Let the common interrupt handler do the rest of the work */

	return tiva_i2c_interrupt(priv, status);
}
#endif

/************************************************************************************
 * Name: tiva_i2c8_interrupt
 *
 * Description:
 *   I2C8 interrupt service routine
 *
 ************************************************************************************/

#if !defined(CONFIG_I2C_POLLED) && defined(CONFIG_TIVA_I2C8)
static int tiva_i2c8_interrupt(int irq, void *context, FAR void *arg)
{
	struct tiva_i2c_priv_s *priv;
	uint32_t status;

	/* Read the masked interrupt status */

	priv = &tiva_i2c8_priv;
	status = tiva_i2c_getreg(priv, TIVA_I2CM_MIS_OFFSET);

	/* Let the common interrupt handler do the rest of the work */

	return tiva_i2c_interrupt(priv, status);
}
#endif

/************************************************************************************
 * Name: tiva_i2c9_interrupt
 *
 * Description:
 *   I2C9 interrupt service routine
 *
 ************************************************************************************/

#if !defined(CONFIG_I2C_POLLED) && defined(CONFIG_TIVA_I2C9)
static int tiva_i2c9_interrupt(int irq, void *context, FAR void *arg)
{
	struct tiva_i2c_priv_s *priv;
	uint32_t status;

	/* Read the masked interrupt status */

	priv = &tiva_i2c9_priv;
	status = tiva_i2c_getreg(priv, TIVA_I2CM_MIS_OFFSET);

	/* Let the common interrupt handler do the rest of the work */

	return tiva_i2c_interrupt(priv, status);
}
#endif

/************************************************************************************
 * Name: tiva_i2c_initialize
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ************************************************************************************/

static int tiva_i2c_initialize(struct tiva_i2c_priv_s *priv, uint32_t frequency)
{
	const struct tiva_i2c_config_s *config = priv->config;
	uint32_t regval;
	int ret;

	i2cvdbg("I2C%d: refs=%d\n", config->devno, priv->refs);

	/* Enable power and clocking to the I2C peripheral.
	 *
	 * - Enable Power (TM4C129 family only):  Applies power (only) to the I2C
	 *   peripheral.  This is not an essential step since enabling clocking
	 *   will also apply power.  The only significance is that the I2C state
	 *   will be retained if the I2C clocking is subsequently disabled.
	 * - Enable Clocking (All families):  Applies both power and clocking to
	 *   the I2C peripheral, bringing it a fully functional state.
	 */

#ifdef TIVA_SYSCON_RCGCI2C
	tiva_i2c_enablepwr(config->devno);
	tiva_i2c_enableclk(config->devno);

	i2cvdbg("I2C%d: RCGI2C[%08x]=%08x\n", config->devno, TIVA_SYSCON_RCGCI2C, getreg32(TIVA_SYSCON_RCGCI2C));
#else
	modifyreg32(TIVA_SYSCON_RCGC1, 0, priv->rcgbit);

	i2cvdbg("I2C%d: RCGC1[%08x]=%08x\n", config->devno, TIVA_SYSCON_RCGC1, getreg32(TIVA_SYSCON_RCGC1));
#endif

	/* Reset the I2C block */

#ifdef TIVA_SYSCON_SRI2C
	modifyreg32(TIVA_SYSCON_SRI2C, 0, SYSCON_SRI2C(config->devno));
	modifyreg32(TIVA_SYSCON_SRI2C, SYSCON_SRI2C(config->devno), 0);
#else
	modifyreg32(TIVA_SYSCON_SRCR1, 0, priv->rstbit);
	modifyreg32(TIVA_SYSCON_SRCR1, priv->rstbit, 0);
#endif

	/* Configure pins */

	i2cvdbg("I2C%d: SCL=%08x SDA=%08x\n", config->devno, config->scl_pin, config->sda_pin);

	ret = tiva_configgpio(config->scl_pin);
	if (ret < 0) {
		i2cvdbg("I2C%d: tiva_configgpio(%08x) failed: %d\n", config->scl_pin, ret);
		return ret;
	}

	ret = tiva_configgpio(config->sda_pin);
	if (ret < 0) {
		i2cvdbg("I2C%d: tiva_configgpio(%08x) failed: %d\n", config->sda_pin, ret);
		tiva_configgpio(MKI2C_INPUT(config->scl_pin));
		return ret;
	}

	/* Enable the I2C master block */

	regval = tiva_i2c_getreg(priv, TIVA_I2CM_CR_OFFSET);
	regval |= I2CM_CR_MFE;
	tiva_i2c_putreg(priv, TIVA_I2CM_CR_OFFSET, regval);

#ifdef TIVA_I2CSC_PC_OFFSET
#ifdef CONFIG_TIVA_I2C_HIGHSPEED
	/* Enable high-speed mode */

	tiva_i2c_putreg(priv, TIVA_I2CSC_PC_OFFSET, I2CSC_PC_HS);
#else
	/* Disable high-speed mode */

	tiva_i2c_putreg(priv, TIVA_I2CSC_PC_OFFSET, 0);
#endif
#endif

	/* Configure the the initial I2C clock frequency. */

	(void)tiva_i2c_setclock(priv, frequency);

	/* Attach interrupt handlers and enable interrupts at the NVIC (still
	 * disabled at the source).
	 */

#ifndef CONFIG_I2C_POLLED
	(void)irq_attach(config->irq, config->isr, NULL);
	up_enable_irq(config->irq);
#endif

	return OK;
}

/************************************************************************************
 * Name: tiva_i2c_uninitialize
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ************************************************************************************/

static int tiva_i2c_uninitialize(struct tiva_i2c_priv_s *priv)
{
	uint32_t regval;

	i2cvdbg("I2C%d: refs=%d\n", priv->config->devno, priv->refs);

	/* Disable I2C */

	regval = tiva_i2c_getreg(priv, TIVA_I2CM_CR_OFFSET);
	regval &= ~I2CM_CR_MFE;
	tiva_i2c_putreg(priv, TIVA_I2CM_CR_OFFSET, regval);

	/* Unconfigure GPIO pins */

	tiva_configgpio(MKI2C_INPUT(priv->config->scl_pin));
	tiva_configgpio(MKI2C_INPUT(priv->config->sda_pin));

	/* Disable and detach interrupts */

#ifndef CONFIG_I2C_POLLED
	up_disable_irq(priv->config->irq);
	irq_detach(priv->config->irq);
#endif

	/* Disable clocking */

#ifdef TIVA_SYSCON_RCGCI2C
	modifyreg32(TIVA_SYSCON_RCGCI2C, SYSCON_RCGCI2C(priv->config->devno), 0);
#else
	modifyreg32(TIVA_SYSCON_RCGC1, priv->rcgbit, 0);
#endif

	return OK;
}

/************************************************************************************
 * Name: tiva_i2c_setclock
 *
 * Description:
 *   Set the I2C frequency
 *
 ************************************************************************************/

static uint32_t tiva_i2c_setclock(struct tiva_i2c_priv_s *priv, uint32_t frequency)
{
	uint32_t regval;
	uint32_t tmp;

	i2cvdbg("I2C%d: frequency: %u\n", priv->config->devno, frequency);

	/* Calculate the clock divider that results in the highest frequency that
	 * is than or equal to the desired speed.
	 */

	tmp = 2 * 10 * frequency;
	regval = (((SYSCLK_FREQUENCY + tmp - 1) / tmp) - 1) << I2CM_TPR_SHIFT;

	DEBUGASSERT((regval & I2CM_TPR_MASK) == regval);
	tiva_i2c_putreg(priv, TIVA_I2CM_TPR_OFFSET, regval);

#if defined(CONFIG_TIVA_I2C_HIGHSPEED) && defined(TIVA_I2CSC_PC_OFFSET)
	/* If the I2C peripheral is High-Speed enabled then choose the highest
	 * speed that is less than or equal to 3.4 Mbps.
	 */

	regval = tiva_i2c_getreg(priv, TIVA_I2CSC_PC_OFFSET);
	if ((regval & I2CSC_PC_HS) != 0) {
		tmp = (2 * 3 * 3400000);
		regval = (((SYSCLK_FREQUENCY + tmp - 1) / tmp) - 1) << I2CM_TPR_SHIFT;

		tiva_i2c_putreg(priv, TIVA_I2CM_TPR_OFFSET, I2CM_TPR_HS | regval);
	}
#endif

	return frequency;
}

/************************************************************************************
 * Name: tiva_i2c_setfrequency
 *
 * Description:
 *   Set the I2C frequency
 *
 ************************************************************************************/

static uint32_t tiva_i2c_setfrequency(struct i2c_dev_s *dev, uint32_t frequency)
{
	struct tiva_i2c_inst_s *inst = (struct tiva_i2c_inst_s *)dev;
	struct tiva_i2c_priv_s *priv;

	DEBUGASSERT(inst && inst->priv);
	priv = inst->priv;

	i2cvdbg("I2C%d: frequency: %u\n", inst->priv->config->devno, frequency);

	/* Get exclusive access to the I2C device */

	tiva_i2c_sem_wait(dev);

	/* Set the clocking for the selected frequency */

	inst->frequency = tiva_i2c_setclock(priv, frequency);

	tiva_i2c_sem_post(dev);
	return frequency;
}

/************************************************************************************
 * Name: tiva_i2c_setaddress
 *
 * Description:
 *   Set the I2C slave address
 *
 ************************************************************************************/

static int tiva_i2c_setaddress(struct i2c_dev_s *dev, int addr, int nbits)
{
	struct tiva_i2c_inst_s *inst = (struct tiva_i2c_inst_s *)dev;

	DEBUGASSERT(inst && inst->priv && inst->priv->config);
	i2cvdbg("I2C%d: addr: %02x nbits=%d\n", inst->priv->config->devno, addr, nbits);

	tiva_i2c_sem_wait(dev);

	inst->address = addr;
	inst->flags = (nbits == 10) ? I2C_M_TEN : 0;

	tiva_i2c_sem_post(dev);
	return OK;
}

/************************************************************************************
 * Name: tiva_i2c_process
 *
 * Description:
 *   Common I2C transfer logic
 *
 ************************************************************************************/

static int tiva_i2c_process(struct i2c_dev_s *dev, struct i2c_msg_s *msgv, int msgc)
{
	struct tiva_i2c_inst_s *inst = (struct tiva_i2c_inst_s *)dev;
	struct tiva_i2c_priv_s *priv = inst->priv;
	uint32_t regval;
	int errcode = OK;

	ASSERT(msgc > 0);

	i2cvdbg("I2C%d: msgc=%d\n", priv->config->devno, msgc);

	/* Reset mptr and mcnt to ensure an unexpected data interrupt doesn't
	 * overwrite stale data.
	 */

	priv->mcnt = 0;
	priv->mptr = NULL;

	priv->msgv = msgv;
	priv->msgc = msgc;

	priv->mstatus = 0;

	/* Reset I2C trace logic */

	tiva_i2c_tracereset(priv);
	tiva_i2c_tracenew(priv, 0);

	/* Set I2C clock frequency  */

	tiva_i2c_setclock(priv, inst->frequency);

	/* Send the address, then the process moves into the ISR.  I2C
	 * interrupts will be enabled within tiva_i2c_waitdone().
	 */

	tiva_i2c_startxfr(priv);

	/* Wait for an ISR, if there was a timeout, fetch latest status to get
	 * the BUSY flag.
	 */

	if (tiva_i2c_sem_waitdone(priv) < 0) {
		i2cdbg("I2C%d: ERROR: Timed out\n", priv->config->devno);
		errcode = ETIMEDOUT;
	}
#if 0							/* I2CM_CS_CLKTO */
	else if ((priv->mstatus & (I2CM_CS_ERROR | I2CM_CS_ARBLST | I2CM_CS_CLKTO)) != 0)
#else
	else if ((priv->mstatus & (I2CM_CS_ERROR | I2CM_CS_ARBLST)) != 0)
#endif
	{
		i2cdbg("I2C%d: ERROR:  I2C error status: %08x\n", priv->config->devno, priv->mstatus);

		if ((priv->mstatus & I2CM_CS_ARBLST) != 0) {
			/* Arbitration Lost */

			errcode = EAGAIN;
		} else if ((priv->mstatus & (I2CM_CS_ADRACK | I2CM_CS_DATACK)) != 0) {
			/* Acknowledge Failure */

			errcode = ENXIO;
		}
#if 0							/* I2CM_CS_CLKTO */
		else if ((priv->mstatus & I2CM_CS_CLKTO) != 0) {
			/* Timeout */

			errcode = ETIME;
		}
#endif
		else {
			/* Something else? */

			errcode = EIO;
		}
	}

	/* This is not an error, but should not happen.  The I2CM_CS_BUSBSY signal
	 * can hang, however.  This normally indicates the STOP was never sent,
	 * possibly because some other error occurred.
	 *
	 * The status bits are not valid if BUSY is set.  But in this context I
	 * assume that busy bit stuck on would be a very bad situation, worthy
	 * of a reset.
	 */

	regval = tiva_i2c_getreg(priv, TIVA_I2CM_CS_OFFSET);
	if ((regval & (I2CM_CS_BUSY | I2CM_CS_BUSBSY)) != 0) {
		/* I2C Bus is for some reason busy.  If I2CM_CS_BUSY then none of the
		 * other bits are valid.
		 */

		i2cdbg("I2C%d: ERROR:  I2C still busy: %08x\n", priv->config->devno, regval);

		/* Reset and reinitialize the I2C hardware */

		tiva_i2c_initialize(priv, inst->frequency);

		/* Is the busy condition a consequence of some other error? */

		if (errcode == OK) {
			/* No... then just being busy is the cause of the error */

			errcode = EBUSY;
		}
	}

	/* Dump the trace result */

	tiva_i2c_tracedump(priv);

	/* Ensure that no ISR happening after we finish can overwrite any user data */

	priv->mcnt = 0;
	priv->mptr = NULL;

	tiva_i2c_sem_post(dev);

	return -errcode;
}

/************************************************************************************
 * Name: tiva_i2c_write
 *
 * Description:
 *   Write I2C data
 *
 ************************************************************************************/

static int tiva_i2c_write(struct i2c_dev_s *dev, const uint8_t *buffer, int buflen)
{
	struct tiva_i2c_inst_s *inst = (struct tiva_i2c_inst_s *)dev;
	struct i2c_msg_s msgv = {
		.addr = inst->address,
		.flags = inst->flags,
		.buffer = (uint8_t *)buffer,
		.length = buflen
	};

	DEBUGASSERT(inst && inst->priv && inst->priv->config);
	i2cvdbg("I2C%d: buflen=%d\n", inst->priv->config->devno, buflen);

	tiva_i2c_sem_wait(dev);		/* ensure that address or flags don't change meanwhile */
	return tiva_i2c_process(dev, &msgv, 1);
}

/************************************************************************************
 * Name: tiva_i2c_read
 *
 * Description:
 *   Read I2C data
 *
 ************************************************************************************/

int tiva_i2c_read(struct i2c_dev_s *dev, uint8_t *buffer, int buflen)
{
	struct tiva_i2c_inst_s *inst = (struct tiva_i2c_inst_s *)dev;
	struct i2c_msg_s msgv = {
		.addr = inst->address,
		.flags = inst->flags | I2C_M_READ,
		.buffer = buffer,
		.length = buflen
	};

	DEBUGASSERT(inst && inst->priv && inst->priv->config);
	i2cvdbg("I2C%d: buflen=%d\n", inst->priv->config->devno, buflen);

	tiva_i2c_sem_wait(dev);		/* ensure that address or flags don't change meanwhile */
	return tiva_i2c_process(dev, &msgv, 1);
}

/************************************************************************************
 * Name: tiva_i2c_writeread
 *
 * Description:
 *  Read then write I2C data
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_WRITEREAD
static int tiva_i2c_writeread(struct i2c_dev_s *dev, const uint8_t *wbuffer, int wbuflen, uint8_t *buffer, int buflen)
{
	struct tiva_i2c_inst_s *inst = (struct tiva_i2c_inst_s *)dev;
	struct i2c_msg_s msgv[2] = {
		{
			.addr = inst->address,
			.flags = inst->flags,
			.buffer = (uint8_t *)wbuffer,	/* This is really ugly, sorry const ... */
			.length = wbuflen
		},
		{
			.addr = inst->address,
			.flags = inst->flags | ((buflen > 0) ? I2C_M_READ : I2C_M_NORESTART),
			.buffer = buffer,
			.length = (buflen > 0) ? buflen : -buflen
		}
	};

	DEBUGASSERT(inst && inst->priv && inst->priv->config);
	i2cvdbg("I2C%d: wbuflen=%d buflen=%d\n", inst->priv->config->devno, wbuflen, buflen);

	tiva_i2c_sem_wait(dev);		/* Ensure that address or flags don't change meanwhile */
	return tiva_i2c_process(dev, msgv, 2);
}
#endif

/************************************************************************************
 * Name: tiva_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_TRANSFER
static int tiva_i2c_transfer(struct i2c_dev_s *dev, struct i2c_msg_s *msgv, int msgc)
{
	struct tiva_i2c_inst_s *inst = (struct tiva_i2c_inst_s *)dev;

	DEBUGASSERT(inst && inst->priv && inst->priv->config);
	i2cvdbg("I2C%d: msgc=%d\n", inst->priv->config->devno, msgc);
	UNUSED(inst);

	tiva_i2c_sem_wait(dev);		/* Ensure that address or flags don't change meanwhile */
	return tiva_i2c_process(dev, msgv, msgc);
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_i2cinitialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ************************************************************************************/

struct i2c_dev_s *up_i2cinitialize(int port)
{
	struct tiva_i2c_priv_s *priv = NULL;	/* Private data of device with multiple instances */
	struct tiva_i2c_inst_s *inst = NULL;	/* Device, single instance */
	const struct tiva_i2c_config_s *config;	/* Constant configuration */
	int irqs;

	i2cvdbg("I2C%d: Initialize\n", port);

	/* Get I2C private structure */

	switch (port) {
#ifdef CONFIG_TIVA_I2C0
	case 0:
		priv = &tiva_i2c0_priv;
		config = &tiva_i2c0_config;
		break;
#endif

#ifdef CONFIG_TIVA_I2C1
	case 1:
		priv = &tiva_i2c1_priv;
		config = &tiva_i2c1_config;
		break;
#endif

#ifdef CONFIG_TIVA_I2C2
	case 2:
		priv = &tiva_i2c2_priv;
		config = &tiva_i2c2_config;
		break;
#endif

#ifdef CONFIG_TIVA_I2C3
	case 3:
		priv = &tiva_i2c3_priv;
		config = &tiva_i2c3_config;
		break;
#endif

#ifdef CONFIG_TIVA_I2C4
	case 4:
		priv = &tiva_i2c4_priv;
		config = &tiva_i2c4_config;
		break;
#endif

#ifdef CONFIG_TIVA_I2C5
	case 5:
		priv = &tiva_i2c5_priv;
		config = &tiva_i2c5_config;
		break;
#endif

#ifdef CONFIG_TIVA_I2C6
	case 6:
		priv = &tiva_i2c6_priv;
		config = &tiva_i2c6_config;
		break;
#endif

#ifdef CONFIG_TIVA_I2C7
	case 7:
		priv = &tiva_i2c7_priv;
		config = &tiva_i2c7_config;
		break;
#endif

#ifdef CONFIG_TIVA_I2C8
	case 8:
		priv = &tiva_i2c8_priv;
		config = &tiva_i2c8_config;
		break;
#endif

#ifdef CONFIG_TIVA_I2C9
	case 9:
		priv = &tiva_i2c9_priv;
		config = &tiva_i2c9_config;
		break;
#endif

	default:
		i2cdbg("I2C%d: ERROR: Not supported\n", port);
		return NULL;
	}

	/* Allocate instance */

	inst = kmm_malloc(sizeof(struct tiva_i2c_inst_s));
	if (!inst) {
		i2cdbg("I2C%d: ERROR: Failed to allocate device instance\n", port);
		return NULL;
	}

	/* Initialize instance */

	inst->ops = &tiva_i2c_ops;
	inst->priv = priv;
	inst->frequency = 100000;
	inst->address = 0;
	inst->flags = 0;

	/* Initialize private data for the first time, increment reference count,
	 * power-up hardware and configure GPIOs.
	 */

	irqs = irqsave();

	priv->refs++;
	if (priv->refs == 1) {
		/* Initialize the device structure */

		priv->config = config;
		tiva_i2c_sem_init((struct i2c_dev_s *)inst);

		/* Initialize the I2C hardware */

		tiva_i2c_initialize(priv, 100000);
	}

	irqrestore(irqs);
	return (struct i2c_dev_s *)inst;
}

/************************************************************************************
 * Name: up_i2cuninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ************************************************************************************/

int up_i2cuninitialize(struct i2c_dev_s *dev)
{
	struct tiva_i2c_inst_s *inst = (struct tiva_i2c_inst_s *)dev;
	struct tiva_i2c_priv_s *priv;
	int irqs;

	DEBUGASSERT(inst && inst->priv);
	priv = inst->priv;

	DEBUGASSERT(priv->config && priv->refs > 0);
	i2cvdbg("I2C%d: Uninitialize\n", priv->config->devno);

	/* Decrement reference count and check for underflow */

	irqs = irqsave();

	/* Free this instance */

	kmm_free(inst);

	/* Check if the reference count will decrement to zero */

	if (priv->refs < 2) {
		/* Yes.. Disable power and other HW resource (GPIO's) */

		tiva_i2c_uninitialize(priv);
		priv->refs = 0;

		/* Release unused resources */

		tiva_i2c_sem_destroy(dev);
	} else {
		/* No.. just decrement the number of references to the device */

		priv->refs--;
	}

	irqrestore(irqs);
	return OK;
}

/************************************************************************************
 * Name: up_i2creset
 *
 * Description:
 *   Reset an I2C bus
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_RESET
int up_i2creset(struct i2c_dev_s *dev)
{
	struct tiva_i2c_inst_s *inst = (struct tiva_i2c_inst_s *)dev;
	struct tiva_i2c_priv_s *priv;
	unsigned int clock_count;
	unsigned int stretch_count;
	uint32_t scl_gpio;
	uint32_t sda_gpio;
	int ret = ERROR;

	DEBUGASSERT(inst && inst->priv && inst->priv->config);
	i2cvdbg("I2C%d:\n", inst->priv->config->devno);

	/* Get I2C private structure */

	priv = inst->priv;

	/* Our caller must own a ref */

	ASSERT(priv->refs > 0);

	/* Lock out other clients */

	tiva_i2c_sem_wait(dev);

	/* Un-initialize the port */

	tiva_i2c_uninitialize(priv);

	/* Use GPIO configuration to un-wedge the bus */

	scl_gpio = MKI2C_OUTPUT(priv->config->scl_pin);
	sda_gpio = MKI2C_OUTPUT(priv->config->sda_pin);

	tiva_configgpio(scl_gpio);
	tiva_configgpio(sda_gpio);

	/* Let SDA go high */

	tiva_gpiowrite(sda_gpio, 1);

	/* Clock the bus until any slaves currently driving it let it go. */

	clock_count = 0;
	while (!tiva_gpioread(sda_gpio)) {
		/* Give up if we have tried too hard */

		if (clock_count++ > 10) {
			goto out;
		}

		/* Sniff to make sure that clock stretching has finished.
		 *
		 * If the bus never relaxes, the reset has failed.
		 */

		stretch_count = 0;
		while (!tiva_gpioread(scl_gpio)) {
			/* Give up if we have tried too hard */

			if (stretch_count++ > 10) {
				goto out;
			}

			up_udelay(10);
		}

		/* Drive SCL low */

		tiva_gpiowrite(scl_gpio, 0);
		up_udelay(10);

		/* Drive SCL high again */

		tiva_gpiowrite(scl_gpio, 1);
		up_udelay(10);
	}

	/* Generate a start followed by a stop to reset slave
	 * state machines.
	 */

	tiva_gpiowrite(sda_gpio, 0);
	up_udelay(10);
	tiva_gpiowrite(scl_gpio, 0);
	up_udelay(10);
	tiva_gpiowrite(scl_gpio, 1);
	up_udelay(10);
	tiva_gpiowrite(sda_gpio, 1);
	up_udelay(10);

	/* Revert the GPIO configuration. */

	tiva_configgpio(MKI2C_INPUT(sda_gpio));
	tiva_configgpio(MKI2C_INPUT(scl_gpio));

	/* Re-init the port */

	tiva_i2c_initialize(priv, inst->frequency);
	ret = OK;

out:

	/* Release the port for re-use by other clients */

	tiva_i2c_sem_post(dev);
	return ret;
}
#endif							/* CONFIG_I2C_RESET */

#endif							/* CONFIG_TIVA_I2C0 ... CONFIG_TIVA_I2C9 */
