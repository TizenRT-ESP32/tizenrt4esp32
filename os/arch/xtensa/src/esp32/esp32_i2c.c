/****************************************************************************
 *
 * Copyright 2018 Samsung Electronics All Rights Reserved.
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
 *
 *   Copyright (C) 2014-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The Samsung sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2016 Samsung Electronics, Inc.
 *   All rights reserved.
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
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <tinyara/arch.h>
#include <tinyara/i2c.h>
#include <arch/board/board.h>

#include "chip/esp32_soc.h"
#include "chip/esp32_iomux.h"
#include "chip/esp32_gpio_sigmap.h"
#include "esp32_config.h"
#include "esp32_gpio.h"
#include "chip/periph_defs.h"

#include <arch/chip/i2c_struct.h>
#include "chip/esp32_i2c_reg.h"
#include "esp32_cpuint.h"
#include "esp32_gpio.h"
#include "esp32_i2c.h"
#include "rom/esp32_gpio.h"
#include "periph_ctrl.h"
#include "irq/irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define I2C_NUM_MAX         2

#define I2C_IO_INIT_LEVEL               (1)
#define ESP32_DEFAULT_I2CXFER_CLOCK     (100 * 1000)	/* 100Khz */
#define ESP32_DEFAULT_I2C_TIMEOUT		(100)
#define I2C_MASTER_TOUT_CNUM_DEFAULT    (8)

#define I2C_ESP32_BUFFER_SIZE           (32)

#define ESP32_I2C0_DEFAULT_SCLK_PIN         4
#define ESP32_I2C0_DEFAULT_SCLK_PULLUP_EN   0
#define ESP32_I2C0_DEFAULT_SDA_PIN          5
#define ESP32_I2C0_DEFAULT_SDA_PULLUP_EN    0

#define ESP32_I2C1_DEFAULT_SCLK_PIN         18
#define ESP32_I2C1_DEFAULT_SCLK_PULLUP_EN   0
#define ESP32_I2C1_DEFAULT_SDA_PIN          19
#define ESP32_I2C1_DEFAULT_SDA_PULLUP_EN    0

#ifndef min
#define min(a, b)       (((a) < (b)) ? (a) : (b))
#endif

#define BIT_MASK(n)     (BIT(n) - 1)

#define WAIT_ISR        1

#undef i2cinfo
#if 0
#define i2cinfo(format, ...)   printf(format, ##__VA_ARGS__)
#else
#define i2cinfo(format, ...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* I2C Device Private Data */
struct esp32_i2c_priv_s {
	const struct i2c_ops_s *ops;	/* Standard I2C operations */
	i2c_port_t i2c_num;
	const i2c_config_t *config;	/* Port configuration */

	sem_t sem_excl;				/* Mutual exclusion semaphore */
	sem_t sem_isr;

	uint8_t refs;				/* Reference count */

	uint8_t cpuint;				/* CPU interrupt */

	int xfer_speed;
	unsigned int slave_addr;
	unsigned int timeout;

	unsigned int initialized;
	unsigned int retries;
};

enum i2c_esp32_opcodes {
	I2C_ESP32_OP_RSTART = 0,
	I2C_ESP32_OP_WRITE,
	I2C_ESP32_OP_READ,
	I2C_ESP32_OP_STOP,
	I2C_ESP32_OP_END
};

struct i2c_esp32_cmd {
	uint32_t num_bytes: 8;
	uint32_t ack_en: 1;
	uint32_t ack_exp: 1;
	uint32_t ack_val: 1;
	uint32_t opcode: 3;
	uint32_t reserved: 17;
	uint32_t done: 1;
};
/****************************************************************************
 * Private Data
 ****************************************************************************/
static const i2c_config_t esp32_i2c0_config = {
	.periph = ESP32_PERIPH_I2C_EXT0,

#if defined(CONFIG_ESP32_I2C0_MODE_SLAVE) && (0 < CONFIG_ESP32_I2C0_MODE_SLAVE)
	.mode = I2C_MODE_SLAVE,
#else
	.mode = I2C_MODE_MASTER,
#endif

	.irq_num = ESP32_IRQ_I2C_EXT0,

#ifdef CONFIG_ESP32_I2C0_SCLK_PIN
	.scl_pin = CONFIG_ESP32_I2C0_SCLK_PIN,
#else
	.scl_pin = ESP32_I2C0_DEFAULT_SCLK_PIN,
#endif
#ifdef CONFIG_ESP32_I2C0_SCLK_PULLUP_EN
	.scl_pullup_en = CONFIG_ESP32_I2C0_SCLK_PULLUP_EN,
#else
	.scl_pullup_en = ESP32_I2C0_DEFAULT_SCLK_PULLUP_EN,
#endif
	.scl_in_sig = I2CEXT0_SCL_IN_IDX,
	.scl_out_sig = I2CEXT0_SCL_OUT_IDX,

#ifdef CONFIG_ESP32_I2C0_SDA_PIN
	.sda_pin = CONFIG_ESP32_I2C0_SDA_PIN,
#else
	.sda_pin = ESP32_I2C0_DEFAULT_SDA_PIN
#endif
#ifdef CONFIG_ESP32_I2C0_SDA_PULLUP_EN
	.sda_pullup_en = CONFIG_ESP32_I2C0_SDA_PULLUP_EN,
#else
	.sda_pullup_en = ESP32_I2C0_DEFAULT_SDA_PULLUP_EN,
#endif
	.sda_in_sig = I2CEXT0_SDA_IN_IDX,
	.sda_out_sig = I2CEXT0_SDA_OUT_IDX,
};

static struct esp32_i2c_priv_s esp32_i2c0_priv = {
	.i2c_num = 0,
	.xfer_speed = 0,
	.timeout = ESP32_DEFAULT_I2C_TIMEOUT,
	.initialized = 0,
	.retries = 3,
};

static const i2c_config_t esp32_i2c1_config = {
	.periph = ESP32_PERIPH_I2C_EXT1,
#if defined(CONFIG_ESP32_I2C1_MODE_SLAVE) && (0 < CONFIG_ESP32_I2C1_MODE_SLAVE)
	.mode = I2C_MODE_SLAVE,
#else
	.mode = I2C_MODE_MASTER,
#endif

	.irq_num = ESP32_IRQ_I2C_EXT1,
#ifdef CONFIG_ESP32_I2C1_SCLK_PIN
	.scl_pin = CONFIG_ESP32_I2C1_SCLK_PIN,
#else
	.scl_pin = ESP32_I2C1_DEFAULT_SCLK_PIN,
#endif
#ifdef CONFIG_ESP32_I2C1_SCLK_PULLUP_EN
	.scl_pullup_en = CONFIG_ESP32_I2C1_SCLK_PULLUP_EN,
#else
	.scl_pullup_en = ESP32_I2C1_DEFAULT_SCLK_PULLUP_EN,
#endif
	.scl_in_sig = I2CEXT1_SCL_IN_IDX,
	.scl_out_sig = I2CEXT1_SCL_OUT_IDX,

#ifdef CONFIG_ESP32_I2C1_SDA_PIN
	.sda_pin = CONFIG_ESP32_I2C1_SDA_PIN,
#else
	.sda_pin = ESP32_I2C1_DEFAULT_SDA_PIN,
#endif
#ifdef CONFIG_ESP32_I2C1_SDA_PULLUP_EN
	.scl_pullup_en = CONFIG_ESP32_I2C1_SDA_PULLUP_EN,
#else
	.scl_pullup_en = ESP32_I2C1_DEFAULT_SDA_PULLUP_EN,
#endif
	.sda_in_sig = I2CEXT1_SDA_IN_IDX,
	.sda_out_sig = I2CEXT1_SDA_OUT_IDX,
};

static struct esp32_i2c_priv_s esp32_i2c1_priv = {
	.i2c_num = 1,
	.xfer_speed = 0,
	.timeout = ESP32_DEFAULT_I2C_TIMEOUT,
	.initialized = 0,
	.retries = 3,
};

struct esp32_i2c_priv_s *g_esp32_i2c_priv[I2C_NUM_MAX] = { NULL, NULL };
static i2c_dev_t *const I2C[I2C_NUM_MAX] = { &I2C0, &I2C1 };

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int hsi2c_setup(struct esp32_i2c_priv_s *priv);
static int i2c_hw_enable(i2c_port_t i2c_num);
static int i2c_hw_disable(i2c_port_t i2c_num);
static void i2c_hw_setpins(const i2c_config_t *config);

uint32_t esp32_i2c_setclock(FAR struct i2c_dev_s *dev, uint32_t frequency);

static inline void i2c_esp32_reset_interrupts(i2c_port_t i2c_num);

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int i2c_esp32_isr(int irq, FAR void *context, FAR void *arg)
{
	struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)arg;

	const int fifo_give_mask = I2C_ACK_ERR_INT_ST | I2C_TIME_OUT_INT_ST | I2C_TRANS_COMPLETE_INT_ST | I2C_ARBITRATION_LOST_INT_ST;

	uint32_t status = I2C[priv->i2c_num]->int_status.val;
	if (status & fifo_give_mask) {

		/* Only give the semaphore if a watched interrupt happens.
		 * Error checking is performed at the other side of the
		 * semaphore, by reading the status register.
		 */
		sem_post(&priv->sem_isr);
	}

	/* Acknowledge all I2C interrupts */
	i2c_esp32_reset_interrupts(priv->i2c_num);
	return 0;
}

static void i2c_interrupts_enable(i2c_port_t i2c_num)
{
	uint32_t intr_mask = 0;
	intr_mask |= I2C_ARBITRATION_LOST_INT_ENA_M | I2C_TIME_OUT_INT_ST_M;
	intr_mask |= (I2C_TRANS_COMPLETE_INT_ENA_M | I2C_ACK_ERR_INT_ENA_M);
	I2C[i2c_num]->int_clr.val = intr_mask;
	I2C[i2c_num]->int_ena.val = intr_mask;
}

static void i2c_interrupts_disable(i2c_port_t i2c_num)
{
	I2C[i2c_num]->int_clr.val = 0xffffffff;
	I2C[i2c_num]->int_ena.val = 0;
}

static int hsi2c_setup(struct esp32_i2c_priv_s *priv)
{
	int i2c_num = priv->i2c_num;

	//configure ctrl register
	I2C[i2c_num]->ctr.rx_lsb_first = I2C_DATA_MODE_MSB_FIRST;	//set rx data msb first
	I2C[i2c_num]->ctr.tx_lsb_first = I2C_DATA_MODE_MSB_FIRST;	//set tx data msb first
	I2C[i2c_num]->ctr.ms_mode = priv->config->mode;	//mode for master or slave
	I2C[i2c_num]->ctr.sda_force_out = 1;	// set open-drain output mode
	I2C[i2c_num]->ctr.scl_force_out = 1;	// set open-drain output mode
	I2C[i2c_num]->ctr.sample_scl_level = 0;	//sample at high level of clock

	i2cinfo("[i2c] %d mode %d\n", i2c_num, I2C[i2c_num]->ctr.ms_mode);
	//configure speed
	esp32_i2c_setclock((FAR struct i2c_dev_s *)priv, ESP32_DEFAULT_I2CXFER_CLOCK);

	/* Use FIFO to transmit data */
	I2C[i2c_num]->fifo_conf.nonfifo_en = 0;
	i2c_interrupts_disable(i2c_num);

	/*register isr */
	priv->cpuint = esp32_alloc_levelint(1);
	if (priv->cpuint < 0) {
		/* Failed to allocate a CPU interrupt of this type */
		i2cinfo("[i2c] %d fail to alloc intr %d\n", i2c_num, priv->cpuint);
		return priv->cpuint;
	}

	/* Set up to receive peripheral interrupts on the current CPU */
	int cpu = 1;
#ifdef CONFIG_SMP
	cpu = up_cpu_index();
#else
	cpu = 0;
#endif

#if 0
	i2cinfo("I2c_%d_ISR: cpu=%d, periph=%d, cpuint=%d\n", i2c_num, cpu, priv->config->periph, priv->cpuint);
#endif
	/* Attach the GPIO peripheral to the allocated CPU interrupt */
	up_disable_irq(priv->cpuint);
	esp32_attach_peripheral(cpu, priv->config->periph, priv->cpuint);

	/* Attach and enable the IRQ */
	int ret = irq_attach(priv->config->irq_num, i2c_esp32_isr, priv);
	if (ret == OK) {
		/* Enable the CPU interrupt */
		up_enable_irq(priv->cpuint);
	}

	return OK;
}

void i2c_hw_setpins(const i2c_config_t *config)
{
	uint16_t mode = 0;

	mode = INPUT | OUTPUT | OPEN_DRAIN | FUNCTION_2;
	if (config->scl_pullup_en) {
		mode |= PULLUP;
	}
	esp32_gpiowrite(config->scl_pin, I2C_IO_INIT_LEVEL);
	esp32_configgpio(config->scl_pin, mode);
	gpio_matrix_in(config->scl_pin, config->scl_in_sig, 0);
	gpio_matrix_out(config->scl_pin, config->scl_out_sig, 0, 0);

	mode = INPUT | OUTPUT | OPEN_DRAIN | FUNCTION_2;
	if (config->sda_pullup_en) {
		mode |= PULLUP;
	}
	esp32_gpiowrite(config->sda_pin, I2C_IO_INIT_LEVEL);
	esp32_configgpio(config->sda_pin, mode);
	gpio_matrix_in(config->sda_pin, config->sda_in_sig, 0);
	gpio_matrix_out(config->sda_pin, config->sda_out_sig, 0, 0);
}

static int i2c_hw_enable(i2c_port_t i2c_num)
{
	if (i2c_num == I2C_NUM_0) {
		periph_module_enable(PERIPH_I2C0_MODULE);
	} else if (i2c_num == I2C_NUM_1) {
		periph_module_enable(PERIPH_I2C1_MODULE);
	}
	return OK;
}

static int i2c_hw_disable(i2c_port_t i2c_num)
{
	if (i2c_num == I2C_NUM_0) {
		periph_module_disable(PERIPH_I2C0_MODULE);
	} else if (i2c_num == I2C_NUM_1) {
		periph_module_disable(PERIPH_I2C1_MODULE);
	}
	return OK;
}

/* Some slave device will die by accident and keep the SDA in low level,
* in this case, master should send several clock to make the slave release the bus.
* Slave mode of ESP32 might also get in wrong state that held the SDA low,
* in this case, master device could send a stop signal to make esp32 slave release the bus.
**/
#if 0
static int i2c_hw_clear_bus(const i2c_config_t *config)
{
	i2c_hw_setpins(config);

	esp32_gpiowrite(config->scl_pin, 1);
	esp32_gpiowrite(config->sda_pin, 1);
	esp32_gpiowrite(config->sda_pin, 0);	//start
	for (int i = 0; i < 9; i++) {
		esp32_gpiowrite(config->scl_pin, 0);
		esp32_gpiowrite(config->scl_pin, 1);
	}
	esp32_gpiowrite(config->sda_pin, 1);
	return OK;
}
#endif

static inline void i2c_esp32_reset_txfifo(i2c_port_t i2c_num)
{
	/* Writing 1 and then 0 to these bits will reset the I2C fifo */
	I2C[i2c_num]->fifo_conf.tx_fifo_rst = 1;
	I2C[i2c_num]->fifo_conf.tx_fifo_rst = 0;
}

static inline void i2c_esp32_reset_rxfifo(i2c_port_t i2c_num)
{
	/* Writing 1 and then 0 to these bits will reset the I2C fifo */
	I2C[i2c_num]->fifo_conf.tx_fifo_rst = 1;
	I2C[i2c_num]->fifo_conf.tx_fifo_rst = 0;
}

static inline void i2c_esp32_reset_interrupts(i2c_port_t i2c_num)
{
	I2C[i2c_num]->int_clr.val = 0xFFFFFFFF;
}

static void i2c_esp32_clear_commands(i2c_port_t i2c_num)
{
	volatile struct i2c_esp32_cmd *cmd = (void *)I2C_COMD0_REG(i2c_num);
	for (int i = 0; i < 16; i++) {
		*cmd++ = (struct i2c_esp32_cmd) {
			0
		};
	}
}

void i2c_esp32_dump(struct esp32_i2c_priv_s *priv)
{
	i2cinfo("esp32_i2c_%d: \n", priv->i2c_num);
	i2cinfo("\tscl_low_period:  %08x\n", I2C[priv->i2c_num]->scl_low_period);
	i2cinfo("\tctr:  %08x;\n", I2C[priv->i2c_num]->ctr.val);
	i2cinfo("\tstatus : %08x;\n", I2C[priv->i2c_num]->status_reg.val);
	i2cinfo("\ttimeout : %08x\n", I2C[priv->i2c_num]->timeout);
	i2cinfo("\tslave_addr : %08x\n", I2C[priv->i2c_num]->slave_addr);
	i2cinfo("\tfifo_st : %08x\n", I2C[priv->i2c_num]->fifo_st);
	i2cinfo("\tfifo_conf : %08x\n", I2C[priv->i2c_num]->fifo_conf);
	//i2cinfo("\tfifo_data : %08x\n", I2C[priv->i2c_num]->fifo_data);
	i2cinfo("\tint_raw : %08x\n", I2C[priv->i2c_num]->int_raw);
	i2cinfo("\tint_ena : %08x\n", I2C[priv->i2c_num]->int_ena);
	i2cinfo("\tint_status : %08x\n", I2C[priv->i2c_num]->int_status);
	i2cinfo("\tsda_hold : %08x\n", I2C[priv->i2c_num]->sda_hold);
	i2cinfo("\tsda_sample : %08x\n", I2C[priv->i2c_num]->sda_sample);
	i2cinfo("\tscl_high_period : %08x\n", I2C[priv->i2c_num]->scl_high_period);
	i2cinfo("\tscl_start_hold : %08x\n", I2C[priv->i2c_num]->scl_start_hold);
	i2cinfo("\tscl_stop_hold : %08x\n", I2C[priv->i2c_num]->scl_stop_hold);
	i2cinfo("\tscl_filter_cfg : %08x\n", I2C[priv->i2c_num]->scl_filter_cfg);
	i2cinfo("\tsda_filter_cfg : %08x\n", I2C[priv->i2c_num]->sda_filter_cfg);
	for (int i = 0; i < 16; i++) {
		i2cinfo("\tCMD %d : n=%d, acken=%d, ackexp=%d,ackval=%d,opcode=%d, done=%d \n", i, I2C[priv->i2c_num]->command[i].byte_num, I2C[priv->i2c_num]->command[i].ack_en, I2C[priv->i2c_num]->command[i].ack_exp, I2C[priv->i2c_num]->command[i].ack_val, I2C[priv->i2c_num]->command[i].op_code, I2C[priv->i2c_num]->command[i].done);
	}
	i2cinfo("\tramdata[0] : %08x\n", I2C[priv->i2c_num]->ram_data[0]);
	i2cinfo("\tramdata[1] : %08x\n", I2C[priv->i2c_num]->ram_data[1]);
}

/*
 * Write slave address to fifo and update cmd pointer:
 *   if I2C_M_TEN, the slave address takes 2bytes;
 *   if read, add an I2C_ESP32_OP_WRITE command to write addr;
 */
static volatile struct i2c_esp32_cmd *i2c_esp32_write_addr(struct i2c_dev_s *dev, volatile struct i2c_esp32_cmd *cmd, struct i2c_msg_s *msg, uint16_t addr)
{
	uint32_t addr_len = 1;
	struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;

	i2c_esp32_reset_txfifo(priv->i2c_num);

	WRITE_PERI_REG(I2C_DATA_APB_REG(priv->i2c_num), (addr & I2C_FIFO_RDATA));

	if ((msg->flags & I2C_M_TEN) == I2C_M_TEN) {
		WRITE_PERI_REG(I2C_DATA_APB_REG(priv->i2c_num), ((addr >> 8) & I2C_FIFO_RDATA));
		addr_len++;
		i2cinfo("[i2c] addr %d is ten-bit!\n", addr);
	}
	if ((msg->flags & I2C_M_READ) == I2C_M_READ) {
		*cmd++ = (struct i2c_esp32_cmd) {
			.opcode = I2C_ESP32_OP_WRITE, .ack_en = true, .num_bytes = addr_len,
		};
	} else {
		//msg->length += addr_len;
	}

	return cmd;
}

static int i2c_esp32_wait(struct i2c_dev_s *dev, volatile struct i2c_esp32_cmd *wait_cmd)
{
	struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;
#if WAIT_ISR
	int count = 0;
	/*wait for cmd done */
	if (wait_cmd) {
		while (!wait_cmd->done) {
			usleep(10000);
			if (count++ > 100) {
				i2cinfo("[i2c drv] wait_cmd timeout!\n");
				return -ETIMEDOUT;
			}
		}
	}

	/* Wait for I2C bus to finish its business */
	count = 0;
	while (I2C[priv->i2c_num]->status_reg.bus_busy) {
		usleep(10000);
		if (count++ > 100) {
			i2cinfo("[i2c drv] status_reg.bus_busy timeout!\n");
			return -ETIMEDOUT;
		}
	}
#else
	int time = 0;
	do {
		if ((wait_cmd && wait_cmd->done) && !I2C[priv->i2c_num]->status_reg.bus_busy) {
			break;
		}
		usleep(10);
		time += 10;
		if (time > priv->timeout) {
			return -ETIMEDOUT;
		}
	} while (!I2C[priv->i2c_num]->int_raw.arbitration_lost && !I2C[priv->i2c_num]->int_raw.time_out && !I2C[priv->i2c_num]->int_raw.ack_err);	///
#endif

	return 0;
}

#define NS_COUNT_IN_MS      (1000000)
#define NS_COUNT_IN_S       (1000000000)
static int i2c_esp32_transmit(struct i2c_dev_s *dev)
{
	int ret = 0;
	uint32_t status;
	struct timespec abstime = { 0 };
	/* Start transmission and wait for the ISR to give the semaphore */
	struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;

	I2C[priv->i2c_num]->ctr.trans_start = 1;

#if WAIT_ISR
	(void)clock_gettime(CLOCK_REALTIME, &abstime);
	abstime.tv_nsec += priv->timeout * NS_COUNT_IN_MS;
	if (abstime.tv_nsec >= NS_COUNT_IN_S) {
		abstime.tv_nsec %= NS_COUNT_IN_S;
		abstime.tv_sec++;
	}
	ret = sem_timedwait(&priv->sem_isr, &abstime);
	if (ret < 0) {
		ret = get_errno();
		return -ETIMEDOUT;
	}

	status = I2C[priv->i2c_num]->int_raw.val;
	if (status & (I2C_ARBITRATION_LOST_INT_RAW | I2C_ACK_ERR_INT_RAW)) {
		return -EIO;
	}
	if (status & I2C_TIME_OUT_INT_RAW) {
		return -ETIMEDOUT;
	}
#endif

	return ret;
}

static int i2c_esp32_transmit_wait(struct i2c_dev_s *dev, volatile struct i2c_esp32_cmd *wait_cmd)
{
	int ret;
	struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;

	i2c_interrupts_enable(priv->i2c_num);
	ret = i2c_esp32_transmit(dev);
	if (!ret) {
		ret = i2c_esp32_wait(dev, wait_cmd);
	}
	i2c_interrupts_disable(priv->i2c_num);

	return ret;
}

static int i2c_esp32_read_msg(struct i2c_dev_s *dev, uint16_t addr, struct i2c_msg_s *msg)
{
	struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;

	volatile struct i2c_esp32_cmd *cmd = (void *)I2C_COMD0_REG(priv->i2c_num);
	uint32_t i;
	int ret;
	uint32_t read_count = 0;

	/* Set the R/W bit to R */
	addr |= BIT(0);

	/* I2C_ESP32_OP_RSTART must be the first one;
	 * if I2C_M_NOSTART, it means the i2c-frame is too long to be handled in one msg!
	 */
	if ((msg->flags & I2C_M_NOSTART) != I2C_M_NOSTART) {
		*cmd++ = (struct i2c_esp32_cmd) {
			.opcode = I2C_ESP32_OP_RSTART
		};
	}

	cmd = i2c_esp32_write_addr(dev, cmd, msg, addr);
	for (; msg->length; cmd = (void *)I2C_COMD0_REG(priv->i2c_num)) {
		volatile struct i2c_esp32_cmd *wait_cmd = NULL;
		uint32_t to_read = min(I2C_ESP32_BUFFER_SIZE, msg->length - 1);

		/* Might be the last byte, in which case, `to_read` will
		 * be 0 here.  See comment below.
		 */
		if (to_read) {
			*cmd++ = (struct i2c_esp32_cmd) {
				.opcode = I2C_ESP32_OP_READ, .num_bytes = to_read,
			};
		}
		/* I2C master won't acknowledge the last byte read from the
		 * slave device.  Divide the read command in two segments as
		 * recommended by the ESP32 Technical Reference Manual.
		 */
		if (msg->length - to_read <= 1) {
			/* Read the last byte and explicitly ask for an
			 * acknowledgment.
			 */
			*cmd++ = (struct i2c_esp32_cmd) {
				.opcode = I2C_ESP32_OP_READ, .num_bytes = 1, .ack_val = true,
			};

			/* Account for the `msg.len - 1` when clamping
			 * transmission length to FIFO buffer size.
			 */
			to_read++;
			/*No more msg to excute */
			if (I2C_M_NORESTART != (msg->flags & I2C_M_NORESTART)) {
				wait_cmd = cmd;
				*cmd++ = (struct i2c_esp32_cmd) {
					.opcode = I2C_ESP32_OP_STOP
				};
			}
		}
		if (!wait_cmd) {
			*cmd++ = (struct i2c_esp32_cmd) {
				.opcode = I2C_ESP32_OP_END
			};
		}

		ret = i2c_esp32_transmit_wait(dev, wait_cmd);
		if (ret < 0) {
			return ret;
		}
		for (i = 0; i < to_read; i++) {
			uint32_t v = I2C[priv->i2c_num]->fifo_data.val;
			v = I2C_FIFO_RDATA & v;
			*msg->buffer++ = v;
		}
		msg->length -= to_read;
		read_count += to_read;

		i2c_esp32_reset_txfifo(priv->i2c_num);
	}

	return read_count;
}

static int i2c_esp32_write_msg(struct i2c_dev_s *dev, uint16_t addr, struct i2c_msg_s *msg)
{
	struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;
	volatile struct i2c_esp32_cmd *cmd = (void *)I2C_COMD0_REG(priv->i2c_num);
	int addrSent = 0;
	uint32_t send_count = 0;

	/* I2C_ESP32_OP_RSTART must be the first one;
	 * if I2C_M_NOSTART, it means the i2c-frame is too long to be handled in one msg!
	 */
	if ((msg->flags & I2C_M_NOSTART) != I2C_M_NOSTART) {
		*cmd++ = (struct i2c_esp32_cmd) {
			.opcode = I2C_ESP32_OP_RSTART
		};
	}
	cmd = i2c_esp32_write_addr(dev, cmd, msg, addr);
	for (; msg->length; cmd = (void *)I2C_COMD0_REG(priv->i2c_num)) {
		uint32_t i = 1;
		int ret;
		uint32_t to_send = 0;	//data len

		/* Get datalength in this command */
		if (addrSent) {
			ret = 0;
		} else {
			ret = 1;			//address Length
			if ((msg->flags & I2C_M_TEN) == I2C_M_TEN) {
				ret = 2;
			}
			addrSent = 0;
		}
		to_send = min(I2C_ESP32_BUFFER_SIZE - ret, msg->length);
		/* Copy data to TX fifo */
		for (i = 0; i < to_send; i++) {
			WRITE_PERI_REG(I2C_DATA_APB_REG(priv->i2c_num), *msg->buffer++);
		}

		/* set WRITE command */
		*cmd++ = (struct i2c_esp32_cmd) {
			.opcode = I2C_ESP32_OP_WRITE, .num_bytes = to_send + ret,	//total len
			 .ack_en = true,
		};

		send_count =+ to_send;
		msg->length -= to_send;

		/*stop when end of msgdata and no more msg to excute */
		if (!msg->length && I2C_M_NORESTART != (msg->flags & I2C_M_NORESTART)) {
			*cmd = (struct i2c_esp32_cmd) {
				.opcode = I2C_ESP32_OP_STOP
			};
		} else {

			*cmd = (struct i2c_esp32_cmd) {
				.opcode = I2C_ESP32_OP_END
			};
		}

		ret = i2c_esp32_transmit_wait(dev, cmd);
		if (ret < 0) {
			return send_count;
		}

		i2c_esp32_reset_txfifo(priv->i2c_num);
	}

	return send_count;
}

static int esp32_i2c_initialize(struct esp32_i2c_priv_s *priv)
{
	/* Configure GPIO pins */
	i2c_hw_setpins(priv->config);

	/* Enable I2C modlue */
	i2c_hw_enable(priv->i2c_num);

	/* Chip level configure */
	int ret = hsi2c_setup(priv);
	if (ret != OK) {
		i2c_hw_disable(priv->i2c_num);
		return ERROR;
	}
	//end of initallize!!
	priv->initialized = 1;

	return OK;
}

static int esp32_i2c_uninitialize(struct esp32_i2c_priv_s *priv)
{
	/* Disable I2C */
	i2c_hw_disable(priv->i2c_num);

	I2C[priv->i2c_num]->ctr.trans_start = 0;
	i2c_esp32_reset_txfifo(priv->i2c_num);
	i2c_esp32_reset_rxfifo(priv->i2c_num);
	i2c_esp32_reset_interrupts(priv->i2c_num);
	i2c_esp32_clear_commands(priv->i2c_num);

	I2C[priv->i2c_num]->int_ena.val = 0;

	//end of initallize!!
	priv->initialized = 0;

	return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief    Setting Slave Address for slave mode
 * @param    struct i2c_dev_s *dev : pointer to i2c_dev_s
 * @param    int addr : Slave address
 * @param    int nbits : 0==7bit
 *           1==10bit
 * @return    = 0
 * @note
 */
int esp32_i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits)
{
	struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;
	if (priv == NULL) {
		i2cinfo("i2c dev is NULL\n");
		return ERROR;
	}
	sem_wait(&priv->sem_excl);
	if (addr > 0) {
		priv->slave_addr = addr;
	}
	if (nbits > 0 && priv->config != NULL) {
		;
	}
	sem_post(&priv->sem_excl);
	return OK;
}

/**
* @brief    i2c clock setting
* @param    struct i2c_dev_s *dev : pointer to i2c_dev_s
* @param    unsigned int frequency : ex ) 100000 == 100khz
* @return   void
* @note
*/
uint32_t esp32_i2c_setclock(FAR struct i2c_dev_s *dev, uint32_t frequency)
{
	struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;
	int i2c_num = priv->i2c_num;

	if (priv != NULL && priv->xfer_speed != frequency) {
		sem_wait(&priv->sem_excl);

		priv->xfer_speed = frequency;
		int cycle = (I2C_APB_CLK_FREQ / frequency);
		int half_cycle = cycle / 2;
		I2C[i2c_num]->timeout.tout = cycle * I2C_MASTER_TOUT_CNUM_DEFAULT;
		//set timing for data
		I2C[i2c_num]->sda_hold.time = half_cycle / 2;
		I2C[i2c_num]->sda_sample.time = half_cycle / 2;

		I2C[i2c_num]->scl_low_period.period = half_cycle;
		I2C[i2c_num]->scl_high_period.period = half_cycle;
		//set timing for start signal
		I2C[i2c_num]->scl_start_hold.time = half_cycle;
		I2C[i2c_num]->scl_rstart_setup.time = half_cycle;
		//set timing for stop signal
		I2C[i2c_num]->scl_stop_hold.time = half_cycle;
		I2C[i2c_num]->scl_stop_setup.time = half_cycle;

		sem_post(&priv->sem_excl);
	}
	return OK;
}

/**
 * @brief    Generic I2C transfer function
 * @param    struct i2c_master_s *dev : structure visible to the I2C client
 * @param    struct i2c_msg_s *msgv : I2C transaction segment
 * @param    int msgc :  length
 * @return   int : ==0 :OK
 * @note
 */
int esp32_i2c_transfer(struct i2c_dev_s *dev, struct i2c_msg_s *msgs, int msgc)
{
	struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;
	int ret = 0;
	uint8_t i = 0;
	uint16_t addr = msgs->addr;;

	/* Ensure that address or flags don't change meanwhile */
	sem_wait(&priv->sem_excl);

	I2C[priv->i2c_num]->ctr.trans_start = 0;
	i2c_esp32_reset_txfifo(priv->i2c_num);
	i2c_esp32_reset_rxfifo(priv->i2c_num);
	i2c_esp32_reset_interrupts(priv->i2c_num);
	i2c_esp32_clear_commands(priv->i2c_num);

	/* Mask out unused address bits, and make room for R/W bit */
	i = 0;
	addr = msgs[i].addr;
	if ((msgs[i].flags & I2C_M_TEN) != I2C_M_TEN) {
		addr &= BIT_MASK(7);
	} else {
		addr &= BIT_MASK(10);
	}
	addr <<= 1;

	for (i = 0; i < msgc; i++) {

		if ((msgs[i].flags & I2C_M_READ) != I2C_M_READ) {
			ret = i2c_esp32_write_msg(dev, addr, &msgs[i]);
		} else {
			ret = i2c_esp32_read_msg(dev, addr, &msgs[i]);
		}

		if (ret < 0) {
			break;
		}
	}

	/* Ensure that address or flags don't change meanwhile */
	sem_post(&priv->sem_excl);
	return ret;
}

int esp32_i2c_read(FAR struct i2c_dev_s *dev, FAR uint8_t *buffer, int buflen)
{
	struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;
	struct i2c_msg_s msg;
	unsigned int flags;

	flags = 0;

	/* Setup for the transfer */
	msg.addr = priv->slave_addr;
	msg.flags = (flags | I2C_M_READ);
	msg.buffer = (FAR uint8_t *) buffer;
	msg.length = buflen;

	/*
	 * Then perform the transfer
	 *
	 * REVISIT:  The following two operations must become atomic in order to
	 * assure thread safety.
	 */

	return esp32_i2c_transfer(dev, &msg, 1);
}

int esp32_i2c_write(FAR struct i2c_dev_s *dev, FAR const uint8_t *buffer, int buflen)
{
	struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;
	struct i2c_msg_s msg;

	/* Setup for the transfer */
	msg.addr = priv->slave_addr;
	msg.flags = 0;
	msg.buffer = (FAR uint8_t *) buffer;	/* Override const */
	msg.length = buflen;

	/*
	 * Then perform the transfer
	 *
	 * REVISIT:  The following two operations must become atomic in order to
	 * assure thread safety.
	 */
	return esp32_i2c_transfer(dev, &msg, 1);
}

/* I2C Interface */
static const struct i2c_ops_s esp32_i2c_ops = {
	.setfrequency = esp32_i2c_setclock,
	.setaddress = esp32_i2c_setaddress,
	.write = esp32_i2c_write,
	.read = esp32_i2c_read,
	.transfer = esp32_i2c_transfer,
};

/**
 * @brief   Initialize one I2C bus
 * @param   int port :
 * @return  struct i2c_master_s : device structure
 * @note
 */
struct i2c_dev_s *up_i2cinitialize(int port)
{
	struct esp32_i2c_priv_s *priv = NULL;
	const i2c_config_t *config;

	/* check if port is valid */
	if (port < 0 || port >= I2C_NUM_MAX) {
		return NULL;
	}

	/* Get I2C private structure */
	if (g_esp32_i2c_priv[port] != NULL) {
		priv = g_esp32_i2c_priv[port];
		priv->refs++;
		i2cinfo("I2C port %d already initiallized: %d\n", port, priv->refs);
		return (FAR struct i2c_dev_s *)priv;
	}

	switch (port) {
	case 0:
		priv = &esp32_i2c0_priv;
		config = &esp32_i2c0_config;
		break;

	case 1:
		priv = &esp32_i2c1_priv;
		config = &esp32_i2c1_config;
		break;

	default:
		i2cinfo("Unsupport I2C port %d\n");
		return NULL;
	}

	priv->i2c_num = port;

	/* Initialize private device structure */
	priv->ops = &esp32_i2c_ops;

	/*
	 * Initialize private data for the first time, increment reference count,
	 * power-up hardware and configure GPIOs.
	 */
	priv->refs++;

	if (priv->refs == 1) {
		/* Initialize the device structure */
		priv->config = config;
		sem_init(&priv->sem_excl, 0, 1);
		sem_init(&priv->sem_isr, 0, 0);
		sem_setprotocol(&priv->sem_isr, SEM_PRIO_NONE);

		/* Initialize the I2C hardware */
		esp32_i2c_initialize(priv);
	}

	g_esp32_i2c_priv[port] = priv;

	return (FAR struct i2c_dev_s *)priv;
}

/**
 * @brief    Unitialize one I2C bus
 * @param    struct i2c_master_s *dev :
 * @return   ==0 : OK
 * @note
 */
int up_i2cuninitialize(FAR struct i2c_dev_s *dev)
{
	struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;
	int flags;

	DEBUGASSERT(priv && priv->config && priv->refs > 0);

	/* Decrement reference count and check for underflow */
	flags = irqsave();

	/* Check if the reference count will decrement to zero */
	if (priv->refs < 2) {
		/* Yes.. Disable power and other HW resource (GPIO's) */
		esp32_i2c_uninitialize(priv);
		priv->refs = 0;

		/* Release unused resources */
		sem_destroy(&priv->sem_excl);

		sem_destroy(&priv->sem_isr);	//

	} else {
		/* No.. just decrement the number of references to the device */
		priv->refs--;
	}

	irqrestore(flags);
	return OK;
}

/**
 * @brief   Register I2C bus into file system as I2C character driver
 * @param   I2C bus number :
 * @return  none
 * @note
 */
void esp32_i2c_register(int bus)
{
	FAR struct i2c_dev_s *i2c;
	char path[16];

	i2c = up_i2cinitialize(bus);
	if (i2c != NULL) {
#ifdef CONFIG_I2C_USERIO
		snprintf(path, 16, "/dev/i2c-%d", bus);
		if (i2c_uioregister(path, i2c) < 0) {
			up_i2cuninitialize(i2c);
		}
#endif
	}
}

/*******************************************The end of file****************************************/
