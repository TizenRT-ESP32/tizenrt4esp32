/******************************************************************
 *
 * Copyright 2018 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <stdlib.h>
#include <apps/shell/tash.h>

#include <tinyara/fs/fs.h>
#include <tinyara/fs/ioctl.h>
#include <tinyara/kmalloc.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

#if defined(CONFIG_SPI)
#include "../../os/arch/xtensa/src/esp32/esp32_spi.h"
#include <tinyara/spi/spi.h>
#include "w25q128.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/
#define SPI_TEST_PORT_NUM   2

#if SPI_TEST_PORT_NUM==2
#if defined(CONFIG_SPI2_DMA_CHANNEL) && 0 < CONFIG_SPI2_DMA_CHANNEL
#define MAX_DATALEN_PER_FRAME  4076
#else
#define MAX_DATALEN_PER_FRAME  60
#endif
#elif SPI_TEST_PORT_NUM==3
#if defined(CONFIG_SPI3_DMA_CHANNEL) && 0 < CONFIG_SPI3_DMA_CHANNEL
#define MAX_DATALEN_PER_FRAME  4076
#else
#define MAX_DATALEN_PER_FRAME  60
#endif
#else
#error "Unsupport SPI port!"
#endif

#define MAX_BUFFER_LEN  (4 + MAX_DATALEN_PER_FRAME + 4)

#define BUFFER_ALIGNED_MASK         0x03
#define BUFFER_ALIGNED_ADDR(buf)    (((uint32_t)buf + BUFFER_ALIGNED_MASK) & (~BUFFER_ALIGNED_MASK))

#define MHZ(n) ((n)*1000*1000)

#define RUNTIME_MS(st,ed) ((ed.tv_sec - st.tv_sec) * 1000 + (ed.tv_nsec - st.tv_nsec) / 1000000)

/**/
#define DEBUG_W25Q128   0
/****************************************************************************
 * Private Data
 ****************************************************************************/
const int spi_port = SPI_TEST_PORT_NUM;
const int spi_freq = MHZ(20);
//const int spi_freq = MHZ(16);
const int spi_bits = 8;
const int spi_conf = 0;
const int spi_w25q128 = 0;

const int w25q_test_addr = 0x008000;
const int w25q_test_len = 4096;

struct spi_dev_s *spi_dev = NULL;

unsigned char spi_buf[MAX_BUFFER_LEN];

struct timespec starttime = { 0 };
struct timespec endtime = { 0 };

/****************************************************************************
* Private Functions
****************************************************************************/
int getInt(char *arg, int *value)
{
	if (arg == NULL) {
		return ERROR;
	}

	int ret = strlen(arg);
	*value = 0;
	for (int i = 0; i < ret; i++) {
		arg[i] = tolower(arg[i]);
	}
	if (arg[0] == '0' && arg[1] == 'x') {
		sscanf(arg, "0x%x", value);
		ret = 16;
	} else {
		sscanf(arg, "%d", value);
		ret = 10;
	}
	return ret;
}

int w25q128_open_port(void)
{
	spi_dev = up_spiinitialize(spi_port);
	if (spi_dev == NULL) {
		printf("Open SPI%d fails!\n", spi_port);
		return -1;
	}

	SPI_SETFREQUENCY(spi_dev, spi_freq);
	SPI_SETBITS(spi_dev, spi_bits);
	SPI_SETMODE(spi_dev, spi_conf);

	return OK;
}

int w25q128_read_deviceID(unsigned char *buf)
{
	unsigned int ret = 0;
	unsigned char rd_buf[8] = { 0 };
	rd_buf[0] = READ_DEVID_CMD;
	rd_buf[1] = 0;
	rd_buf[2] = 0;
	rd_buf[3] = 0;

	if (spi_dev != NULL && buf != NULL) {
		SPI_LOCK(spi_dev, true);

		SPI_SELECT(spi_dev, spi_w25q128, true);
		SPI_EXCHANGE(spi_dev, rd_buf, rd_buf, 6);
		SPI_SELECT(spi_dev, spi_w25q128, false);

		SPI_LOCK(spi_dev, false);
	}

	buf[0] = rd_buf[4];
	buf[1] = rd_buf[5];
	ret = (rd_buf[4] * 256 + rd_buf[5]);
#if DEBUG_W25Q128
	printf("id %p:  ", rd_buf);
	for (int i = 0; i < 8; i++) {
		printf("%02x  ", rd_buf[i]);
	}
	printf("\n\n");
#endif
	return ret;
}

int w25q128_get_status(uint8_t *status)
{
	int ret = ERROR;
	if (spi_dev != NULL && NULL != status) {
		uint8_t cmd[4] = { READ_STATUS_REG1_CMD, 0 };
		SPI_LOCK(spi_dev, true);

		SPI_SELECT(spi_dev, spi_w25q128, true);
		SPI_EXCHANGE(spi_dev, cmd, cmd, 4);
		SPI_SELECT(spi_dev, spi_w25q128, false);

		SPI_LOCK(spi_dev, false);

		*status = cmd[1];
#if DEBUG_W25Q128
	    printf("[w25q128] status: %02x %02x %02x\n", cmd[1], cmd[2], cmd[3]);
#endif
		ret = OK;
	}
	return ret;
}

int w25q128_enable_write(char enable)
{
	int ret = ERROR;

	if (spi_dev != NULL) {
		uint8_t *cmd = (uint8_t *) BUFFER_ALIGNED_ADDR(spi_buf);
		memset(cmd, 0, 4);
		if (enable > 0) {
			cmd[0] = WRITE_ENABLE_CMD;
		} else {
			cmd[0] = WRITE_DISABLE_CMD;
		}

		SPI_LOCK(spi_dev, true);
		SPI_SELECT(spi_dev, spi_w25q128, true);
		SPI_SNDBLOCK(spi_dev, cmd, 1);
		SPI_SELECT(spi_dev, spi_w25q128, false);
		SPI_LOCK(spi_dev, false);

		w25q128_get_status(cmd);
		if (cmd[0] & W25Q128FV_FSR_WREN) {
			ret = OK;
		}
	}

	return ret;
}

int w25q128_reset(void)
{
	int ret = ERROR;

	if (spi_dev != NULL) {
		uint8_t *cmd = (uint8_t *) BUFFER_ALIGNED_ADDR(spi_buf);
		cmd[0] = RESET_ENABLE_CMD;
		cmd[1] = RESET_MEMORY_CMD;

		SPI_LOCK(spi_dev, true);

		SPI_SELECT(spi_dev, spi_w25q128, true);
		SPI_SNDBLOCK(spi_dev, cmd, 2);
		SPI_SELECT(spi_dev, spi_w25q128, false);

		SPI_LOCK(spi_dev, false);

		ret = OK;
	}
	return ret;
}

int w25q128_erase_block(uint32_t Address, w25q128_Sector_e sector)
{
	int ret = ERROR;
	if (spi_dev != NULL) {
		int ticks = 0;
		uint8_t cmd[4];
		uint32_t block_addr;
		switch (sector) {
		case SECTOR:
			cmd[0] = SECTOR_ERASE_CMD;
			block_addr = W25Q128FV_SUBSECTOR_MASK & Address;;
			break;
		case SMALLBLOCK:
			cmd[0] = SMALL_BLOCK_ERASE_CMD;
			block_addr = W25Q128FV_HALFSECTOR_MASK & Address;;
			break;
		case BIGBLOCK:
			cmd[0] = BIG_BLOCK_ERASE_CMD;
			block_addr = W25Q128FV_SECTOR_MASK & Address;;
			break;
		default:
			return ERROR;
		}
		cmd[1] = (uint8_t)(block_addr >> 16);
		cmd[2] = (uint8_t)(block_addr >> 8);
		cmd[3] = (uint8_t)(block_addr);

#if DEBUG_W25Q128
		printf("[w25q128] erase %d 0x%02x, 0x%08x 0x%08x\n", sector, cmd[0], Address, block_addr);
#endif
		/* Enable write operations */
		w25q128_enable_write(1);

		SPI_LOCK(spi_dev, true);
		SPI_SELECT(spi_dev, spi_w25q128, true);
		SPI_SNDBLOCK(spi_dev, cmd, 4);
		SPI_SELECT(spi_dev, spi_w25q128, false);
		SPI_LOCK(spi_dev, false);

		/* wait erase end */
		do {
			ret = w25q128_get_status(cmd);
			if (ret == OK && 0 == (cmd[0] & W25Q128FV_FSR_BUSY)) {
				break;
			}
			usleep(1000);		//10ms
			if (++ticks > 400) {	//Max 400ms
				return ERROR;
			}
		} while (1);
#if DEBUG_W25Q128
		printf("[w25q128] erase %08x %08x: %d\n", Address, block_addr, ticks);
#endif
	}
	return OK;
}

int w25q128_Read(uint32_t ReadAddr, uint8_t *pData, uint32_t Size)
{
	int ret = ERROR;
	if (spi_dev != NULL && pData != NULL) {
		uint8_t *cmd = (uint8_t *) BUFFER_ALIGNED_ADDR(spi_buf);
		int readlen = 0;
		int len = 0;
		uint32_t addr = ReadAddr;

		SPI_LOCK(spi_dev, true);
		do {
			if (Size - readlen > MAX_DATALEN_PER_FRAME) {
				len = MAX_DATALEN_PER_FRAME;
			} else {
				len = Size - readlen;
			}
			memset(cmd, 0, len + 4);
			/* Configure the command */
			cmd[0] = READ_CMD;
			cmd[1] = (uint8_t)(addr >> 16);
			cmd[2] = (uint8_t)(addr >> 8);
			cmd[3] = (uint8_t)(addr);
#if DEBUG_W25Q128
			printf("[w25q128] READ_CMD %08x %d\n  ", addr, len);
#endif
			SPI_SELECT(spi_dev, spi_w25q128, true);
			SPI_EXCHANGE(spi_dev, cmd, cmd, 4 + len);
			SPI_SELECT(spi_dev, spi_w25q128, false);

			memcpy(pData + readlen, cmd + 4, len);
			addr += len;
			readlen += len;
		} while (readlen < Size);

		SPI_LOCK(spi_dev, false);
		ret = OK;
	}
	return ret;
}

int w25q128_Write(uint32_t WriteAddr, uint8_t *pData, uint32_t Size)
{
	if (spi_dev == NULL || pData == NULL) {
		return ERROR;
	}

	uint8_t *cmd = (uint8_t *) BUFFER_ALIGNED_ADDR(spi_buf);
	uint32_t end_addr, current_size, current_addr;
	uint8_t status = 0;

	/* Calculation of the size between the write address and the end of the page */
	current_addr = 0;

	while (current_addr <= WriteAddr) {
		current_addr += W25Q128FV_PAGE_SIZE;
	}
	current_size = current_addr - WriteAddr;

	/* Check if the size of the data is less than the remaining place in the page */
	if (current_size > Size) {
		current_size = Size;
	}

	/* Initialize the adress variables */
	current_addr = WriteAddr;
	end_addr = WriteAddr + Size;
#if DEBUG_W25Q128
	w25q128_get_status(&status);
	printf("[w25q]  %02x write %08x: %d %d\n", status, WriteAddr, current_size, Size);
#endif

	int wrlen;
	int len;
	uint32_t addr;
	/* Perform the write page by page */
	do {
		addr = current_addr;
		wrlen = 0;
		do {
			if (current_size - wrlen > MAX_DATALEN_PER_FRAME) {
				len = MAX_DATALEN_PER_FRAME;
			} else {
				len = current_size - wrlen;
			}
			if (len > W25Q128FV_PAGE_SIZE) {
				len = W25Q128FV_PAGE_SIZE;
			}

			/* Enable write operations: befor send PAGE_PROG_CMD */
			w25q128_enable_write(1);

			/* Configure the command */
			cmd[0] = PAGE_PROG_CMD;
			cmd[1] = (uint8_t)(addr >> 16);
			cmd[2] = (uint8_t)(addr >> 8);
			cmd[3] = (uint8_t)(addr);
			memcpy(cmd + 4, pData + wrlen, len);
#if DEBUG_W25Q128
			printf("[w25q] wr %08x %d %d:...\n  ", addr, len, wrlen);
#endif
			SPI_LOCK(spi_dev, true);
			SPI_SELECT(spi_dev, spi_w25q128, true);
			SPI_SNDBLOCK(spi_dev, cmd, 4 + len);
			SPI_SELECT(spi_dev, spi_w25q128, false);
			SPI_LOCK(spi_dev, false);

			wrlen += len;
			addr += len;

			/* Wait the end of Flash writing */
			len = 0;
			do {
				int ret = w25q128_get_status(&status);
				if (ret == OK && 0 == (status & W25Q128FV_FSR_BUSY)) {
					break;
				}
				/* Check for the Timeout */
				usleep(100);	//0.1ms
				if (++len > 32) {	//Max 3ms
					printf("[w25q128] Writesend timeout\n");
					return ERROR;
				}
			} while (1);
		} while (wrlen < current_size);

		/* Update the address and size variables for next page programming */
		current_addr += current_size;
		pData += current_size;
		current_size = ((current_addr + W25Q128FV_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : W25Q128FV_PAGE_SIZE;
	} while (current_addr < end_addr);

	return OK;
}

int w25q128_program_block(uint32_t WriteAddr, uint8_t *pData)
{
	int block_addr = WriteAddr & W25Q128FV_PAGE_MASK;
	if (0 == (WriteAddr & W25Q128FV_SUBSECTOR_ALIGN)) {
		w25q128_erase_block(block_addr, SECTOR);
#if DEBUG_W25Q128
		printf("\nw25q128 progm %d, %d :%p\n", WriteAddr, block_addr, pData);
#endif
	}
	w25q128_Write(block_addr, pData, W25Q128FV_PAGE_SIZE);

	return OK;
}

/****************************************************************************
* Demo Functions
****************************************************************************/
static int esp32_spi_start_demo(int argc, char **args)
{
	printf("[w25q128] Initiallize SPI%d %dHZ...\n", spi_port, spi_freq);
	fflush(stdout);
	usleep(2000);
	if (0 < w25q128_open_port()) {
		return ERROR;
	}
	unsigned char spi_data_buf[4] = { 0 };
	unsigned int id = w25q128_read_deviceID(spi_data_buf);
	w25q128_get_status(spi_data_buf + 2);
	printf("[w25q128] devid: %x; buf: %02x %02x; status: %02x \n", id, spi_data_buf[0], spi_data_buf[1], spi_data_buf[2]);

	return OK;
}

static int esp32_spi_write_demo(int argc, char **args)
{
	int write_flag = 0;
	if (argc >= 2) {
		getInt(args[1], &write_flag);
		write_flag &= 0x03;
		printf("[w25q128] %s, %d\n", args[1], write_flag);
	}
	unsigned char *spi_data_buf = (unsigned char *)zalloc(w25q_test_len);
	if (spi_data_buf == NULL) {
		printf("[w25q128] alloc %dB, fails\n", w25q_test_len);
		return -ERROR;
	}
	int ret;
	printf("[w25q128] eraseblock:\n");
	(void)clock_gettime(CLOCK_REALTIME, &starttime);
	ret = w25q128_erase_block(w25q_test_addr, SECTOR);
	(void)clock_gettime(CLOCK_REALTIME, &endtime);
	printf("[w25q128] eraseblock %d; T:%d\n", ret, RUNTIME_MS(starttime, endtime));

#if DEBUG_W25Q128
	memset(spi_data_buf, 0, w25q_test_len);
	usleep(5000);
	printf("[w25q128] start read...\n");
	(void)clock_gettime(CLOCK_REALTIME, &starttime);
	ret = w25q128_Read(w25q_test_addr, spi_data_buf, w25q_test_len);
	(void)clock_gettime(CLOCK_REALTIME, &endtime);
	printf("read %08x %dB ret=%d T:%d\n", w25q_test_addr, w25q_test_len, ret, RUNTIME_MS(starttime, endtime));
	for (int i = 0; i < w25q_test_len; i++) {
		if (0 == (i & 0x0f)) {
			printf("\n\t");
		}
		printf("%02x ", spi_data_buf[i]);
	}
	printf("\n");
#endif

	uint16_t *pdata = (uint16_t *)spi_data_buf;
	for (int i = 0; i < w25q_test_len / sizeof(uint16_t); i++) {
		pdata[i] = (uint16_t)(0x80 * write_flag + i);
	}

	printf("[w25q128] write:\n");
	(void)clock_gettime(CLOCK_REALTIME, &starttime);
	ret = w25q128_Write(w25q_test_addr, spi_data_buf, w25q_test_len);
	(void)clock_gettime(CLOCK_REALTIME, &endtime);
	printf("write %08x %dB: %d, T:%d\n", w25q_test_addr, w25q_test_len, ret, RUNTIME_MS(starttime, endtime));

#if DEBUG_W25Q128
	memset(spi_data_buf, 0, w25q_test_len);
	usleep(5000);
	printf("[w25q128] start read...\n");
	(void)clock_gettime(CLOCK_REALTIME, &starttime);
    ret = w25q128_Read(w25q_test_addr, spi_data_buf, w25q_test_len);
	(void)clock_gettime(CLOCK_REALTIME, &endtime);
	printf("read %08x %dB ret=%d T:%d\n", w25q_test_addr, w25q_test_len, ret, RUNTIME_MS(starttime, endtime));
	for (int i = 0; i < w25q_test_len; i++) {
		if (0 == (i & 0x0f)) {
			printf("\n\t");
		}
		printf("%02x ", spi_data_buf[i]);
	}
	printf("\n");
#endif

	free(spi_data_buf);

	return OK;
}

extern int kdbg_free(int argc, char ** args);
static int esp32_spi_read_demo(int argc, char **args)
{
	unsigned char *spi_data_buf = (unsigned char *)zalloc(w25q_test_len);
	if (spi_data_buf == NULL) {
		printf("[w25q128] alloc %dB, fails\n", w25q_test_len);
		return -ERROR;
	}
	int ret;
	memset(spi_data_buf, 0xff, w25q_test_len);
	printf("[w25q128] start read...\n");
	(void)clock_gettime(CLOCK_REALTIME, &starttime);
	ret = w25q128_Read(w25q_test_addr, spi_data_buf, w25q_test_len);
	(void)clock_gettime(CLOCK_REALTIME, &endtime);
	printf("read %08x %dB: %d, T:%d\n", w25q_test_addr, w25q_test_len, ret, RUNTIME_MS(starttime, endtime));
	if (ret == OK) {
		for (int i = 0; i < w25q_test_len; i++) {
			if (0 == (i & 0x0f)) {
				printf("\n\t");
			}
			printf("%02x ", spi_data_buf[i]);
		}
		printf("\n\n");
		kdbg_free(argc, args);
	}
	free(spi_data_buf);
	return OK;
}

static int esp32_spi_reset_demo(int argc, char **args)
{
	printf("[w25q] start reset...\n");
	uint8_t status = 0;
	w25q128_reset();
	do {
		w25q128_get_status(&status);
	} while (0 < (status & W25Q128FV_FSR_BUSY));
	printf(" reset done!\n");
	return OK;
}

/****************************************************************************
* tash test Functions
****************************************************************************/
void ESP32_SPI_command_install(void)
{
	tash_cmd_install("w25q128_start", esp32_spi_start_demo, TASH_EXECMD_ASYNC);
	tash_cmd_install("w25q128_reset", esp32_spi_reset_demo, TASH_EXECMD_ASYNC);
	tash_cmd_install("w25q128_read", esp32_spi_read_demo, TASH_EXECMD_ASYNC);
	tash_cmd_install("w25q128_write", esp32_spi_write_demo, TASH_EXECMD_ASYNC);
}
#endif
