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
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <tinyara/audio/audio.h>
#include <tinyara/audio/i2s.h>
#include <tinyara/i2c.h>
#include <tinyara/fs/fs.h>
#include <tinyara/fs/ioctl.h>

#include <apps/shell/tash.h>

#include "wm8960.h"
#include "w25q128.h"
#include "i2schar_demo.h"

#include "esp32_i2c_gpio_test.h"

/****************************************************************************
* micro defines
****************************************************************************/
#ifdef CONFIG_ESP32_I2S_TX
#define I2S_PLAY_TEST       1
#endif
#ifdef CONFIG_ESP32_I2S_RX
#define I2S_RECORD_TEST     1
#endif

#define DEMO_WAV_ENABLE     0

#define PLAY_COUNT          4

/****************************************************************************
* datatype
****************************************************************************/

/****************************************************************************
* varibles
****************************************************************************/

/****************************************************************************
* function prototypes
****************************************************************************/
extern struct i2s_dev_s *esp32_i2s_initialize(uint16_t port);
extern int kdbg_free(int argc, char **args);

/****************************************************************************
* private functions
****************************************************************************/
#if defined(I2S_PLAY_TEST) && I2S_PLAY_TEST > 0
int demo_play_wav(int argc, char *argv[])
{
	int flag = 0;
	if (argc > 1 && argv[1] != NULL) {
		getInt(argv[1], &flag);
	}
	int res = -1;
	res = wm8960_open();
	if (res < 0) {
		return ERROR;
	}

	printf("WM8960_Play_Init start...\n");
	sleep(1);
	res = wm8960_play_init();
	if (res == 0) {
		printf("WM8960_Init complete !!\n");
	} else {
		printf("WM8960_Init fail ! Error code: %d\n", res);
		return ERROR;
	}

	printf("flag:%d\n", flag);
	if (0 < flag) {
#if defined(CONFIG_SPI)
		if (0 < w25q128_open_port()) {
			return ERROR;
		}
		uint8_t buf[4] = { 0 };
		unsigned int id = w25q128_read_deviceID(buf);
		w25q128_get_status(buf + 2);
		printf("w25q128 devid: %x; buf: %02x %02x; status: %02x \n", id, buf[0], buf[1], buf[2]);

		uint8_t *pBuffer = (uint8_t *)zalloc(RECORD_MEM_SIZE);
		if (pBuffer == NULL) {
			printf("zalloc(%d) fails\n", RECORD_MEM_SIZE);
			return ERROR;
		}

		uint32_t record_addr = RECORD_ADDR & W25Q128FV_SUBSECTOR_MASK;
		uint32_t rlen = 0;
		uint32_t len = 0;
		while (rlen < RECORD_LEN) {
			len = RECORD_LEN - rlen;
			if (len > RECORD_MEM_SIZE) {
				len = RECORD_MEM_SIZE;
			}
			w25q128_Read(record_addr, pBuffer, len);

			res = wm8960_audio_play((uint16_t *)pBuffer, RECORD_MEM_SIZE);
			if (res < 0) {
				printf("wm8960_audio_play 0x%08x fail: %d!!\n\n", record_addr, res);
				break;
			}

			record_addr += len;
			rlen += len;
		}
		free(pBuffer);
#endif
	} else {
#if defined(DEMO_WAV_ENABLE) && (0 < DEMO_WAV_ENABLE)
		int wav_data_length = sizeof(demo_wav_data);
		printf("wav_data_length:%d\n", wav_data_length);
		int count = 0;
		while (count++ < PLAY_COUNT) {
			res = wm8960_audio_play((uint16_t *)demo_wav_data, wav_data_length);
			printf("wm8960_audio_play %d Done:%d!!\n\n", count, res);
			sleep(1);
		}
#else
		printf("Play demo wav file is disabled.\n");
#endif
	}

	wm8960_close();
	printf("\nThe end of play demo!\n");

	return res;
}
#endif

#if defined(I2S_RECORD_TEST) && I2S_RECORD_TEST > 0
int demo_record_wav(int argc, char *argv[])
{
	int res = -1;
	res = wm8960_open();
	if (res < 0) {
		return ERROR;
	}

	printf("WM8960_Reocrd_Init start...\n");
	usleep(10000);
	res = wm8960_record_init();
	if (res == 0) {
		printf("WM8960_Init complete !!\n");
	} else {
		printf("WM8960_Init fail ! Error code: %d\n", res);
		return ERROR;
	}

	uint16_t *pBuffer = (uint16_t *)zalloc(RECORD_MEM_SIZE);
	if (pBuffer == NULL) {
		printf("zalloc(%d) fails\n", RECORD_MEM_SIZE);
		return ERROR;
	}
#if defined(CONFIG_SPI)
	if (0 < w25q128_open_port()) {
		return ERROR;
	}
	uint8_t buf[4] = { 0 };
	unsigned int id = w25q128_read_deviceID(buf);
	w25q128_get_status(buf + 2);
	printf("w25q128 devid: %x; buf: %02x %02x; status: %02x \n", id, buf[0], buf[1], buf[2]);
#endif

	int count = 0;
	while (count++ < (RECORD_LEN / RECORD_MEM_SIZE)) {
		res = wm8960_audio_record(pBuffer, RECORD_MEM_SIZE);
		if (res < 0) {
			printf("wm8960_audio_record %d fail:%d!!\n\n", count, res);
			break;
		}
#if defined(CONFIG_SPI)
		uint32_t record_addr = RECORD_ADDR & W25Q128FV_SUBSECTOR_MASK;
		uint32_t wlen = 0;
		uint32_t len = 0;
		uint8_t *pData = (uint8_t *)pBuffer;
		while (wlen < RECORD_MEM_SIZE) {
			len = RECORD_MEM_SIZE - wlen;
			if (len > W25Q128FV_SUBSECTOR_SIZE) {
				len = W25Q128FV_SUBSECTOR_SIZE;
			}
			if (0 == (record_addr & W25Q128FV_SUBSECTOR_ALIGN)) {
				w25q128_erase_block(record_addr, SECTOR);
			}
			w25q128_Write(record_addr, pData + wlen, len);

			wlen += len;
			record_addr += len;
		}
#else
		for (int len = 0; len < RECORD_MEM_SIZE / sizeof(uint16_t); len++) {
			if (0 == (len & 0x07)) {
				printf("\n");
			}
			printf("0x%04x,", pBuffer[len]);
		}
		printf("\n");
#endif
	}

	/* release resource after record */
	free(pBuffer);

	wm8960_close();
	printf("\nThe end of record!\n");

	return res;
}
#endif

int demo_loopback(int argc, char *argv[])
{
	int res = -1;
	res = wm8960_open();
	if (res < 0) {
		return ERROR;
	}

	res = wm8960_loopback_init();

	wm8960_close();

	return res;
}

int demo_memtest(int argc, char *argv[])
{
	int res = -1;
	struct ap_buffer_s *apb = NULL;
	struct audio_buf_desc_s desc;
	int index = 0;
	kdbg_free(1, NULL);
	for (; index < 100; index++) {
		desc.numbytes = 4064;
		desc.u.ppBuffer = &apb;

		res = apb_alloc(&desc);
		if (res < 0) {
			printf("apb_alloc(%d) fails: %d\n", desc.numbytes, res);
			break;
		}
		memset(apb->samp, 0x00, desc.numbytes);

		apb_free(apb);
		kdbg_free(1, NULL);
	}
	return res;
}

static void esp32_i2schar_initialize(void)
{
	static bool i2s_initialized;
	struct i2s_dev_s *i2s;
	int ret;
	int port;

	/* Have we already initialized? */
	if (!i2s_initialized) {
		/* Call espP32_i2s_initialize() to get an instance of the I2S interface */
#if defined(CONFIG_ESP32_I2S0) && (1==CONFIG_ESP32_I2S0)
		port = 0;
		i2s = esp32_i2s_initialize(port);
		if (!i2s) {
			lldbg("ERROR: Failed to get the ESP32 I2S%d driver\n", port);
			return;
		} else {
			ret = i2schar_register(i2s, port);
			if (ret < 0) {
				lldbg("ERROR: i2schar_register failed: %d\n", ret);
			}
		}
		//I2S_ERR_CB_REG(i2s, err_cb, "Error_Test_String");
#endif

#if defined(CONFIG_ESP32_I2S1) && (1==CONFIG_ESP32_I2S1)
		port = 1;
		i2s = esp32_i2s_initialize(port);
		if (!i2s) {
			lldbg("ERROR: Failed to get the ESP32 I2S%d driver\n", port);
			return;
		} else {
			ret = i2schar_register(i2s, port);
			if (ret < 0) {
				lldbg("ERROR: i2schar_register failed: %d\n", ret);
			}
		}
		//I2S_ERR_CB_REG(i2s, err_cb, "Error_Test_String");
#endif

		/* Now we are initialized */
		i2s_initialized = true;
	}
}

/****************************************************************************
* public Functions
****************************************************************************/
static const tash_cmd_t commands[] = {
#if defined(I2S_PLAY_TEST) && I2S_PLAY_TEST > 0
	{"wm8960_play", demo_play_wav, TASH_EXECMD_ASYNC},
#endif
#if defined(I2S_RECORD_TEST) && I2S_RECORD_TEST > 0
	{"wm8960_record", demo_record_wav, TASH_EXECMD_ASYNC},
#endif
	{"i2s_memtest", demo_memtest, TASH_EXECMD_ASYNC},
	{"i2s_loopback", demo_loopback, TASH_EXECMD_ASYNC}
};

void esp32_i2schar_install(void)
{
	esp32_i2schar_initialize();

	int i = 0;
	for (; i < sizeof(commands) / sizeof(tash_cmd_t); i++) {
		if (commands[i].name != NULL && NULL != commands[i].entry) {
			tash_cmd_install(commands[i].name, commands[i].entry, commands[i].exectype);
		}
	}
}
