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
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <tinyara/gpio.h>
#include <tinyara/fs/fs.h>

#include "key_led.h"
/****************************************************************************
 * private macro definations
 ****************************************************************************/
#define KEY_LED_DELAY   (200 * 1000)
#define KEY_LED_COUNT   (250)
typedef struct _led_status_ {
	uint32_t green: 8;
	uint32_t red: 8;
	uint32_t blue: 8;
	uint32_t reserverd: 8;
} led_status_s;
/****************************************************************************
* private varibles
****************************************************************************/
const char *gpio_path_format = "/dev/gpio%d";

const led_status_s led_run_status[] = {
	{.green = LED_OFF, .red = LED_OFF, .blue = LED_OFF},
	{.green = LED_ON, .red = LED_OFF, .blue = LED_OFF},
	{.green = LED_OFF, .red = LED_ON, .blue = LED_OFF},
	{.green = LED_ON, .red = LED_ON, .blue = LED_OFF},
	{.green = LED_OFF, .red = LED_OFF, .blue = LED_ON},
	{.green = LED_ON, .red = LED_OFF, .blue = LED_ON},
	{.green = LED_OFF, .red = LED_ON, .blue = LED_ON},
	{.green = LED_ON, .red = LED_ON, .blue = LED_ON}
};

int key_get_status(int key, int *status)
{
	int fd_key = -1;

	int ret = ERROR;
	char path[16] = { 0 };

	if (key == KEY_IO_NUM) {
		sprintf(path, gpio_path_format, KEY_IO_NUM);
		fd_key = open(path, O_RDWR);
		if (fd_key < 0) {
			printf("open key fails\n");
			return ret;
		}

		ioctl(fd_key, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_IN);
		ioctl(fd_key, GPIOIOC_SET_DRIVE, GPIO_DRIVE_FLOAT);

		memset(path, 0, sizeof(path));
		ret = read(fd_key, path, 4);
		path[4] = 0;
		*status = atoi(path);
	}

	if (fd_key >= 0) {
		close(fd_key);
		fd_key = -1;
	}

	return OK;
}

static int fd_led_red = -1;
static int fd_led_green = -1;
static int fd_led_blue = -1;

int led_open(void)
{
	char path[16] = { 0 };

	sprintf(path, gpio_path_format, LED_RED_IO_NUM);
	fd_led_red = open(path, O_RDWR);
	if (fd_led_red < 0) {
		return ERROR;
	}
	ioctl(fd_led_red, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_OUT);

	sprintf(path, gpio_path_format, LED_GREEN_IO_NUM);
	fd_led_green = open(path, O_RDWR);
	if (fd_led_green < 0) {
		return ERROR;
	}
	ioctl(fd_led_green, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_OUT);

	sprintf(path, gpio_path_format, LED_BLUE_IO_NUM);
	fd_led_blue = open(path, O_RDWR);
	if (fd_led_blue < 0) {
		return ERROR;
	}
	ioctl(fd_led_blue, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_OUT);

	return OK;
}

int led_set_status(int led, int status)
{
	char str[4] = { 0 };
	int ret = ERROR;
	int len = sprintf(str, "%d", status > 0);
	if (led == LED_GREEN_IO_NUM) {
		if (fd_led_green >= 0) {
			ret = write(fd_led_green, str, len);
		}
	} else if (led == LED_RED_IO_NUM) {
		if (fd_led_red >= 0) {
			ret = write(fd_led_red, str, len);
		}
	} else if (led == LED_BLUE_IO_NUM) {
		if (fd_led_blue >= 0) {
			ret = write(fd_led_blue, str, len);
		}
	}
	//printf("LED %d : %s\n", led, str);
	return ret;
}

int led_close(void)
{
	if (fd_led_green >= 0) {
		close(fd_led_green);
		fd_led_green = -1;
	}

	if (fd_led_red >= 0) {
		close(fd_led_red);
		fd_led_red = -1;
	}

	if (fd_led_blue >= 0) {
		close(fd_led_blue);
		fd_led_blue = -1;
	}

	return OK;
}

int key_led_test(int argc, char *argv[])
{
	int ret = led_open();
	if (ret != OK) {
		printf("Open led port fails!!\n");
		goto END_OF_TEST;
	}

	printf("key&led test start...\n");

	led_set_status(LED_GREEN_IO_NUM, LED_OFF);

	led_set_status(LED_RED_IO_NUM, LED_OFF);
	led_set_status(LED_BLUE_IO_NUM, LED_OFF);

	int led_status = 0;
	int key_status = -1, key_oldstatus = -1;
	for (int i = 0; i < KEY_LED_COUNT; i++) {
		usleep(KEY_LED_DELAY);
		ret = key_get_status(KEY_IO_NUM, &key_status);

		if (key_oldstatus != key_status) {
			key_oldstatus = key_status;
			if (key_status > 0) {
				led_status++;
				if (led_status >= sizeof(led_run_status) / sizeof(led_status_s)) {
					led_status = 0;
				}

				led_status_s cur_status = led_run_status[led_status];
				led_set_status(LED_GREEN_IO_NUM, cur_status.green);
				led_set_status(LED_RED_IO_NUM, cur_status.red);
				led_set_status(LED_BLUE_IO_NUM, cur_status.blue);
			}
		}
	}

END_OF_TEST:
	printf("key&led test end: %d!\n", ret);
	led_close();
	return ret;
}
