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
#include <tinyara/i2c.h>

#include <apps/shell/tash.h>

#ifdef  CONFIG_I2C
#include "bmp280.h"
#endif
#include "key_led.h"
#include "esp32_i2c_gpio_test.h"

/****************************************************************************
* functions defination
****************************************************************************/

static const tash_cmd_t commands[] = {
#ifdef CONFIG_I2C
	{"bmp280_setup", bmp280_setup, TASH_EXECMD_SYNC},
	{"bmp280_mesr", bmp280_measureone, TASH_EXECMD_SYNC},
	{"bmp280_test", bmp280_test, TASH_EXECMD_ASYNC},
#endif
#ifdef CONFIG_GPIO
	{"key_led_test", key_led_test, TASH_EXECMD_ASYNC},
#endif
	{NULL, NULL, 0}				//END
};

int esp32_i2c_gpio_cmd_install(void)
{
	for (int i = 0; i < sizeof(commands) / sizeof(tash_cmd_t); i++) {
		if (commands[i].name != NULL && commands[i].entry != NULL) {
			tash_cmd_install(commands[i].name, commands[i].entry, commands[i].exectype);
		}
	}

	return 0;
}
