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

#ifndef __APPS_EXAMPLES_ESP32_DEMO_I2C_GPIO_TEST_H
#define __APPS_EXAMPLES_ESP32_DEMO_I2C_GPIO_TEST_H
/****************************************************************************
* Included Files
****************************************************************************/
#include <tinyara/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <apps/shell/tash.h>

/****************************************************************************
* type defination
****************************************************************************/
typedef struct _tash_cmd_ {
	const char *name;
	const TASH_CMD_CALLBACK entry;
	const int exectype;
} tash_cmd_t;

/****************************************************************************
* functions defination
****************************************************************************/
int esp32_i2c_gpio_cmd_install(void);

#endif							//__APPS_EXAMPLES_ESP32_DEMO_I2C_GPIO_TEST_H
