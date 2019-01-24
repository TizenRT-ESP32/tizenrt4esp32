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

#ifndef __APPS_EXAMPLES_ESP32_DEMO_KEY_LED_H_
#define __APPS_EXAMPLES_ESP32_DEMO_KEY_LED_H_
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/****************************************************************************
 * Macro definations
 ****************************************************************************/
#define LED_RED_IO_NUM				0
#define LED_GREEN_IO_NUM			2
#define LED_BLUE_IO_NUM				4

#define LED_ON                  1
#define LED_OFF                 0

/*
Several pins of JP1 can't be used as gpio:
GPIO16 & GPIO17: are not connected to the connectoer.
GPIO32 & GPIO33: are connected to XTAL 32khz.
There are several pins limited by hardware:
GPIO14 & GPIO15: are connected to SD card and pulled up to VDD33.
GPIO13: pulled up and down by both 10Kohm resistances; it is not limit.
*/
#define KEY_IO_NUM				15

#define KEY_PRESSED             1

/****************************************************************************
 * functions defination
 ****************************************************************************/

int key_get_status(int key, int *status);

int led_open(void);
int led_set_status(int led, int status);
int led_close(void);
int key_led_test(int argc, char *argv[]);

#endif							//__APPS_EXAMPLES_ESP32_DEMO_KEY_LED_H_
