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

#ifndef __APPS_EXAMPLES_ESP32_DEMO_BMP280_H
#define __APPS_EXAMPLES_ESP32_DEMO_BMP280_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/****************************************************************************
 * type defination
 ****************************************************************************/
/* BMP280 registers address */
typedef enum _bmp280_reg_addr_ {
	REG_ID = 0xD0,
	REG_RESET = 0xE0,
	REG_STATUS = 0xF3,
	REG_CTRL_MEAS = 0xF4,
	REG_CONFIG = 0xF5,
	REG_PRESS_MSB = 0xF7,
	REG_PRESS_LSB = 0xF8,
	REG_PRESS_XLSB = 0xF9,
	REG_TEMP_MSB = 0xFA,
	REG_TEMP_LSB = 0xFB,
	REG_TEMP_XLSB = 0xFC,
} BMP280_REG_ADDR;

/*calibration parameters */
typedef enum _bmp280_calib_e {
	DIG_CALIB_START_REG = 0x88,
	DIG_T1_LSB_REG = 0x88,
	DIG_T1_MSB_REG = 0x89,
	DIG_T2_LSB_REG = 0x8A,
	DIG_T2_MSB_REG = 0x8B,
	DIG_T3_LSB_REG = 0x8C,
	DIG_T3_MSB_REG = 0x8D,
	DIG_P1_LSB_REG = 0x8E,
	DIG_P1_MSB_REG = 0x8F,
	DIG_P2_LSB_REG = 0x90,
	DIG_P2_MSB_REG = 0x91,
	DIG_P3_LSB_REG = 0x92,
	DIG_P3_MSB_REG = 0x93,
	DIG_P4_LSB_REG = 0x94,
	DIG_P4_MSB_REG = 0x95,
	DIG_P5_LSB_REG = 0x96,
	DIG_P5_MSB_REG = 0x97,
	DIG_P6_LSB_REG = 0x98,
	DIG_P6_MSB_REG = 0x99,
	DIG_P7_LSB_REG = 0x9A,
	DIG_P7_MSB_REG = 0x9B,
	DIG_P8_LSB_REG = 0x9C,
	DIG_P8_MSB_REG = 0x9D,
	DIG_P9_LSB_REG = 0x9E,
	DIG_P9_MSB_REG = 0x9F,
} BMP280_CALIB_ADDR_E;

typedef struct bmp280_calib_s {
	unsigned short dig_T1;
	signed short dig_T2;
	signed short dig_T3;
	unsigned short dig_P1;
	signed short dig_P2;
	signed short dig_P3;
	signed short dig_P4;
	signed short dig_P5;
	signed short dig_P6;
	signed short dig_P7;
	signed short dig_P8;
	signed short dig_P9;
	unsigned short reserved;
} BMP280_Calib_S;

typedef union {
	uint8_t regs[26];
	BMP280_Calib_S calibs;
} BMP280_Calib_U;

/****************************************************************************
 * type defination
 ****************************************************************************/
/* BMP280 */
#define BMP280_ID_VALUE         0x58
#define BMP280_RESET_VALUE      0xB6

/*Grove BMP280 sensor with I2C interface
    * slave address: 0x77
    * data register adress: enum BMP280_REG_ADDR;
    *      REG_ID value is 0x58!
    *      REG_RESET; write 0xB6, the device resets;other value takes no effect. The readout value is always 0x00.
    *      REG_STATUS: bit0 bit3
    *      -----
    */
#define BMP280_I2C_ADDRESS       0x77
#define BMP280_I2C_ADDR_LEN      7
#define BMP280_I2C_FREQUENCY     (100*1000)

#define I2C_BUF_LEN     32

#define FLOAT_ENABLE    0

/****************************************************************************
 * functions defination
 ****************************************************************************/
int bmp280_setup(int argc, char *argv[]);

int bmp280_read_register(int argc, char *argv[]);
int bmp280_write_register(int argc, char *argv[]);

int bmp280_measureone(int argc, char *argv[]);

int bmp280_test(int argc, FAR char *argv[]);

#endif							//__APPS_EXAMPLES_ESP32_DEMO_BMP280_H
