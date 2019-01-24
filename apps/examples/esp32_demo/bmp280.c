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

#include "bmp280.h"

/****************************************************************************
 * macro definations
 ****************************************************************************/

/****************************************************************************
 * datatype
 ****************************************************************************/

/****************************************************************************
 * private varibles
 ****************************************************************************/
const int bmp280_i2c_address = BMP280_I2C_ADDRESS;	// bmp280 slave i2c_address
const int bmp280_i2c_addrlen = (BMP280_I2C_ADDR_LEN == 10);	//0:7bits; 1:10bits;
const uint32_t bmp280_i2c_frequency = BMP280_I2C_FREQUENCY;	//

const int bmp280_i2c_port = 0;
const char *i2c_path_format = "/dev/i2c-%d";

uint8_t i2c_recv_buf[I2C_BUF_LEN] = { 0 };
uint8_t i2c_send_buf[I2C_BUF_LEN] = { 0 };

int fd_i2c_bmp280 = -1;
struct i2c_dev_s *i2c_dev = NULL;

BMP280_Calib_U bmp280_calib = { 0 };

int32_t bmp280_t_fine = 0;

#define BMP280_CONFIG_VALUE     0xA0
#define BMP280_CONTROL_VALUE    0x49

/****************************************************************************
 * private functions
 ****************************************************************************/
int bmp280_i2c_readBytes(uint8_t regAddr, uint8_t count)
{
	int ret = 0;
	memset(i2c_send_buf, 0, sizeof(i2c_send_buf));	//
	memset(i2c_recv_buf, 0, sizeof(i2c_recv_buf));	//

#ifdef CONFIG_I2C_USERIO
	uint32_t flags = 0;
	FAR struct i2c_msg_s msgs[2] = { 0 };
	FAR struct i2c_rdwr_ioctl_data_s rw_data_s = { 0 };

	rw_data_s.msgs = msgs;
	rw_data_s.nmsgs = 1;
	i2c_send_buf[0] = regAddr;
	msgs[0].addr = bmp280_i2c_address;
	flags = (bmp280_i2c_addrlen == 0) ? 0 : I2C_M_TEN;
	msgs[0].flags = flags | I2C_M_NORESTART;	//write reg_addr
	msgs[0].length = 1;			//
	msgs[0].buffer = i2c_send_buf;	//
	ret = ioctl(fd_i2c_bmp280, I2C_RDWR, (unsigned long)&rw_data_s);

	rw_data_s.msgs = msgs;
	rw_data_s.nmsgs = 1;
	msgs[0].addr = bmp280_i2c_address;
	flags = (bmp280_i2c_addrlen == 0) ? 0 : I2C_M_TEN;
	msgs[0].flags = flags | I2C_M_READ;	//read data
	msgs[0].length = count;		//
	msgs[0].buffer = i2c_recv_buf;	//
	ret = ioctl(fd_i2c_bmp280, I2C_RDWR, (unsigned long)&rw_data_s);
#else
	struct i2c_config_s config;
	config.bmp280_i2c_frequency = bmp280_i2c_frequency;
	config.bmp280_i2c_address = bmp280_i2c_address;	// slave bmp280_i2c_address
	config.bmp280_i2c_addrlen = (bmp280_i2c_addrlen == 0) ? 7 : 10;

	i2c_send_buf[0] = regAddr;

	ret = i2c_writeread(dev, &config, i2c_send_buf, 1, i2c_recv_buf, count);
#endif
	return ret;
}

int bmp280_i2c_writeByte(uint8_t regAddr, uint8_t value)
{
	int ret = 0;

	memset(i2c_send_buf, 0, sizeof(i2c_send_buf));	//
#ifdef CONFIG_I2C_USERIO
	uint32_t flags = 0;

	FAR struct i2c_msg_s msgs[2] = { 0 };
	FAR struct i2c_rdwr_ioctl_data_s rw_data_s = { 0 };

	flags = (bmp280_i2c_addrlen == 0) ? 0 : I2C_M_TEN;
	rw_data_s.nmsgs = 1;
	rw_data_s.msgs = msgs;

	i2c_send_buf[0] = regAddr;
	i2c_send_buf[1] = value;
	msgs[0].addr = bmp280_i2c_address;
	msgs[0].flags = flags;		//write
	msgs[0].length = 2;			//
	msgs[0].buffer = i2c_send_buf;	//

	ret = ioctl(fd_i2c_bmp280, I2C_RDWR, (unsigned long)&rw_data_s);
#else
	struct i2c_config_s config;
	config.bmp280_i2c_frequency = bmp280_i2c_frequency;
	config.bmp280_i2c_address = bmp280_i2c_address;	// slave bmp280_i2c_address
	config.bmp280_i2c_addrlen = (bmp280_i2c_addrlen == 0) ? 7 : 10;

	i2c_send_buf[0] = REG_CTRL_MEAS;
	i2c_send_buf[1] = 2;

	ret = i2c_write(dev, &config, i2c_send_buf, 2);
#endif
	return ret;
}

/* Returns temperature in DegC, double precision. Output value of ¡°51.23¡± equals 51.23 DegC. */
#if FLOAT_ENABLE
static float bmp280_compensate_temperature_double(int32_t adc_T)
#else
static int bmp280_compensate_temperature_double(int32_t adc_T)
#endif
{
	int32_t var1 = 0, var2 = 0;

	var1 = ((((adc_T >> 3) - ((int32_t) bmp280_calib.calibs.dig_T1 << 1))) * ((int32_t) bmp280_calib.calibs.dig_T2)) >> 11;

	var2 = (((((adc_T >> 4) - ((int32_t) bmp280_calib.calibs.dig_T1)) * ((adc_T >> 4) - ((int32_t) bmp280_calib.calibs.dig_T1))) >> 12) * ((int32_t) bmp280_calib.calibs.dig_T3)) >> 14;

	bmp280_t_fine = var1 + var2;
	//printf("bmp280_t_fine: %d;\n", bmp280_t_fine);

#if FLOAT_ENABLE
	float temp = 0;
	temp = adc_T;
	temp = (5.0 * bmp280_t_fine + 128.0) / 25600.0;
	printf("temp = %d\n", (int)temp);
#else
	adc_T = (bmp280_t_fine * 5 + 128) >> 8;
	adc_T /= 100;
	//printf("temp = %d\n", adc_T);
#endif

	return adc_T;
}

/* Returns pressure in Pa as double. Output value of ¡°96386.2¡± equals 96386.2 Pa = 963.862 hPa */
#if FLOAT_ENABLE
static float bmp280_compensate_pressure_double(int32_t adc_P)
#else
static int bmp280_compensate_pressure_double(int32_t adc_P)
#endif
{
	int64_t var1, var2, p;
	var1 = ((int64_t) bmp280_t_fine) - 128000;
	var2 = var1 * var1 * (int64_t) bmp280_calib.calibs.dig_P6;
	var2 = var2 + ((var1 * (int64_t) bmp280_calib.calibs.dig_P5) << 17);
	var2 = var2 + (((int64_t) bmp280_calib.calibs.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) bmp280_calib.calibs.dig_P3) >> 8) + ((var1 * (int64_t) bmp280_calib.calibs.dig_P2) << 12);
	var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) bmp280_calib.calibs.dig_P1) >> 33;

	if (var1 == 0) {
		return 0;				// avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) bmp280_calib.calibs.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t) bmp280_calib.calibs.dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t) bmp280_calib.calibs.dig_P7) << 4);
	p /= 256;
#if FLOAT_ENABLE
	return (float)p / 256;
#else
	return p;
#endif
}

#if 0
float bmp280_estimate_altitude(float seaLevelhPa)
{
	float altitude;

	float pressure = readPressure();	// in Si units for Pascal
	pressure /= 100;

	altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

	return altitude;
}
#endif

/****************************************************************************
* public functions
****************************************************************************/
int bmp280_setup(int argc, char *argv[])
{
#ifdef CONFIG_I2C_USERIO
	char path[16] = { 0 };

	sprintf(path, "/dev/i2c-%d", bmp280_i2c_port);
	fd_i2c_bmp280 = open(path, O_RDWR);
	if (fd_i2c_bmp280 < 0) {
		printf("Open %s fail, exit %d!!\n", path, fd_i2c_bmp280);
		return -1;
	}
	usleep(1000);

	//Config I2C port
	ioctl(fd_i2c_bmp280, I2C_SLAVE, (unsigned long)&bmp280_i2c_address);
	ioctl(fd_i2c_bmp280, I2C_FREQUENCY, (unsigned long)&bmp280_i2c_frequency);
	ioctl(fd_i2c_bmp280, I2C_TENBIT, (unsigned long)bmp280_i2c_addrlen);

	int ret = bmp280_i2c_readBytes(REG_ID, 1);
	printf("REG_ID = %02x, ret=%d\n", i2c_recv_buf[0], ret);
	if (ret == OK && i2c_recv_buf[0] == 0x58) {
		ret = bmp280_i2c_readBytes(DIG_CALIB_START_REG, 26);
		memcpy(bmp280_calib.regs, i2c_recv_buf, 26);

		printf("config REG_CONFIG = ~~!\n");
		ret = bmp280_i2c_writeByte(REG_CONFIG, BMP280_CONFIG_VALUE);
		ret = bmp280_i2c_readBytes(REG_CONFIG, 1);
		printf("REG_CONFIG = %02x, ret=%d\n", i2c_recv_buf[0], ret);

		ret = bmp280_i2c_readBytes(REG_ID, 1);
		printf("REG_ID = %02x, ret=%d\n", i2c_recv_buf[0], ret);
	} else {
		printf("[ERROR] Read ID fails!\n");
	}
#else
	dev = up_i2cinitialize(1);
#endif
	return 0;
}

int bmp280_read_register(int argc, char *argv[])
{
	int ret = 0, i = 0;
	if (fd_i2c_bmp280 < 0) {
		printf("File is not opened!\n");
		return -1;
	}

	uint8_t regAddr = 0, count = 0;
	if (argc >= 2 && argv[1] != NULL) {
		ret = strlen(argv[1]);
		for (i = 0; i < ret; i++) {
			argv[1][i] = tolower(argv[1][i]);
		}
		if (argv[1][0] == '0' && argv[1][1] == 'x') {
			sscanf(argv[1], "0x%x", &ret);
			regAddr = ret;
			ret = 16;
		} else {
			sscanf(argv[1], "%d", &ret);
			regAddr = ret;
			ret = 10;
		}
	} else {
		regAddr = REG_ID;
	}
	if (argc >= 3 && argv[2] != NULL) {
		sscanf(argv[2], "%d", &ret);
		count = ret;
	}
	if (count <= 0) {
		count = 1;
	}

	ret = bmp280_i2c_readBytes(regAddr, count);
	for (i = 0; i < count; i++) {
		printf("data[%d] =  %02x\n", i, i2c_recv_buf[i]);
	}

	return 0;
}

int bmp280_write_register(int argc, char *argv[])
{
	int ret = 0, i = 0;
	if (fd_i2c_bmp280 < 0) {
		printf("File is not opened!\n");
		return -1;
	}

	uint8_t regAddr = 0, value = 0;
	if (argc >= 2 && argv[1] != NULL) {
		ret = strlen(argv[1]);
		for (i = 0; i < ret; i++) {
			argv[1][i] = tolower(argv[1][i]);
		}
		if (argv[1][0] == '0' && argv[1][1] == 'x') {
			sscanf(argv[1], "0x%x", &ret);
		} else {
			sscanf(argv[1], "%d", &ret);
		}
		regAddr = ret;
	} else {
		regAddr = REG_RESET;
	}
	if (argc >= 3 && argv[2] != NULL) {
		ret = strlen(argv[2]);
		for (i = 0; i < ret; i++) {
			argv[2][i] = tolower(argv[2][i]);
		}
		if (argv[2][0] == '0' && argv[2][1] == 'x') {
			sscanf(argv[2], "0x%x", &ret);
		} else {
			sscanf(argv[2], "%d", &ret);
		}
		value = ret;
	} else {
		value = 0xB6;
	}

	ret = bmp280_i2c_writeByte(regAddr, value);

	return 0;
}

int bmp280_measureone(int argc, char *argv[])
{
	int ret = 0, count = 0;
	if (fd_i2c_bmp280 < 0) {
		printf("File is not opened!\n");
		return -1;
	}

	count = 0;
	if (argc >= 2 && argv[1] != NULL) {
		sscanf(argv[1], "%d", &count);
	}

	if (count > 0) {
		bmp280_i2c_writeByte(REG_RESET, 0xB6);
		usleep(1000);
	}
	do {
		usleep(10);
		ret = bmp280_i2c_readBytes(REG_ID, 1);
		count++;
		if (count > 30) {
			ret = -30;
			break;
		}
	} while (ret != OK || i2c_recv_buf[0] != 0x58);
	if (ret != OK) {
		return -1;
	}

	ret = bmp280_i2c_writeByte(REG_CONFIG, BMP280_CONFIG_VALUE);
	ret = bmp280_i2c_writeByte(REG_CTRL_MEAS, BMP280_CONTROL_VALUE);

	ret = bmp280_i2c_readBytes(REG_CONFIG, 1);
	printf("REG_CONFIG = %02x, ret=%d\n", i2c_recv_buf[0], ret);
	ret = bmp280_i2c_readBytes(REG_CTRL_MEAS, 1);
	printf("REG_CTRL_MEAS = %02x, ret=%d\n", i2c_recv_buf[0], ret);

	do {
		usleep(100);
		ret = bmp280_i2c_readBytes(REG_STATUS, 1);
		if (ret != OK || 0 == (i2c_recv_buf[0] & 0x09)) {
			break;
		}
	} while (1);

	if (OK == ret) {
		int32_t temp = 0, preas = 0;
#if FLOAT_ENABLE
		float value = 0;
#endif

		bmp280_i2c_readBytes(REG_TEMP_MSB, 3);
		temp = (i2c_recv_buf[0] << 12) | (i2c_recv_buf[1] << 4) | (i2c_recv_buf[2] >> 4);
#if FLOAT_ENABLE
		value = bmp280_compensate_temperature_double(temp);
		temp = (int)value;
#else
		temp = bmp280_compensate_temperature_double(temp);
#endif
		printf("Temp val=%d\n", temp);

		bmp280_i2c_readBytes(REG_PRESS_MSB, 3);
		preas = (i2c_recv_buf[0] << 12) | (i2c_recv_buf[1] << 4) | (i2c_recv_buf[2] >> 4);
#if FLOAT_ENABLE
		value = bmp280_compensate_pressure_double(preas);
		printf("Preas raw=%d, val=%d\n", preas, (int)value);
		preas = (int)value;
#else
		preas = bmp280_compensate_pressure_double(preas);
#endif
		printf("Preas val=%d\n", preas);
	} else {
		printf("Measure fails: %d!\n\n", ret);
	}
	return 0;
}

int bmp280_test(int argc, FAR char *argv[])
{
	char path[16] = { 0 };

	sprintf(path, "/dev/i2c-%d", bmp280_i2c_port);

	fd_i2c_bmp280 = open(path, O_RDWR);
	if (fd_i2c_bmp280 < 0) {
		printf("Open %s fail, exit %d!!\n", path, fd_i2c_bmp280);
		return -1;
	} else {
		printf("Open %s: %d!!\n", path, fd_i2c_bmp280);
	}

	//Config I2C port
	ioctl(fd_i2c_bmp280, I2C_SLAVE, (unsigned long)&bmp280_i2c_address);
	ioctl(fd_i2c_bmp280, I2C_FREQUENCY, (unsigned long)&bmp280_i2c_frequency);
	ioctl(fd_i2c_bmp280, I2C_TENBIT, (unsigned long)bmp280_i2c_addrlen);

	int ret = bmp280_i2c_readBytes(REG_ID, 1);
	printf("REG_ID = %02x, ret=%d\n", i2c_recv_buf[0], ret);
	if (ret == OK && i2c_recv_buf[0] == 0x58) {
		//DIG_CALIB_REGs
		ret = bmp280_i2c_readBytes(DIG_CALIB_START_REG, 26);
		memcpy(bmp280_calib.regs, i2c_recv_buf, 26);

		printf("config ~~!\n");
		ret = bmp280_i2c_writeByte(REG_CONFIG, BMP280_CONFIG_VALUE);
		ret = bmp280_i2c_writeByte(REG_CTRL_MEAS, BMP280_CONTROL_VALUE);

		ret = bmp280_i2c_readBytes(REG_CONFIG, 1);
		printf("REG_CONFIG = %02x, ret=%d\n", i2c_recv_buf[0], ret);
		ret = bmp280_i2c_readBytes(REG_CTRL_MEAS, 1);
		printf("REG_CTRL_MEAS = %02x, ret=%d\n", i2c_recv_buf[0], ret);

		do {
			usleep(100);
			ret = bmp280_i2c_readBytes(REG_STATUS, 1);
			if (ret != OK || 0 == (i2c_recv_buf[0] & 0x09)) {
				break;
			}
		} while (1);

		if (OK == ret) {
			int32_t temp = 0, preas = 0;
#if FLOAT_ENABLE
			float value = 0;
#endif

			bmp280_i2c_readBytes(REG_TEMP_MSB, 3);
			temp = (i2c_recv_buf[0] << 12) | (i2c_recv_buf[1] << 4) | (i2c_recv_buf[2] >> 4);
#if FLOAT_ENABLE
			value = bmp280_compensate_temperature_double(temp);
			temp = (int)value;
#else
			temp = bmp280_compensate_temperature_double(temp);
#endif
			printf("Temp val=%d\n", temp);

			bmp280_i2c_readBytes(REG_PRESS_MSB, 3);
			preas = (i2c_recv_buf[0] << 12) | (i2c_recv_buf[1] << 4) | (i2c_recv_buf[2] >> 4);
#if FLOAT_ENABLE
			value = bmp280_compensate_pressure_double(preas);
			printf("Preas raw=%d, val=%d\n", preas, (int)value);
			preas = (int)value;
#else
			preas = bmp280_compensate_pressure_double(preas);
#endif
			printf("Preas val=%d\n", preas);
		}
	} else {
		printf("[ERROR] Read ID fails!\n");
	}
	return ret;
}
