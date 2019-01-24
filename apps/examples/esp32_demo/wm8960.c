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
* Include files
****************************************************************************/
#include <tinyara/config.h>

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <tinyara/audio/audio.h>
#include <tinyara/i2c.h>
#include <tinyara/fs/fs.h>

#include "wm8960.h"
#include "i2schar_demo.h"

/****************************************************************************
*
****************************************************************************/
#define WM8960_ADDRESS  0x1a

#define I2C_BUF_LEN     32

#define USE_EARPHONE_MIC    0

#if !defined(USE_EARPHONE_MIC) || (0 >= USE_EARPHONE_MIC)
#define USE_BOARD_MIC       1
#endif

#define DEBUG_WM8960    0

/****************************************************************************
* varibles
****************************************************************************/

uint32_t audio_total_size;
uint32_t audio_rem_size;
uint16_t *current_pos;

/* resgister cache value */
static uint16_t wm8960_reg_val[56] = {
	0x0097, 0x0097, 0x0000, 0x0000, 0x0000, 0x0008, 0x0000, 0x000A,
	0x01C0, 0x0000, 0x00FF, 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x007B, 0x0100, 0x0032, 0x0000, 0x00C3, 0x00C3, 0x01C0,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0100, 0x0100, 0x0050, 0x0050, 0x0050, 0x0050, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0040, 0x0000, 0x0000, 0x0050, 0x0050, 0x0000,
	0x0000, 0x0037, 0x004D, 0x0080, 0x0008, 0x0031, 0x0026, 0x00ED
};

/* I2C control port */
#if defined(CONFIG_I2C)
int fd_i2c = -1;
const int I2C_PORT = 0;
const int i2c_address = WM8960_ADDRESS;
const int i2c_addrlen = 0;		//7bits
const uint32_t i2c_frequency = 100000;
uint8_t recv_buf[I2C_BUF_LEN] = { 0 };
uint8_t send_buf[I2C_BUF_LEN] = { 0 };
#else
#error "WM8960 is controlled via I2C port. Please enable i2c port firstly!"
#endif

/* I2S data port */
const int DATA_PORT = 0;
int fd_i2s = -1;

/****************************************************************************
* Private functions
****************************************************************************/
int i2c_readBytes(uint8_t regAddr, uint8_t count)
{
	int ret = 0;
	memset(send_buf, 0, sizeof(send_buf));	//
	memset(recv_buf, 0, sizeof(recv_buf));	//

#ifdef CONFIG_I2C_USERIO
	uint32_t flags = 0;
	FAR struct i2c_msg_s msgs[2] = { 0 };
	FAR struct i2c_rdwr_ioctl_data_s rw_data_s = { 0 };

	rw_data_s.msgs = msgs;
	rw_data_s.nmsgs = 1;
	send_buf[0] = regAddr;
	msgs[0].addr = i2c_address;
	flags = (i2c_addrlen == 0) ? 0 : I2C_M_TEN;
	msgs[0].flags = flags | I2C_M_NORESTART;	//write reg_addr
	msgs[0].length = 1;			//
	msgs[0].buffer = send_buf;	//
	ret = ioctl(fd_i2c, I2C_RDWR, (unsigned long)&rw_data_s);

	rw_data_s.msgs = msgs;
	rw_data_s.nmsgs = 1;
	msgs[0].addr = i2c_address;
	flags = (i2c_addrlen == 0) ? 0 : I2C_M_TEN;
	msgs[0].flags = flags | I2C_M_READ;	//read data
	msgs[0].length = count;		//
	msgs[0].buffer = recv_buf;	//
	ret = ioctl(fd_i2c, I2C_RDWR, (unsigned long)&rw_data_s);
#else
	struct i2c_config_s config;
	config.frequency = frequency;
	config.address = address;	// slave address
	config.addrlen = (addrlen == 0) ? 7 : 10;

	send_buf[0] = regAddr;

	ret = i2c_writeread(dev, &config, send_buf, 1, recv_buf, count);
#endif
	return ret;
}

int i2c_writeByte(uint8_t regAddr, uint8_t value)
{
	int ret = 0;

	memset(send_buf, 0, sizeof(send_buf));	//
#ifdef CONFIG_I2C_USERIO
	uint32_t flags = 0;

	FAR struct i2c_msg_s msgs[2] = { 0 };
	FAR struct i2c_rdwr_ioctl_data_s rw_data_s = { 0 };

	flags = (i2c_addrlen == 0) ? 0 : I2C_M_TEN;
	rw_data_s.nmsgs = 1;
	rw_data_s.msgs = msgs;

	send_buf[0] = regAddr;
	send_buf[1] = value;
	msgs[0].addr = i2c_address;
	msgs[0].flags = flags;		//write
	msgs[0].length = 2;			//
	msgs[0].buffer = send_buf;	//

	ret = ioctl(fd_i2c, I2C_RDWR, (unsigned long)&rw_data_s);
#else
	struct i2c_config_s config;
	config.frequency = frequency;
	config.address = address;	// slave address
	config.addrlen = (addrlen == 0) ? 7 : 10;

	send_buf[0] = REG_CTRL_MEAS;
	send_buf[1] = 2;

	ret = i2c_write(dev, &config, send_buf, 2);
#endif
	return ret;
}

/**
  * @brief  Write register of WM8960.
  * @param  reg: The number of resigter which to be read.
  * @param  dat: The data which will be writeen to the register.
  * @retval The value of regsiter.
  */
uint8_t wm8960_write_reg(uint8_t reg, uint16_t dat)
{
	uint8_t res = 255;
	uint8_t i2c_data[2];
	i2c_data[0] = (reg << 1) | ((uint8_t)((dat >> 8) & 0x0001));	//RegAddr
	i2c_data[1] = (uint8_t)(dat & 0x00FF);	//RegValue

	int ret = i2c_writeByte(i2c_data[0], i2c_data[1]);

	if (ret == OK) {
		res = 0;
	} else {
		printf("i2c_writeByte %02x %04x: %d\n", reg, dat, ret);
	}

	return res;
}

/**
  * @brief  Read register of WM8960.
  * @param  reg: The number of resigter which to be read.
  * @retval The value of regsiter.
  */
uint16_t wm8960_read_reg(uint8_t reg)
{
	return wm8960_reg_val[reg];
}

/****************************************************************************
* Public functions
****************************************************************************/
/**
* @brief  Open and close ports for WM8960.
* @param  NONE.
* @retval OK if port is opened/closed.
*/
int wm8960_open(void)
{
	char path[16] = { 0 };

	/* Open control port */
	sprintf(path, "/dev/i2c-%d", I2C_PORT);
	fd_i2c = open(path, O_RDWR);
	if (fd_i2c < 0) {
		printf("Open %s fail, exit %d!!\n", path, fd_i2c);
		return ERROR;
	}

	printf("%s opened:%d! config the control port...\n", path, fd_i2c);
	ioctl(fd_i2c, I2C_TENBIT, (unsigned long)i2c_addrlen);
	ioctl(fd_i2c, I2C_SLAVE, (unsigned long)&i2c_address);
	ioctl(fd_i2c, I2C_FREQUENCY, (unsigned long)&i2c_frequency);

	/* Open data port */
	sprintf(path, "/dev/i2schar%d", DATA_PORT);
	fd_i2s = open(path, O_RDWR);
	if (fd_i2s < 0) {
		int errcode = errno;
		printf("failed to open %s: %d\n", DATA_PORT, errcode);

		close(fd_i2c);
		fd_i2c = -1;

		return ERROR;
	}

	return OK;
}

int wm8960_close(void)
{
	close(fd_i2s);
	fd_i2s = -1;
	close(fd_i2c);
	fd_i2c = -1;
	return OK;
}

/**
* @brief  Initialize WM8960
* @param  None
* @retval The return value of the I2C transmit function.
*/
int wm8960_loopback_init(void)
{
	uint8_t res;

	//Reset Device
	res = wm8960_write_reg(0x0F, 0xFFFF);
	if (res != 0) {
		return res;
	} else {
		printf("WM8960 reset completed !!\r\n");
	}

	//Set Power Source
#if USE_BOARD_MIC
	res = wm8960_write_reg(0x19, 0x00EA);	//AINL,ADCL
#elif USE_EARPHONE_MIC
	res = wm8960_write_reg(0x19, 0x00D4);	//AINR,ADCR
#endif
	res += wm8960_write_reg(0x1A, 0x01F8);	//DACL,DACR;LOUT1,ROUT1,SPKL,SPKR
	res += wm8960_write_reg(0x2F, 0x003C);	//LMIC,RMIC,LOMIX,ROMIX

	if (res != 0) {
		printf("Source set fail, Error code: %d\n", res);
		return res;
	}
	//Configure clock
	//MCLK->div1->SYSCLK->DAC/ADC sample Freq = 25MHz(MCLK)/2*256 = 48.8kHz
	wm8960_write_reg(0x04, 0x0000);

	/*********Audio Interface*********/

	//I2S format 16 bits word length
	wm8960_write_reg(0x07, 0x0002);

	/*********PGA*********/
	//Input PGA:Volume Control
	wm8960_write_reg(0x00, 0x003F | 0x0100);	//0x003F
	wm8960_write_reg(0x01, 0x003F | 0x0100);	//0x003F

	//Input Signal Path
#if USE_BOARD_MIC
	wm8960_write_reg(0x20, 0x0010 | 0x0008 | 0x0100);	//LMIC2B,LMN1
	wm8960_write_reg(0x21, 0x0000);
#elif USE_EARPHONE_MIC
	wm8960_write_reg(0x20, 0x0000);
	wm8960_write_reg(0x21, 0x0008 | 0x0100);
#endif

	//Input Boost Mixer
	wm8960_write_reg(0x2B, 0x0000);	//LEFT in2 & in3 mute
	wm8960_write_reg(0x2C, 0x0000);	//RIGHT in2 & in3 mute

	/*********ADC*********/
	//ADC Control //ADC High Pass Filter
	wm8960_write_reg(0x05, 0x000C);

	//ADC Digital Volume Control
	wm8960_write_reg(0x15, 0x00FF | 0x0100);
	wm8960_write_reg(0x16, 0x00FF | 0x0100);

#if USE_BOARD_MIC
	wm8960_write_reg(0x17, 0x01C5);	//TSDEN,VSEL=3.3V, DATSEL=0b01;LOOPBACK
#elif USE_EARPHONE_MIC
	wm8960_write_reg(0x17, 0x01C8);
#endif

	/*********ALC Control*********/
	//Noise Gate Control:
	wm8960_write_reg(0x14, 0x00F9);	// -30dBfs

	/*********OUTPUT SIGNAL PATH*********/
	//DAC Soft-Mute Control
	wm8960_write_reg(0x05, 0x0000);
	wm8960_write_reg(0x06, 0x0000);

	//Digital Volume Control
	wm8960_write_reg(0x0A, 0x00FF | 0x0100);	//FF
	wm8960_write_reg(0x0B, 0x00FF | 0x0100);

	//3D Stereo Enhancement Function
	wm8960_write_reg(0x10, 0x0000);	//No 3D effect

	//ALC close
	wm8960_write_reg(0x11, 0x0000);

	//Left and Right Output Mixer Mute and Volume Control: DAC to out, input3 to out,
	wm8960_write_reg(0x22, 0x0100);	//LD2LO
	wm8960_write_reg(0x25, 0x0100);	//RD2RO
	//bypass: Input Mixer to out
	wm8960_write_reg(0x2D, 0x0000);
	wm8960_write_reg(0x2E, 0x0000);

	/*********ANALOGUE OUTPUTS*********/
	//Headphone LOUT1/ROUT1 Volume Control
	wm8960_write_reg(0x02, 0x005F | 0x80 | 0x0100);	//Left Volume
	wm8960_write_reg(0x03, 0x005F | 0x80 | 0x0100);	//Right Volume

	//Speaker Volume Control
	wm8960_write_reg(0x28, 0x007F | 0x0100);	//Left Speaker Volume
	wm8960_write_reg(0x29, 0x007F | 0x0100);	//Right Speaker Volume

	//Analogue Output Control
	wm8960_write_reg(0x31, 0x00F7);	//Enable Left and right speakers

	//Jack Detect
	wm8960_write_reg(0x18, 1 << 6 | 0 << 5);
	//wm8960_write_reg(0x17, 0x01C3);
	wm8960_write_reg(0x30, 0x0009);	//0x000D,0x0005

	return res;
}

int wm8960_record_init(void)
{
	uint8_t res;

	//Reset Device
	res = wm8960_write_reg(0x0F, 0xFFFF);
	if (res != 0) {
		return res;
	} else {
		printf("WM8960 reset completed !!\r\n");
	}

	//Set Power Source
#if USE_BOARD_MIC
	res = wm8960_write_reg(0x19, 0x00E8);	//AINL,ADCL
#elif USE_EARPHONE_MIC
	res = wm8960_write_reg(0x19, 0x00D4);	//AINR,ADCR
#endif
	res += wm8960_write_reg(0x1A, 0x01F8);	//DACL,DACR;LOUT1,ROUT1,SPKL,SPKR
	res += wm8960_write_reg(0x2F, 0x003C);	//LMIC,RMIC,LOMIX,ROMIX

	if (res != 0) {
		printf("Source set fail, Error code: %d\n", res);
		return res;
	}
	//Configure clock
	wm8960_write_reg(0x04, 0x0000);

	/*********Audio Interface*********/

	//I2S format 16 bits word length
	wm8960_write_reg(0x07, 0x0002);

	/*********PGA*********/
	//Input PGA: Volume Control; 0x003F,0x001F
	wm8960_write_reg(0x00, 0x003F | 0x0100);
	wm8960_write_reg(0x01, 0x003F | 0x0100);

	//Input Signal Path: BOOST: 0x0000, 0x0010, 0x0020, 0x0030
#if USE_BOARD_MIC
	wm8960_write_reg(0x20, 0x0010 | 0x0008 | 0x0100);	//LMIC2B,LMN1
	wm8960_write_reg(0x21, 0x0000);
#elif USE_EARPHONE_MIC
	wm8960_write_reg(0x20, 0x0000);
	wm8960_write_reg(0x21, 0x0008 | 0x0100);
#endif

	//Input Boost Mixer
	wm8960_write_reg(0x2B, 0x0000);	//LEFT in2 & in3 mute
	wm8960_write_reg(0x2C, 0x0000);	//RIGHT in2 & in3 mute

	/*********ADC*********/
	//ADC Control //ADC High Pass Filter
	wm8960_write_reg(0x05, 0x000C);

	//ADC Digital Volume Control: vol=0x0000~0x00FF
	wm8960_write_reg(0x15, 0x00C3 | 0x0100);
	wm8960_write_reg(0x16, 0x00C3 | 0x0100);

#if USE_BOARD_MIC
	wm8960_write_reg(0x17, 0x01C4);
#elif USE_EARPHONE_MIC
	wm8960_write_reg(0x17, 0x01C8);
#endif

	/*********ALC Control*********/
	//Noise Gate Control: NGAT, NGTH: -30dBfs
	wm8960_write_reg(0x14, 0x00F8 | 0x0001);

	/*********OUTPUT SIGNAL PATH*********/
	//DAC Soft-Mute Control
	wm8960_write_reg(0x05, 0x0000);
	wm8960_write_reg(0x06, 0x0000);

	//Digital Volume Control; VOL=0x00~0xff
	wm8960_write_reg(0x0A, 0x0000 | 0x0100);
	wm8960_write_reg(0x0B, 0x0000 | 0x0100);

	//3D Stereo Enhancement Function
	wm8960_write_reg(0x10, 0x0000);	//No 3D effect

	//ALC close
	wm8960_write_reg(0x11, 0x0000);	//

	return 0;
}

int wm8960_play_init(void)
{
	uint8_t res;

	//Reset Device
	res = wm8960_write_reg(0x0f, 0x0000);
	if (res != 0) {
		printf("WM8960 reset fails %d !!\r\n", res);
		return res;
	} else {
		printf("WM8960 reset completed !!\r\n");
	}

	//Set Power Source
	res = wm8960_write_reg(0x19, 1 << 8 | 1 << 7 | 1 << 6);
	res += wm8960_write_reg(0x1A, 1 << 8 | 1 << 7 | 1 << 6 | 1 << 5 | 1 << 4 | 1 << 3);	//PLL power down
	res += wm8960_write_reg(0x2F, 1 << 3 | 1 << 2);
	if (res != 0) {
		printf("[WM8960 init] Power source set fail!! Error code: %d\r\n", res);
		fflush(stdout);
		return res;
	}
	//Configure clock
	wm8960_write_reg(0x04, 0x0000);

	//Configure audio interface
	//I2S format 16 bits word length
	wm8960_write_reg(0x07, 0x0002);

	//Configure ADC/DAC
	wm8960_write_reg(0x05, 0x0000);
	wm8960_write_reg(0x06, 0x0000);	//

	//Configure HP_L and HP_R OUTPUTS
	wm8960_write_reg(0x02, 0x005F | 0x80 | 0x0100);	//LOUT1 Volume Set
	wm8960_write_reg(0x03, 0x005F | 0x80 | 0x0100);	//ROUT1 Volume Set

	//Configure SPK_RP and SPK_RN
	wm8960_write_reg(0x28, 0x006F | 0x0100);	//Left Speaker Volume
	wm8960_write_reg(0x29, 0x006F | 0x0100);	//Right Speaker Volume

	//Enable the OUTPUTS
	wm8960_write_reg(0x31, 0x00F7);	//Enable Class D Speaker Outputs

	//Configure DAC volume
	wm8960_write_reg(0x0a, 0x00FF | 0x0100);
	wm8960_write_reg(0x0b, 0x00FF | 0x0100);

	//Configure MIXER
	wm8960_write_reg(0x22, 1 << 8);	// | 1 << 7);
	wm8960_write_reg(0x25, 1 << 8);	// | 1 << 7);

	//Jack Detect
	wm8960_write_reg(0x18, 1 << 6 | 0 << 5);
	wm8960_write_reg(0x17, 0x01C3);
	wm8960_write_reg(0x30, 0x0009);	//0x000D,0x0005

	return 0;
}

/**
* @brief  Record and play audio.
* @param  None
* @retval None
*/
#define I2S_BUFFSIZE    4064
#define I2S_SAMPLERATE  441000
#define I2S_DELAY   ((8000000/1000) * I2S_BUFFSIZE / (I2S_SAMPLERATE/1000) )
#define I2S_DELAY_us(len) (8000 * (len) / (I2S_SAMPLERATE/1000))
#define I2S_FRAME_MUL   2
#define I2S_FRAME_DIV   3

#define I2S_END_DELAY   10

#define I2SCHAR_RXBUFFERS_COUNT  32

int wm8960_audio_play(uint16_t *pBuffer, uint32_t FullSize)
{
	FAR struct ap_buffer_s *apb;
	struct audio_buf_desc_s desc;
	int index = 0;

	audio_total_size = FullSize;
	audio_rem_size = 0;
	current_pos = pBuffer;
#if defined(DEBUG_WM8960) && (0 < DEBUG_WM8960)
	printf("wm8960_audio_play: data file %d, data count: %d\n", fd_i2s, FullSize);
#endif
	while (1) {
		if (I2S_BUFFSIZE > audio_total_size - audio_rem_size) {
			desc.numbytes = audio_total_size - audio_rem_size;
		} else {
			desc.numbytes = I2S_BUFFSIZE;
		}
		desc.u.ppBuffer = &apb;

		if (desc.numbytes & 0x03) {
			desc.numbytes = (desc.numbytes / 4 + 1) * 4;
		}

		int ret = apb_alloc(&desc);
		if (ret < 0) {
			printf("wm8960_audio_play %d apb_alloc(%d) fails: %d\n", audio_rem_size, desc.numbytes, ret);
			return ERROR;
		}

		memcpy(apb->samp, current_pos, desc.numbytes);
		apb->nbytes = desc.numbytes;

		int bufsize = sizeof(struct ap_buffer_s) + desc.numbytes;
		int nwritten;
		index++;
		/* Then send the buffer */
		do {
			/* Write the buffer to the I2S character driver */
			nwritten = write(fd_i2s, apb, bufsize);
			if (nwritten < 0) {
				int errcode = errno;

				if (errcode != EINTR) {
					apb_free(apb);
					printf("[WM8960] send fails: %d; err:%d\n\n", nwritten, errcode);
					return ERROR;
				}
			} else if (nwritten != bufsize) {
				printf("[WM8960] try send %d fails: %d; \n\n", bufsize, nwritten);
				apb_free(apb);
				return ERROR;
			} else {
#if defined(DEBUG_WM8960) && (0 < DEBUG_WM8960)
				printf("[WM8960] sent %d: %d!!\n\n", index, audio_rem_size);
#endif
			}
		} while (nwritten != bufsize);
		usleep(I2S_FRAME_MUL * I2S_DELAY / I2S_FRAME_DIV);
		apb_free(apb);

		current_pos += desc.numbytes / 2;
		audio_rem_size += desc.numbytes;

		if (audio_rem_size >= audio_total_size) {
#if defined(DEBUG_WM8960) && (0 < DEBUG_WM8960)
			printf("[WM8960] play Done %d: %d\n\n", index, audio_rem_size);
#endif
			break;
		}
	}

	return OK;
}

int wm8960_audio_record(uint16_t *pBuffer, uint32_t FullSize)
{
	struct ap_buffer_s *apb = NULL;
	struct audio_buf_desc_s desc;
	int bufsize, ret;

	audio_total_size = FullSize;
	audio_rem_size = 0;
	current_pos = pBuffer;
#if defined(DEBUG_WM8960) && (0 < DEBUG_WM8960)
	printf("wm8960_audio_record: data file %d, data count: %d\n", fd_i2s, FullSize);
#endif
	while (1) {
		if (I2S_BUFFSIZE > audio_total_size - audio_rem_size) {
			desc.numbytes = audio_total_size - audio_rem_size;
		} else {
			desc.numbytes = I2S_BUFFSIZE;
		}
		if (desc.numbytes & 0x03) {
			desc.numbytes = (desc.numbytes / 4 + 1) * 4;
		}
		desc.u.ppBuffer = &apb;

		ret = apb_alloc(&desc);
		if (ret < 0) {
			printf("%d apb_alloc(%d) fails: %d\n", audio_rem_size, desc.numbytes, ret);
			return ERROR;
		}

		memset(apb->samp, 0x00, desc.numbytes);
#if defined(DEBUG_WM8960) && (0 < DEBUG_WM8960)
		printf("[WM8960] apb %d, desclen: %d\n\n", apb->nmaxbytes, desc.numbytes);
#endif

		bufsize = sizeof(struct ap_buffer_s) + desc.numbytes;
		/* Then read the buffer */
		do {
			/* read the buffer from the I2S character driver */
			ret = read(fd_i2s, apb, bufsize);
			if (ret < 0) {
				int errcode = errno;

				if (errcode != EINTR) {
					apb_free(apb);
					printf("[WM8960] read fails: %d; err:%d\n\n", ret, errcode);
					return ERROR;
				}
			} else if (ret != bufsize) {
				printf("[WM8960] try read %d, return %d \n\n", bufsize, ret);
				apb_free(apb);
				return ERROR;
			} else {
#if defined(DEBUG_WM8960) && (0 < DEBUG_WM8960)
				printf("[WM8960] read %d: %d\n\n", audio_rem_size, desc.numbytes);
#endif
			}
		} while (ret != bufsize);

		usleep(I2S_DELAY);

		memcpy(current_pos, apb->samp, desc.numbytes);
		apb_free(apb);

		current_pos += bufsize / 2;
		audio_rem_size += bufsize;
		if (audio_rem_size >= audio_total_size) {
#if defined(DEBUG_WM8960) && (0 < DEBUG_WM8960)
			printf("[WM8960] record Done %d: %d\n\n", audio_rem_size);
#endif
			break;
		}
	}

	return OK;
}
