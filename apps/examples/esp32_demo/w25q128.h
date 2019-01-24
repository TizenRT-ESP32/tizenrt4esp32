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

#ifndef __SPI_W25Q128_H_
#define __SPI_W25Q128_H_

/**
* @brief  W25Q128FV
*/
/* 128 MBits => 16MBytes */
#define W25Q128FV_FLASH_SIZE                    0x1000000

/* 256 sectors of 64KBytes */
#define W25Q128FV_SECTOR_SIZE                   0x10000
#define W25Q128FV_SECTOR_ALIGN                  (W25Q128FV_SECTOR_SIZE - 1)
#define W25Q128FV_SECTOR_MASK                   (~W25Q128FV_SECTOR_ALIGN)

/* 512 sectors of 32KBytes */
#define W25Q128FV_HALFSECTOR_SIZE               0x8000
#define W25Q128FV_HALFSECTOR_ALIGN              (W25Q128FV_HALFSECTOR_SIZE - 1)
#define W25Q128FV_HALFSECTOR_MASK               (~W25Q128FV_HALFSECTOR_ALIGN)

/* 4096 subsectors of 4kBytes */
#define W25Q128FV_SUBSECTOR_SIZE                0x1000
#define W25Q128FV_SUBSECTOR_ALIGN               (W25Q128FV_SUBSECTOR_SIZE - 1)
#define W25Q128FV_SUBSECTOR_MASK                (~W25Q128FV_SUBSECTOR_ALIGN)

/* 65536 pages of 256 bytes */
#define W25Q128FV_PAGE_SIZE                     0x100
#define W25Q128FV_PAGE_ALIGN                    (W25Q128FV_PAGE_SIZE - 1)
#define W25Q128FV_PAGE_MASK                     (~W25Q128FV_PAGE_ALIGN)

/* Reset Operations */
#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99

#define READ_DEVID_CMD                      0x90

/* Read Operations */
#define READ_CMD                             0x03
#define FAST_READ_CMD                        0x0B

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

/* Register Operations */
#define READ_STATUS_REG1_CMD                  0x05
#define READ_STATUS_REG2_CMD                  0x35
#define READ_STATUS_REG3_CMD                  0x15

#define WRITE_STATUS_REG1_CMD                 0x01
#define WRITE_STATUS_REG2_CMD                 0x31
#define WRITE_STATUS_REG3_CMD                 0x11

/* Program Operations */
#define PAGE_PROG_CMD                        0x02

/* Erase Operations */
#define SECTOR_ERASE_CMD                     0x20
#define SMALL_BLOCK_ERASE_CMD                0x52
#define BIG_BLOCK_ERASE_CMD                  0xD8
#define CHIP_ERASE_CMD                       0xC7

#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_SUSPEND_CMD               0x75

#define W25Q_ERASESEC_MS    400

/* Flag Status Register */
#define W25Q128FV_FSR_BUSY                    ((uint8_t)0x01)	/*!< busy */
#define W25Q128FV_FSR_WREN                    ((uint8_t)0x02)	/*!< write enable */
#define W25Q128FV_FSR_QE                      ((uint8_t)0x04)	/*!< quad enable */

typedef enum _w25q_sec_ {
	SECTOR = 1,
	SMALLBLOCK = 3,
	BIGBLOCK = 4,
	ALLBLOCK = 50,
} w25q128_Sector_e;

/******************************************************************************/

int getInt(char *arg, int *value);

int w25q128_open_port(void);
int w25q128_read_deviceID(unsigned char *buf);
int w25q128_get_status(uint8_t *status);
int w25q128_enable_write(char enable);
int w25q128_reset(void);
int w25q128_erase_block(uint32_t Address, w25q128_Sector_e sector);
int w25q128_Read(uint32_t ReadAddr, uint8_t *pData, uint32_t Size);
int w25q128_Write(uint32_t WriteAddr, uint8_t *pData, uint32_t Size);

int w25q128_program_block(uint32_t WriteAddr, uint8_t *pData);

void ESP32_SPI_command_install(void);
/******************************************************************************/
#endif							//__SPI_W25Q128_H_
