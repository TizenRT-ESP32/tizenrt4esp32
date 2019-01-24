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
#ifndef __I2SCHAR_DEMO_H_
#define __I2SCHAR_DEMO_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <pthread.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Record memory define in w25q128 */
#define RECORD_LEN              (2 * 0x100000)	//(1024 * 1024)
#define RECORD_ADDR             (0x00200000)
#define RECORD_END              (RECORD_ADDR + RECORD_LEN)

/* Record memory size in a frame*/
#define RECORD_MEM_SIZE         (48 * 1024)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public functions
 ****************************************************************************/
void esp32_i2schar_install(void);

#endif							/* __I2SCHAR_DEMO_H_ */
