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
#ifndef __WM8960_H__
#define __WM8960_H__

/******************************************************************************/
/* 16-bits audio data size */
#define AUDIODATA_SIZE  2

/* Offset relative to WAV file header size   */
#define AUIDO_START_ADDRESS       58

/* Test data length */
#define WAVDATALEN	501700

/******************************************************************************/
extern const uint16_t demo_wav_data[WAVDATALEN];

/******************************************************************************/

int wm8960_open(void);
int wm8960_close(void);

uint8_t wm8960_write_reg(uint8_t reg, uint16_t dat);
uint16_t wm8960_read_reg(uint8_t reg);

int wm8960_play_init(void);
int wm8960_record_init(void);
int wm8960_loopback_init(void);

int wm8960_audio_play(uint16_t *pBuffer, uint32_t FullSize);

int wm8960_audio_record(uint16_t *pBuffer, uint32_t FullSize);

/******************************************************************************/
#endif
