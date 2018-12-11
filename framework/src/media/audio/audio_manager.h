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

/**
 * @file audio_manager.h
 * @brief All macros, structures and functions to manager audio devices.
 */

#ifndef __AUDIO_MANAGER_H
#define __AUDIO_MANAGER_H

#include <sys/time.h>
#include <stddef.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/
/**
 * @brief Result types of Audio Manager APIs such as FAIL, SUCCESS, or INVALID ARGS
 */
enum audio_manager_result_e {
	AUDIO_MANAGER_DEVICE_NOT_SUPPORT = -9,
	AUDIO_MANAGER_RESAMPLE_FAIL = -8,
	AUDIO_MANAGER_DEVICE_FAIL = -7,
	AUDIO_MANAGER_CARD_NOT_READY = -6,
	AUDIO_MANAGER_XRUN_STATE = -5,
	AUDIO_MANAGER_INVALID_PARAM = -4,
	AUDIO_MANAGER_INVALID_DEVICE_NAME = -3,
	AUDIO_MANAGER_NO_AVAIL_CARD = -2,
	AUDIO_MANAGER_FAIL = -1,
	AUDIO_MANAGER_SUCCESS = 0
};

typedef enum audio_manager_result_e audio_manager_result_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: init_audio_stream_in
 *
 * Description:
 *   Find all available audio cards for input stream and initialize the
 *   mutexes of each card. The one of the audio cards is set as the active one.
 *
 * Return Value:
 *   On success, AUDIO_MANAGER_SUCCESS. Otherwise a negative value.
 ****************************************************************************/
audio_manager_result_t init_audio_stream_in(void);

/****************************************************************************
 * Name: init_audio_stream_out
 *
 * Description:
 *   Find all available audio cards for output stream and initialize the
 *   mutexes of the cards. The one of the audio cards is set as the active one.
 *
 * Return Value:
 *   On success, AUDIO_MANAGER_SUCCESS. Otherwise a negative value.
 ****************************************************************************/
audio_manager_result_t init_audio_stream_out(void);

/****************************************************************************
 * Name: set_audio_stream_in
 *
 * Description:
 *   Opening the pcm for the input stream and setup the status of the active
 *   input card. If the target sample rate is out of range from the sample rates
 *   supported by the active input audio card, a resampling flag is set.
 *
 * Input parameters:
 *   channels: number of channels
 *   sample_rate: sample rate with which the stream is operated
 *   format: audio file format to be streamed in
 *
 * Return Value:
 *   On success, AUDIO_MANAGER_SUCCESS. Otherwise a negative value.
 ****************************************************************************/
audio_manager_result_t set_audio_stream_in(unsigned int channels, unsigned int sample_rate, int format);

/****************************************************************************
 * Name: set_audio_stream_out
 *
 * Description:
 *   Opening the pcm for the output stream and setup the status of the active
 *   output card. If the target sample rate is out of range from the sample rates
 *   supported by the active output audio card, a resampling flag is set.
 *
 * Input parameters:
 *   channels: number of channels
 *   sample_rate: sample rate with which the stream is operated
 *   format: audio file format to be streamed out
 *
 * Return Value:
 *   On success, AUDIO_MANAGER_SUCCESS. Otherwise, a negative value.
 ****************************************************************************/
audio_manager_result_t set_audio_stream_out(unsigned int channels, unsigned int sample_rate, int format);

/****************************************************************************
 * Name: start_audio_stream_in
 *
 * Description:
 *   Read the specified number of frames from the input stream.
 *   If the input audio device have been paused, resume and proceed the reading.
 *   If the resampling flag is set, resamplings are performed for all target frames.
 *
 * Input parameters:
 *   data: buffer to get the frame data
 *   frames: number of frames to be read
 *
 * Return Value:
 *   On success, the number of frames read. Otherwise, a negative value.
 ****************************************************************************/
int start_audio_stream_in(void *data, unsigned int frames);

/****************************************************************************
 * Name: start_audio_stream_out
 *
 * Description:
 *   Write the specified frame data to the output stream.
 *   If the output audio device have been paused, resume and proceed the writing.
 *   If the resampling flag is set, resamplings are performed for all target frames.
 *
 * Input parameters:
 *   data: buffer to transfer the frame data
 *   frames: number of frames to be written
 *
 * Return Value:
 *   On success, the number of frames written. Otherwise, a negative value.
 ****************************************************************************/
int start_audio_stream_out(void *data, unsigned int frames);

/****************************************************************************
 * Name: pause_audio_stream_in
 *
 * Description:
 *   Pause the active input audio device.
 *   Note that only the active audio device currently running can be paused.
 *   If the device is resumed, the paused stream is continued.
 *
 * Return Value:
 *   On success, AUDIO_MANAGER_SUCCESS. Otherwise, a negative value.
 ****************************************************************************/
audio_manager_result_t pause_audio_stream_in(void);

/****************************************************************************
 * Name: pause_audio_stream_out
 *
 * Description:
 *   Pause the active output audio device.
 *	 Note that only the active audio device currently running can be paused.
 *	 If the device is resumed, the paused stream is continued.
 *
 * Return Value:
 *   On success, AUDIO_MANAGER_SUCCESS. Otherwise, a negative value.
 ****************************************************************************/
audio_manager_result_t pause_audio_stream_out(void);

/****************************************************************************
 * Name: stop_audio_stream_in
 *
 * Description:
 *   Stop the active input audio device.
 *   Note that only the active audio device currently running can be stopped.
 *	 Once the device is stopped, the stream should be restarted from the beginning
 *	 with calling set_audio_stream_in().
 *
 * Return Value:
 *   On success, AUDIO_MANAGER_SUCCESS. Otherwise, a negative value.
 ****************************************************************************/
audio_manager_result_t stop_audio_stream_in(void);

/****************************************************************************
 * Name: stop_audio_stream_out
 *
 * Description:
 *   Stop the active output audio device.
 *   Note that only the active audio device currently running can be stopped.
 *	 Once the device is stopped, the stream should be restarted from the beginning
 *	 with calling set_audio_stream_out().
 *
 * Return Value:
 *   On success, AUDIO_MANAGER_SUCCESS. Otherwise, a negative value.
 ****************************************************************************/
audio_manager_result_t stop_audio_stream_out(void);

/****************************************************************************
 * Name: reset_audio_stream_in
 *
 * Description:
 *   Close the active input audio stream.
 *   After the reset, the stream should be restarted from the beginning with
 *   calling set_audio_stream_in().
 *
 * Return Value:
 *   On success, AUDIO_MANAGER_SUCCESS. Otherwise, a negative value.
 ****************************************************************************/
audio_manager_result_t reset_audio_stream_in(void);

/****************************************************************************
 * Name: reset_audio_stream_out
 *
 * Description:
 *   Close the active output audio stream.
 *   After the reset, the stream should be restarted from the beginning with
 *   calling set_audio_stream_out().
 *
 * Return Value:
 *   On success, AUDIO_MANAGER_SUCCESS. Otherwise, a negative value.
 ****************************************************************************/
audio_manager_result_t reset_audio_stream_out(void);

/****************************************************************************
 * Name: get_input_frame_count
 *
 * Description:
 *   Get the frame size of the pcm buffer in the active input audio device.
 *
 * Return Value:
 *   On success, the size of the pcm buffer for input streams. Otherwise, 0.
 ****************************************************************************/
unsigned int get_input_frame_count(void);

/****************************************************************************
 * Name: get_input_frames_to_byte
 *
 * Description:
 *   Get the byte size of the given frame value in input stream.
 *
 * Input parameter:
 *   frames: the target of which byte size is returned.
 *
 * Return Value:
 *   On success, the byte size of the frame in input stream. Otherwise, 0.
 ****************************************************************************/
unsigned int get_input_frames_to_byte(unsigned int frames);

/****************************************************************************
 * Name: get_input_bytes_to_frame
 *
 * Description:
 *   Get the number of frames for the given byte size in input stream.
 *
 * Input parameter:
 *   bytes: the target of which frame count is returned.
 *
 * Return Value:
 *   On success, the number of frames in input stream. Otherwise, 0.
 ****************************************************************************/
unsigned int get_input_bytes_to_frame(unsigned int bytes);

/****************************************************************************
 * Name: get_output_frame_count
 *
 * Description:
 *   Get the frame size of the pcm buffer in the active output audio device.
 *
 * Return Value:
 *   On success, the size of the pcm buffer for output streams. Otherwise, 0.
 ****************************************************************************/
unsigned int get_output_frame_count(void);

/****************************************************************************
 * Name: get_output_frames_to_byte
 *
 * Description:
 *   Get the byte size of the given frame value in output stream.
 *
 * Input parameter:
 *   frames: the target of which byte size is returned.
 *
 * Return Value:
 *   On success, the byte size of the frame in output stream. Otherwise, 0.
 ****************************************************************************/
unsigned int get_output_frames_to_byte(unsigned int frames);

/****************************************************************************
 * Name: get_output_bytes_to_frame
 *
 * Description:
 *   Get the number of frames for the given byte size in output stream.
 *
 * Input parameter:
 *   bytes: the target of which frame count is returned.
 *
 * Return Value:
 *   On success, the number of frames in output stream. Otherwise, 0.
 ****************************************************************************/
unsigned int get_output_bytes_to_frame(unsigned int bytes);

/****************************************************************************
 * Name: get_max_audio_volume
 *
 * Description:
 *   Get the maximum volume of the active output audio device.
 *
 * Input parameter:
 *   volume: the pointer to get the maximum volume value
 *
 * Return Value:
 *   AUDIO_MANAGER_SUCCESS only if the maximum volume is obtained successfully
 *   from the current device. Otherwise, a negative value.
 ****************************************************************************/
audio_manager_result_t get_max_audio_volume(uint8_t *volume);

/****************************************************************************
 * Name: get_input_audio_gain
 *
 * Input parameter:
 *   gain: the pointer to get the current gain value
 *
 * Return Value:
 *   AUDIO_MANAGER_SUCCESS only if the gain is obtained successfully
 *   from the current device. Otherwise, a negative value.
 ****************************************************************************/
audio_manager_result_t get_input_audio_gain(uint8_t *gain);

/****************************************************************************
 * Name: get_output_audio_volume
 *
 * Input parameter:
 *   volume: the pointer to get the current volume value
 *
 * Return Value:
 *   AUDIO_MANAGER_SUCCESS only if the volume is obtained successfully
 *   from the current device. Otherwise, a negative value.
 ****************************************************************************/
audio_manager_result_t get_output_audio_volume(uint8_t *volume);

/****************************************************************************
 * Name: set_input_audio_gain
 *
 * Description:
 *   Adjust the gain level of the active input audio device.
 *
 * Input parameter:
 *   gain: gain value to set, Min = 0, Max = get_max_audio_gain()
 *
 * Return Value:
 *   On success, AUDIO_MANAGER_SUCCESS. Otherwise, a negative value.
 ****************************************************************************/
audio_manager_result_t set_input_audio_gain(uint8_t gain);

/****************************************************************************
 * Name: set_output_audio_volume
 *
 * Description:
 *   Adjust the volume level of the active output audio device.
 *
 * Input parameter:
 *   volume: volume value to set, Min = 0, Max = get_max_audio_volume()
 *
 * Return Value:
 *   On success, AUDIO_MANAGER_SUCCESS. Otherwise, a negative value.
 ****************************************************************************/
audio_manager_result_t set_output_audio_volume(uint8_t volume);

#if defined(__cplusplus)
}								/* extern "C" */
#endif
#endif
