/* ****************************************************************
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

/**
 * @defgroup MEDIA MEDIA
 * @brief Provides APIs for Media Framework 
 * @{
 */

/**
 * @file media/MediaRecorder.h
 * @brief Media MediaRecorder APIs
 */

#ifndef __MEDIA_MEDIARECORDER_H
#define __MEDIA_MEDIARECORDER_H

#include <memory>
#include <media/OutputDataSource.h>
#include <media/MediaRecorderObserverInterface.h>

namespace media {

class MediaRecorderImpl;

/**
 * @brief result of call the apis
 * @details @b #include <media/MediaRecorder.h>
 * @since TizenRT v2.0 PRE
 */
enum recorder_error_e : int {
	/** MediaRecorder Error case */
	RECORDER_ERROR_NOT_ALIVE = INT32_MIN,
	RECORDER_ERROR_INVALID_STATE,
	RECORDER_ERROR_INVALID_OPERATION,
	RECORDER_ERROR_INVALID_PARAM,
	RECORDER_ERROR_INTERNAL_OPERATION_FAILED,
	RECORDER_ERROR_FILE_OPEN_FAILED,
	RECORDER_ERROR_OUT_OF_MEMORY,
	/** MediaRecorder Success case */
	RECORDER_ERROR_NONE = 0
};

typedef enum recorder_error_e recorder_error_t;
const int RECORDER_OK = RECORDER_ERROR_NONE;
typedef int recorder_result_t;

/**
 * @class 
 * @brief This class implements the MediaRecorder capability agent.
 * @details @b #include <media/MediaRecorder.h>
 * @since TizenRT v2.0 PRE
 */
class MediaRecorder
{
public:
	/**
	 * @brief Constructs an empty MediaRecorder.
	 * @details @b #include <media/MediaRecorder.h>
	 * @since TizenRT v2.0 PRE
	 */
	MediaRecorder();
	
	/**
	 * @brief Deconstructs an empty MediaRecorder.
	 * @details @b #include <media/MediaRecorder.h>
	 * @since TizenRT v2.0 PRE
	 */
	~MediaRecorder();
	
	/**
	 * @brief Create MediaRecorder for capturing
	 * @details @b #include <media/MediaRecorder.h>
	 * This function is a synchronous api
	 * @return The result of the create operation
	 * @since TizenRT v2.0 PRE
	 */
	recorder_result_t create();
	
	/**
	 * @brief Destroy MediaRecorder
	 * @details @b #include <media/MediaRecorder.h>
	 * This function is a synchronous api
	 * @return The result of the destroy operation
	 * @since TizenRT v2.0 PRE
	 */
	recorder_result_t destroy();
	
	/**
	 * @brief Allocate and prepare resources related to the recorder, it should be called before start
	 * @details @b #include <media/MediaRecorder.h>
	 * This function is a synchronous api
	 * @return The result of the prepare operation
	 * @since TizenRT v2.0 PRE
	 */
	recorder_result_t prepare();
	
	/**
	 * @brief Releases allocated resources related to the recorder.
	 * @details @b #include <media/MediaRecorder.h>
	 * This function is a synchronous api
	 * @return The result of the unpreapre operation
	 * @since TizenRT v2.0 PRE
	 */
	recorder_result_t unprepare();
	
	/**
	 * @brief Start recording.
	 * @details @b #include <media/MediaRecorder.h>
	 * This function is a asynchronous api
	 * Order to MediaRecordWorker begin recording through the queue
	 * @return The result of the unpreapre operation
	 * @since TizenRT v2.0 PRE
	 */
	recorder_result_t start();
	
	/**
	 * @brief Pause recording.
	 * @details @b #include <media/MediaRecorder.h>
	 * This function is a asynchronous api
	 * Order to MediaRecordWorker pause recording through the queue
	 * @return The result of the pause operation
	 * @since TizenRT v2.0 PRE
	 */
	recorder_result_t pause();
	
	/**
	 * @brief Stop recording.
	 * @details @b #include <media/MediaRecorder.h>
	 * This function is a asynchronous api
	 * Order to MediaRecordWorker stop recording through the queue
	 * @return The result of the stop operation
	 * @since TizenRT v2.0 PRE
	 */
	recorder_result_t stop();
	
	/**
	 * @brief Gets the current volume
	 * @details @b #include <media/MediaRecorder.h>
	 * This function is a synchronous api
	 * @return The value of current mic volume
	 * @since TizenRT v2.0 PRE
	 */
	recorder_result_t getVolume(uint8_t *vol);
	
	/**
	 * @brief Sets the volume adjusted
	 * @details @b #include <media/MediaRecorder.h>
	 * This function is a synchronous api
	 * @param[in] vol The vol that the value of mic volume
	 * @return The result of setting the mic volume
	 * @since TizenRT v2.0 PRE
	 */
	recorder_result_t setVolume(uint8_t vol);
	
	/**
	 * @brief Sets the DataSource of output data
	 * @details @b #include <media/MediaRecorder.h>
	 * This function is a synchronous api
	 * @param[in] dataSource The dataSource that the config of output data
	 * @return The result of setting the datasource
	 * @since TizenRT v2.0 PRE
	 */
	recorder_result_t setDataSource(std::unique_ptr<stream::OutputDataSource> dataSource);
	
	/**
	 * @brief Sets the observer of MediaRecorder
	 * @details @b #include <media/MediaRecorder.h>
	 * This function is a synchronous api
	 * It sets the user's function
	 * @param[in] observer The callback to be set for Media Recorder Observer.
	 * @return The result of setting the observer
	 * @since TizenRT v2.0 PRE
	 */
	recorder_result_t setObserver(std::shared_ptr<MediaRecorderObserverInterface> observer);

	/**
	 * @brief Set limitation of recording time by given value(second), will be stopped when it reaches that.
	 * This should be called after setDataSource but before prepare
	 * @details @b #include <media/MediaRecorder.h>
	 * This function is a synchronous api
	 * It sets the user's function
 	 * @param[in] Max duration(second), No limitation If zero or negative.
	 * @return The result of setting the duration
	 * @since TizenRT v2.0 PRE
	 */
	recorder_result_t setDuration(int second);

	/**
	 * @brief MediaRecorder operator==
	 * @details @b #include <media/MediaRecorder.h>
	 * This function is a synchronous api
	 * Compares the MediaRecorder objects for equality
	 * @return The result of the compare operation for MediaRecorder object
	 * @since TizenRT v2.0 PRE
	 */
	bool operator==(const MediaRecorder& rhs);

private:
	std::shared_ptr<MediaRecorderImpl> mPMrImpl;
	uint64_t mId;
};
} // namespace media

#endif
/** @} */ // end of MEDIA group
