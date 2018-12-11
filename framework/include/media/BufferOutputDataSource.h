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
 * @ingroup MEDIA
 * @{
 */

/**
 * @file media/BufferOutputDataSource.h
 * @brief Media BufferOutputDataSource APIs
 */

#ifndef __MEDIA_BUFFEROUTPUTDATASOURCE_H
#define __MEDIA_BUFFEROUTPUTDATASOURCE_H

#include <media/OutputDataSource.h>

namespace media {
namespace stream {
/**
 * @class
 * @brief This class is buffer output data structure
 * @details @b #include <media/BufferOutputDataSource.h>
 * @since TizenRT v2.0 PRE
 */
class BufferOutputDataSource : public OutputDataSource
{
public:
	/**
	 * @brief Constructs an empty BufferOutputDataSource.
	 * @details @b #include <media/BufferOutputDataSource.h>
	 * Delete the default construct
	 * @since TizenRT v2.0 PRE
	 */
	BufferOutputDataSource();
	/**
	 * @brief Constructs a new object provide with audio configuration
	 * @details @b #include <media/BufferOutputDataSource.h>
	 * param[in] channels   The channels that the channels of audio
	 * param[in] sampleRate The sampleRate that the sample rate of audio
	 * param[in] pcmFormat  The pcmFormat that the pcm format of audio
	 * param[in] callback   The callback that the callback function
	 * @since TizenRT v2.0 PRE
	 */
	BufferOutputDataSource(unsigned int channels, unsigned int sampleRate, audio_format_type_t pcmFormat);
	/**
	 * @brief Copy constructs for BufferOutputDataSource.
	 * @details @b #include <media/BufferOutputDataSource.h>
	 * @since TizenRT v2.0 PRE
	 */
	BufferOutputDataSource(const BufferOutputDataSource &source);
	/**
	 * @brief Operator= for BufferOutputDataSource.
	 * @details @b #include <media/BufferOutputDataSource.h>
	 * @since TizenRT v2.0 PRE
	 */
	BufferOutputDataSource &operator=(const BufferOutputDataSource &source);
	/**
	 * @brief Deconstructs an empty BufferOutputDataSource.
	 * @details @b #include <media/BufferOutputDataSource.h>
	 * @since TizenRT v2.0 PRE
	 */
	virtual ~BufferOutputDataSource();

	/**
	 * @brief Whether file is ready to be write.
	 * @details @b #include <media/BufferOutputDataSource.h>
	 * @return True.
	 * @since TizenRT v2.0 PRE
	 */
	bool isPrepare() override;
	/**
	 * @brief Open the file
	 * @details @b #include <media/BufferOutputDataSource.h>
	 * @return True.
	 * @since TizenRT v2.0 PRE
	 */
	bool open() override;
	/**
	 * @brief Close the file
	 * @details @b #include <media/BufferOutputDataSource.h>
	 * @return True.
	 * @since TizenRT v2.0 PRE
	 */
	bool close() override;

protected:
	ssize_t onStreamBufferReadable(bool isFlush) override;
};

} // namespace stream
} // namespace media

#endif
/** @} */ // end of MEDIA group
