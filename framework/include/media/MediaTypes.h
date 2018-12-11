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
 * @file media/MediaTypes.h
 * @brief Provide definiton of Media Type
 */

#ifndef __MEDIA_TYPES_H
#define __MEDIA_TYPES_H

#include <tinyalsa/tinyalsa.h>

namespace media {

/**
 * @brief Audio type.
 * @details
 * @since TizenRT v2.0 PRE
 */
typedef enum audio_type_e {
	/** Audio type is invalid */
	AUDIO_TYPE_INVALID = 0,
	/** Audio type is unknown */
	AUDIO_TYPE_UNKNOWN = AUDIO_TYPE_INVALID,
	/** Audio type is mp3 */
	AUDIO_TYPE_MP3 = 1,
	/** Audio type is AAC */
	AUDIO_TYPE_AAC = 2,
	/** Audio type is PCM */
	AUDIO_TYPE_PCM = 3,
	/** Audio type is OPUS */
	AUDIO_TYPE_OPUS = 4,
	/** Audio type is FLAC */
	AUDIO_TYPE_FLAC = 5
} audio_type_t;

/**
 * @class
 * @brief Audio format type, each value follows pcm_format in tinyalsa.
 * @details
 * @since TizenRT v2.0 PRE
 */
typedef enum audio_format_type_e {
	/* Signed 8 bit */
	AUDIO_FORMAT_TYPE_S8 = PCM_FORMAT_S8,
	/* Signed 16 bit use this as a default */
	AUDIO_FORMAT_TYPE_S16_LE = PCM_FORMAT_S16_LE,
	/* Signed 32 bit */
	AUDIO_FORMAT_TYPE_S32_LE = PCM_FORMAT_S32_LE
} audio_format_type_t;

} // namespace media

#endif
/** @} */ // end of MEDIA group
