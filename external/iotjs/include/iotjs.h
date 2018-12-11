/* Copyright 2015-present Samsung Electronics Co., Ltd. and other contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @defgroup IOTJS IOTJS
 * @ingroup IOTJS
 * @{
 */

/**
 * @file iotjs.h
 * @brief IoT.js is Open Source software under the Apache 2.0 license. Complete license and copyright information can be found within the code.\n\n
 * Official website : http://iotjs.net\n
 * Online API Reference : https://github.com/Samsung/iotjs/blob/master/docs/api/IoT.js-API-reference.md\n
 */

#ifndef IOTJS_IOTJS_H
#define IOTJS_IOTJS_H


#ifdef __cplusplus
#define IOTJS_EXTERN_C extern "C"
#else /* !__cplusplus */
#define IOTJS_EXTERN_C extern
#endif /* !__cplusplus */


IOTJS_EXTERN_C int iotjs_entry(int argc, char** argv);


#endif /* IOTJS_IOTJS_H */
/** @} */ // end of IOTJS group
