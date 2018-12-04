// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __ESP_LOG_H__
#define __ESP_LOG_H__

#include <stdint.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

#undef nvdbg
#define nvdbg printf

#define ESP_LOGE( tag, format, ... ) nvdbg(format, ##__VA_ARGS__)
#define ESP_LOGW( tag, format, ... ) nvdbg(format, ##__VA_ARGS__)
#define ESP_LOGI( tag, format, ... ) nvdbg(format, ##__VA_ARGS__)
#define ESP_LOGD( tag, format, ... ) nvdbg(format, ##__VA_ARGS__)
#define ESP_LOGV( tag, format, ... ) nvdbg(format, ##__VA_ARGS__)

#endif /* __ESP_LOG_H__ */
