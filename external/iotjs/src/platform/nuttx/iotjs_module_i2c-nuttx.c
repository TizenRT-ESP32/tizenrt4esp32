/* Copyright 2016-present Samsung Electronics Co., Ltd. and other contributors
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

#if !defined(__NUTTX__)
#error "Module __FILE__ is for nuttx only"
#endif

#include <nuttx/i2c/i2c_master.h>

#include "iotjs_systemio-nuttx.h"

#include "modules/iotjs_module_i2c.h"


#define I2C_DEFAULT_FREQUENCY 400000

struct iotjs_i2c_platform_data_s {
  int device;
  struct i2c_master_s* i2c_master;
  struct i2c_config_s config;
};

void i2c_create_platform_data(iotjs_jhandler_t* jhandler, iotjs_i2c_t* i2c,
                              iotjs_i2c_platform_data_t** ppdata) {
  iotjs_i2c_platform_data_t* pdata = IOTJS_ALLOC(iotjs_i2c_platform_data_t);

  DJHANDLER_CHECK_ARGS(2, number, function);
  pdata->device = JHANDLER_GET_ARG(0, number);
  pdata->i2c_master = NULL;
  *ppdata = pdata;
}

void i2c_destroy_platform_data(iotjs_i2c_platform_data_t* pdata) {
  IOTJS_RELEASE(pdata);
}

#define I2C_WORKER_INIT_TEMPLATE                                            \
  iotjs_i2c_reqwrap_t* req_wrap = iotjs_i2c_reqwrap_from_request(work_req); \
  iotjs_i2c_reqdata_t* req_data = iotjs_i2c_reqwrap_data(req_wrap);

#define IOTJS_I2C_METHOD_HEADER(arg)              \
  IOTJS_VALIDATED_STRUCT_METHOD(iotjs_i2c_t, arg) \
  iotjs_i2c_platform_data_t* platform_data = _this->platform_data;


void I2cSetAddress(iotjs_i2c_t* i2c, uint8_t address) {
  IOTJS_I2C_METHOD_HEADER(i2c);
  platform_data->config.address = address;
  platform_data->config.addrlen = 7;
}

void OpenWorker(uv_work_t* work_req) {
  I2C_WORKER_INIT_TEMPLATE;
  iotjs_i2c_t* i2c = iotjs_i2c_instance_from_reqwrap(req_wrap);

  IOTJS_I2C_METHOD_HEADER(i2c);
  platform_data->i2c_master = iotjs_i2c_config_nuttx(platform_data->device);
  if (!platform_data->i2c_master) {
    DLOG("I2C OpenWorker : cannot open");
    req_data->error = kI2cErrOpen;
    return;
  }

  platform_data->config.frequency = I2C_DEFAULT_FREQUENCY;

  req_data->error = kI2cErrOk;
}

void I2cClose(iotjs_i2c_t* i2c) {
  IOTJS_I2C_METHOD_HEADER(i2c);
  iotjs_i2c_unconfig_nuttx(platform_data->i2c_master);
}

void WriteWorker(uv_work_t* work_req) {
  I2C_WORKER_INIT_TEMPLATE;
  iotjs_i2c_t* i2c = iotjs_i2c_instance_from_reqwrap(req_wrap);
  IOTJS_I2C_METHOD_HEADER(i2c);

  uint8_t len = req_data->buf_len;
  uint8_t* data = (uint8_t*)req_data->buf_data;

  IOTJS_ASSERT(platform_data->i2c_master);
  IOTJS_ASSERT(len > 0);

  int ret =
      i2c_write(platform_data->i2c_master, &platform_data->config, data, len);
  if (ret < 0) {
    DLOG("I2C WriteWorker : cannot write - %d", ret);
    req_data->error = kI2cErrWrite;
  } else {
    req_data->error = kI2cErrOk;
  }

  if (req_data->buf_data != NULL) {
    iotjs_buffer_release(req_data->buf_data);
  }
}

void ReadWorker(uv_work_t* work_req) {
  I2C_WORKER_INIT_TEMPLATE;
  iotjs_i2c_t* i2c = iotjs_i2c_instance_from_reqwrap(req_wrap);
  IOTJS_I2C_METHOD_HEADER(i2c);

  uint8_t len = req_data->buf_len;
  req_data->buf_data = iotjs_buffer_allocate(len);

  IOTJS_ASSERT(platform_data->i2c_master);
  IOTJS_ASSERT(len > 0);

  int ret = i2c_read(platform_data->i2c_master, &platform_data->config,
                     (uint8_t*)req_data->buf_data, len);
  if (ret != 0) {
    DLOG("I2C ReadWorker : cannot read - %d", ret);
    req_data->error = kI2cErrRead;
    return;
  }
  req_data->error = kI2cErrOk;
}
