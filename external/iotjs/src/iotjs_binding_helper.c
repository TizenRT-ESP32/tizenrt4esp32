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

#include "iotjs_def.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


void iotjs_uncaught_exception(const iotjs_jval_t* jexception) {
  const iotjs_jval_t* process = iotjs_module_get(MODULE_PROCESS);

  iotjs_jval_t jonuncaughtexception =
      iotjs_jval_get_property(process, IOTJS_MAGIC_STRING__ONUNCAUGHTEXCEPTION);
  IOTJS_ASSERT(iotjs_jval_is_function(&jonuncaughtexception));

  iotjs_jargs_t args = iotjs_jargs_create(1);
  iotjs_jargs_append_jval(&args, jexception);

  bool throws;
  iotjs_jval_t jres =
      iotjs_jhelper_call(&jonuncaughtexception, process, &args, &throws);

  iotjs_jargs_destroy(&args);
  iotjs_jval_destroy(&jres);
  iotjs_jval_destroy(&jonuncaughtexception);

  if (throws) {
    iotjs_environment_t* env = iotjs_environment_get();

    if (!iotjs_environment_is_exiting(env)) {
      iotjs_set_process_exitcode(2);
      iotjs_environment_go_state_exiting(env);
    }
  }
}


void iotjs_process_emit_exit(int code) {
  const iotjs_jval_t* process = iotjs_module_get(MODULE_PROCESS);

  iotjs_jval_t jexit =
      iotjs_jval_get_property(process, IOTJS_MAGIC_STRING_EMITEXIT);
  IOTJS_ASSERT(iotjs_jval_is_function(&jexit));

  iotjs_jargs_t jargs = iotjs_jargs_create(1);
  iotjs_jargs_append_number(&jargs, code);

  bool throws;
  iotjs_jval_t jres = iotjs_jhelper_call(&jexit, process, &jargs, &throws);

  iotjs_jargs_destroy(&jargs);
  iotjs_jval_destroy(&jres);
  iotjs_jval_destroy(&jexit);

  if (throws) {
    iotjs_set_process_exitcode(2);
  }
}


// Calls next tick callbacks registered via `process.nextTick()`.
bool iotjs_process_next_tick() {
  iotjs_environment_t* env = iotjs_environment_get();

  if (iotjs_environment_is_exiting(env)) {
    return false;
  }

  const iotjs_jval_t* process = iotjs_module_get(MODULE_PROCESS);

  iotjs_jval_t jon_next_tick =
      iotjs_jval_get_property(process, IOTJS_MAGIC_STRING__ONNEXTTICK);
  IOTJS_ASSERT(iotjs_jval_is_function(&jon_next_tick));

  iotjs_jval_t jres =
      iotjs_jhelper_call_ok(&jon_next_tick, iotjs_jval_get_undefined(),
                            iotjs_jargs_get_empty());

  IOTJS_ASSERT(iotjs_jval_is_boolean(&jres));

  bool ret = iotjs_jval_as_boolean(&jres);
  iotjs_jval_destroy(&jres);
  iotjs_jval_destroy(&jon_next_tick);

  return ret;
}


// Make a callback for the given `function` with `this_` binding and `args`
// arguments. The next tick callbacks registered via `process.nextTick()`
// will be called after the callback function `function` returns.
void iotjs_make_callback(const iotjs_jval_t* jfunction,
                         const iotjs_jval_t* jthis,
                         const iotjs_jargs_t* jargs) {
  iotjs_jval_t result =
      iotjs_make_callback_with_result(jfunction, jthis, jargs);
  iotjs_jval_destroy(&result);
}


iotjs_jval_t iotjs_make_callback_with_result(const iotjs_jval_t* jfunction,
                                             const iotjs_jval_t* jthis,
                                             const iotjs_jargs_t* jargs) {
  // Calls back the function.
  bool throws;
  iotjs_jval_t jres = iotjs_jhelper_call(jfunction, jthis, jargs, &throws);
  if (throws) {
    iotjs_uncaught_exception(&jres);
  }

  // Calls the next tick callbacks.
  iotjs_process_next_tick();

  // Return value.
  return jres;
}


int iotjs_process_exitcode() {
  const iotjs_jval_t* process = iotjs_module_get(MODULE_PROCESS);

  iotjs_jval_t jexitcode =
      iotjs_jval_get_property(process, IOTJS_MAGIC_STRING_EXITCODE);
  IOTJS_ASSERT(iotjs_jval_is_number(&jexitcode));

  const int exitcode = (int)iotjs_jval_as_number(&jexitcode);
  iotjs_jval_destroy(&jexitcode);

  return exitcode;
}


void iotjs_set_process_exitcode(int code) {
  const iotjs_jval_t* process = iotjs_module_get(MODULE_PROCESS);
  iotjs_jval_set_property_number(process, IOTJS_MAGIC_STRING_EXITCODE, code);
}


const iotjs_jval_t* iotjs_init_process_module() {
  return iotjs_module_initialize_if_necessary(MODULE_PROCESS);
}
