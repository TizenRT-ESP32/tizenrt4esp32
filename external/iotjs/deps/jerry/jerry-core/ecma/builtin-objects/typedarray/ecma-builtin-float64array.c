/* Copyright JS Foundation and other contributors, http://js.foundation
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

#include "ecma-builtins.h"
#include "ecma-exceptions.h"
#include "ecma-gc.h"
#include "ecma-globals.h"
#include "ecma-helpers.h"
#include "ecma-typedarray-object.h"
#include "jrt.h"

#ifndef CONFIG_DISABLE_ES2015_TYPEDARRAY_BUILTIN
#if CONFIG_ECMA_NUMBER_TYPE == CONFIG_ECMA_NUMBER_FLOAT64

#define ECMA_BUILTINS_INTERNAL
#include "ecma-builtins-internal.h"

#define BUILTIN_INC_HEADER_NAME "ecma-builtin-float64array.inc.h"
#define BUILTIN_UNDERSCORED_ID float64array
#include "ecma-builtin-internal-routines-template.inc.h"

/** \addtogroup ecma ECMA
 * @{
 *
 * \addtogroup ecmabuiltins
 * @{
 *
 * \addtogroup float64array ECMA Float64Array object built-in
 * @{
 */

/**
 * Handle calling [[Call]] of Float64Array
 *
 * @return ecma value
 */
ecma_value_t
ecma_builtin_float64array_dispatch_call (const ecma_value_t *arguments_list_p, /**< arguments list */
                                         ecma_length_t arguments_list_len) /**< number of arguments */
{
  JERRY_ASSERT (arguments_list_len == 0 || arguments_list_p != NULL);

  return ecma_raise_type_error (ECMA_ERR_MSG ("Float64Array cannot be directly called"));
} /* ecma_builtin_float64array_dispatch_call */

/**
 * Handle calling [[Construct]] of Float64Array
 *
 * @return ecma value
 */
ecma_value_t
ecma_builtin_float64array_dispatch_construct (const ecma_value_t *arguments_list_p, /**< arguments list */
                                              ecma_length_t arguments_list_len) /**< number of arguments */
{
  JERRY_ASSERT (arguments_list_len == 0 || arguments_list_p != NULL);

  ecma_object_t *prototype_obj_p = ecma_builtin_get (ECMA_BUILTIN_ID_FLOAT64ARRAY_PROTOTYPE);
  ecma_value_t val = ecma_op_create_typedarray (arguments_list_p,
                                                arguments_list_len,
                                                prototype_obj_p,
                                                3,
                                                LIT_MAGIC_STRING_FLOAT64_ARRAY_UL);

  ecma_deref_object (prototype_obj_p);

  return val;
} /* ecma_builtin_float64array_dispatch_construct */

/**
  * @}
  * @}
  * @}
  */

#endif /* CONFIG_ECMA_NUMBER_TYPE == CONFIG_ECMA_NUMBER_FLOAT64 */
#endif /* !CONFIG_DISABLE_ES2015_TYPEDARRAY_BUILTIN */
