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

#include "ecma-array-object.h"
#include "ecma-exceptions.h"
#include "ecma-function-object.h"
#include "ecma-gc.h"
#include "ecma-globals.h"
#include "ecma-number-object.h"
#include "ecma-promise-object.h"

#ifndef CONFIG_DISABLE_ES2015_PROMISE_BUILTIN

#define ECMA_BUILTINS_INTERNAL
#include "ecma-builtins-internal.h"

#define BUILTIN_INC_HEADER_NAME "ecma-builtin-promise.inc.h"
#define BUILTIN_UNDERSCORED_ID promise
#include "ecma-builtin-internal-routines-template.inc.h"

/** \addtogroup ecma ECMA
 * @{
 *
 * \addtogroup ecmabuiltins
 * @{
 *
 * \addtogroup promise ECMA Promise object built-in
 * @{
 */

/**
 * The common function for 'reject' and 'resolve'.
 *
 * @return ecma value
 *         Returned value must be freed with ecma_free_value.
 */
static ecma_value_t
ecma_builtin_promise_reject_or_resolve (ecma_value_t this_arg, /**< "this" argument */
                                        ecma_value_t argument, /**< argument for reject or resolve */
                                        bool is_resolve) /**< whether it is for resolve routine */
{
  if (!ecma_is_value_object (this_arg))
  {
    return ecma_raise_type_error (ECMA_ERR_MSG ("'this' is not an object."));
  }

  uint8_t builtin_id = ecma_get_object_builtin_id (ecma_get_object_from_value (this_arg));

  if (builtin_id != ECMA_BUILTIN_ID_PROMISE)
  {
    return ecma_raise_type_error (ECMA_ERR_MSG ("'this' is not the Promise constructor."));
  }

  if (is_resolve
      && ecma_is_value_object (argument)
      && ecma_is_promise (ecma_get_object_from_value (argument)))
  {
    return ecma_copy_value (argument);
  }


  ecma_value_t capability = ecma_promise_new_capability ();

  if (ECMA_IS_VALUE_ERROR (capability))
  {
    return capability;
  }

  ecma_string_t *str;

  if (is_resolve)
  {
    str = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_RESOLVE);
  }
  else
  {
    str = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_REJECT);
  }

  ecma_value_t func = ecma_op_object_get (ecma_get_object_from_value (capability), str);
  ecma_deref_ecma_string (str);

  ecma_value_t call_ret = ecma_op_function_call (ecma_get_object_from_value (func),
                                                 ecma_make_simple_value (ECMA_SIMPLE_VALUE_UNDEFINED),
                                                 &argument,
                                                 1);

  ecma_free_value (func);

  if (ECMA_IS_VALUE_ERROR (call_ret))
  {
    return call_ret;
  }

  ecma_free_value (call_ret);

  ecma_string_t *str_promise = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_PROMISE);
  ecma_value_t promise_new = ecma_op_object_get (ecma_get_object_from_value (capability), str_promise);
  ecma_deref_ecma_string (str_promise);
  ecma_free_value (capability);

  return promise_new;
} /* ecma_builtin_promise_reject_or_resolve */

/**
 * Reject the promise if the value is error.
 *
 * See also:
 *         ES2015 25.4.1.1.1
 *
 * @return ecma value of the new promise.
 *         Returned value must be freed with ecma_free_value.
 */
inline static ecma_value_t
ecma_builtin_promise_reject_abrupt (ecma_value_t abrupt_value,
                                    ecma_value_t capability)
{
  ecma_value_t reason = ecma_get_value_from_error_value (abrupt_value);
  ecma_string_t *str_reject = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_REJECT);
  ecma_value_t reject = ecma_op_object_get (ecma_get_object_from_value (capability), str_reject);
  ecma_deref_ecma_string (str_reject);

  ecma_value_t call_ret = ecma_op_function_call (ecma_get_object_from_value (reject),
                                                 ecma_make_simple_value (ECMA_SIMPLE_VALUE_UNDEFINED),
                                                 &reason,
                                                 1);
  ecma_free_value (reject);

  if (ECMA_IS_VALUE_ERROR (call_ret))
  {
    return call_ret;
  }

  ecma_free_value (call_ret);

  ecma_string_t *str_promise = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_PROMISE);
  ecma_value_t promise_new = ecma_op_object_get (ecma_get_object_from_value (capability), str_promise);
  ecma_deref_ecma_string (str_promise);

  return promise_new;
} /* ecma_builtin_promise_reject_abrupt */

/**
 * The Promise.reject routine.
 *
 * See also:
 *         ES2015 25.4.4.4
 *
 * @return ecma value of the new promise.
 *         Returned value must be freed with ecma_free_value.
 */
static ecma_value_t
ecma_builtin_promise_reject (ecma_value_t this_arg, /**< 'this' argument */
                             ecma_value_t reason) /**< the reason for reject */
{
  return ecma_builtin_promise_reject_or_resolve (this_arg, reason, false);
} /* ecma_builtin_promise_reject */

/**
 * The Promise.resolve routine.
 *
 * See also:
 *         ES2015 25.4.4.5
 *
 * @return ecma value of the new promise.
 *         Returned value must be freed with ecma_free_value.
 */
static ecma_value_t
ecma_builtin_promise_resolve (ecma_value_t this_arg, /**< 'this' argument */
                              ecma_value_t argument) /**< the argument for resolve */
{
  return ecma_builtin_promise_reject_or_resolve (this_arg, argument, true);
} /* ecma_builtin_promise_resolve */

/**
 * Runtime Semantics: PerformPromiseRace.
 *
 * See also:
 *         ES2015 25.4.4.3.1
 *
 * @return ecma value of the new promise.
 *         Returned value must be freed with ecma_free_value.
 */
inline static ecma_value_t
ecma_builtin_promise_do_race (ecma_value_t array, /**< the array for race */
                              ecma_value_t capability, /**< PromiseCapability record */
                              ecma_value_t ctor) /**< the caller of Promise.race */
{
  JERRY_ASSERT (ecma_is_value_object (capability)
                && ecma_is_value_object (array)
                && ecma_is_value_object (ctor));
  JERRY_ASSERT (ecma_get_object_builtin_id (ecma_get_object_from_value (ctor)) == ECMA_BUILTIN_ID_PROMISE);
  JERRY_ASSERT (ecma_get_object_type (ecma_get_object_from_value (array)) == ECMA_OBJECT_TYPE_ARRAY);

  ecma_value_t ret = ecma_make_simple_value (ECMA_SIMPLE_VALUE_EMPTY);
  ecma_string_t *magic_string_length_p = ecma_new_ecma_length_string ();
  ecma_object_t *array_p = ecma_get_object_from_value (array);
  ecma_value_t len_value = ecma_op_object_get (array_p, magic_string_length_p);
  ecma_deref_ecma_string (magic_string_length_p);
  ecma_length_t len = (ecma_length_t) ecma_get_integer_from_value (len_value);
  ecma_fast_free_value (len_value);

  ecma_string_t *str_promise = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_PROMISE);
  ecma_string_t *str_resolve = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_RESOLVE);
  ecma_string_t *str_reject = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_REJECT);

  ecma_value_t resolve = ecma_op_object_get (ecma_get_object_from_value (capability),
                                             str_resolve);
  ecma_value_t reject = ecma_op_object_get (ecma_get_object_from_value (capability),
                                            str_reject);

  for (ecma_length_t index = 0; index <= len; index++)
  {
    /* b-d. */
    if (index == len)
    {
      ret = ecma_op_object_get (ecma_get_object_from_value (capability), str_promise);
      break;
    }

    /* e. */
    ecma_string_t *str_index = ecma_new_ecma_string_from_uint32 (index);
    ecma_value_t array_item = ecma_op_object_get (array_p, str_index);
    ecma_deref_ecma_string (str_index);

    /* h. */
    ecma_value_t next_promise = ecma_builtin_promise_resolve (ctor, array_item);
    ecma_free_value (array_item);

    /* i. */
    if (ECMA_IS_VALUE_ERROR (next_promise))
    {
      ret = next_promise;
      break;
    }

    /* j. */
    ecma_value_t then_result = ecma_promise_then (next_promise, resolve, reject);
    ecma_free_value (next_promise);

    /* k. */
    if (ECMA_IS_VALUE_ERROR (then_result))
    {
      ret = then_result;
      break;
    }

    ecma_free_value (then_result);
  }

  ecma_free_value (reject);
  ecma_free_value (resolve);
  ecma_deref_ecma_string (str_promise);
  ecma_deref_ecma_string (str_resolve);
  ecma_deref_ecma_string (str_reject);

  JERRY_ASSERT (!ecma_is_value_empty (ret));

  return ret;
} /* ecma_builtin_promise_do_race */

/**
 * Helper function for increase or decrease the remaining count.
 *
 * @return the current remaining count after increase or decrease.
 */
static ecma_length_t
ecma_builtin_promise_remaining_inc_or_dec (ecma_value_t remaining, /**< the remaining count */
                                           bool is_inc) /**< whether to increase the count */
{
  JERRY_ASSERT (ecma_is_value_object (remaining));

  ecma_object_t *remaining_p = ecma_get_object_from_value (remaining);
  ecma_extended_object_t *ext_object_p = (ecma_extended_object_t *) remaining_p;

  JERRY_ASSERT (ext_object_p->u.class_prop.class_id == LIT_MAGIC_STRING_NUMBER_UL);

  JERRY_ASSERT (ecma_is_value_integer_number (ext_object_p->u.class_prop.u.value));

  ecma_length_t current = (ecma_length_t) ecma_get_integer_from_value (ext_object_p->u.class_prop.u.value);

  if (is_inc)
  {
    current++;
  }
  else
  {
    current--;
  }

  ext_object_p->u.class_prop.u.value = ecma_make_uint32_value (current);

  return current;
} /* ecma_builtin_promise_remaining_inc_or_dec */

/**
 * Native handler for Promise.all Resolve Element Function.
 *
 * See also:
 *         ES2015 25.4.4.1.2
 *
 * @return ecma value of undefined.
 */
static ecma_value_t
ecma_builtin_promise_all_handler (const ecma_value_t function, /**< the function itself */
                                  const ecma_value_t this, /**< this_arg of the function */
                                  const ecma_value_t argv[], /**< argument list */
                                  const ecma_length_t argc) /**< argument number */
{
  JERRY_UNUSED (this);
  JERRY_UNUSED (argc);

  ecma_value_t ret = ecma_make_simple_value (ECMA_SIMPLE_VALUE_UNDEFINED);
  /* 1. */
  ecma_object_t *function_p = ecma_get_object_from_value (function);
  ecma_string_t *str_already_called = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_ALREADY_CALLED);
  ecma_value_t already_called =  ecma_op_object_get (function_p, str_already_called);

  JERRY_ASSERT (ecma_is_value_boolean (already_called));

  /* 2. */
  if (ecma_is_value_true (already_called))
  {
    ecma_fast_free_value (already_called);
    ecma_deref_ecma_string (str_already_called);

    return ret;
  }

  /* 3. */
  ecma_op_object_put (function_p,
                      str_already_called,
                      ecma_make_boolean_value (true),
                      false);

  ecma_string_t *str_index = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_INDEX);
  ecma_string_t *str_value = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_VALUE);
  ecma_string_t *str_capability = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_CAPABILITY);
  ecma_string_t *str_remaining = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_REMAINING_ELEMENT);

  /* 4-7. */
  ecma_value_t index_val = ecma_op_object_get (function_p, str_index);
  ecma_value_t value_array = ecma_op_object_get (function_p, str_value);
  ecma_value_t capability = ecma_op_object_get (function_p, str_capability);
  ecma_value_t remaining = ecma_op_object_get (function_p, str_remaining);

  JERRY_ASSERT (ecma_is_value_integer_number (index_val));

  /* 8. */
  ecma_string_t *index_str = ecma_new_ecma_string_from_uint32 ((uint32_t) ecma_get_integer_from_value (index_val));

  ecma_op_object_put (ecma_get_object_from_value (value_array),
                      index_str,
                      argv[0],
                      false);
  ecma_deref_ecma_string (index_str);

  /* 9-10. */
  if (ecma_builtin_promise_remaining_inc_or_dec (remaining, false) == 0)
  {
    ecma_string_t *str_resolve = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_RESOLVE);
    ecma_value_t resolve = ecma_op_object_get (ecma_get_object_from_value (capability),
                                               str_resolve);
    ecma_deref_ecma_string (str_resolve);
    ret = ecma_op_function_call (ecma_get_object_from_value (resolve),
                                 ecma_make_simple_value (ECMA_SIMPLE_VALUE_UNDEFINED),
                                 &value_array,
                                 1);
    ecma_free_value (resolve);
  }

  ecma_free_value (remaining);
  ecma_free_value (capability);
  ecma_free_value (value_array);
  ecma_fast_free_value (index_val);
  ecma_fast_free_value (already_called);
  ecma_deref_ecma_string (str_already_called);
  ecma_deref_ecma_string (str_index);
  ecma_deref_ecma_string (str_value);
  ecma_deref_ecma_string (str_capability);
  ecma_deref_ecma_string (str_remaining);

  return ret;
} /* ecma_builtin_promise_all_handler */

/**
 * Runtime Semantics: PerformPromiseAll.
 *
 * See also:
 *         ES2015 25.4.4.1.1
 *
 * @return ecma value of the new promise.
 *         Returned value must be freed with ecma_free_value.
 */
inline static ecma_value_t
ecma_builtin_promise_do_all (ecma_value_t array, /**< the array for all */
                             ecma_value_t capability, /**< PromiseCapability record */
                             ecma_value_t ctor) /**< the caller of Promise.race */
{
  JERRY_ASSERT (ecma_is_value_object (capability)
                && ecma_is_value_object (array)
                && ecma_is_value_object (ctor));
  JERRY_ASSERT (ecma_get_object_builtin_id (ecma_get_object_from_value (ctor)) == ECMA_BUILTIN_ID_PROMISE);
  JERRY_ASSERT (ecma_get_object_type (ecma_get_object_from_value (array)) == ECMA_OBJECT_TYPE_ARRAY);

  ecma_value_t ret = ecma_make_simple_value (ECMA_SIMPLE_VALUE_EMPTY);
  ecma_string_t *magic_string_length_p = ecma_new_ecma_length_string ();
  ecma_object_t *array_p = ecma_get_object_from_value (array);
  ecma_value_t len_value = ecma_op_object_get (array_p, magic_string_length_p);
  ecma_deref_ecma_string (magic_string_length_p);
  ecma_length_t len = (ecma_length_t) ecma_get_integer_from_value (len_value);
  ecma_fast_free_value (len_value);

  ecma_string_t *str_promise = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_PROMISE);
  ecma_string_t *str_resolve = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_RESOLVE);
  ecma_string_t *str_reject = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_REJECT);
  ecma_string_t *str_already_called = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_ALREADY_CALLED);
  ecma_string_t *str_index = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_INDEX);
  ecma_string_t *str_value = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_VALUE);
  ecma_string_t *str_capability = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_CAPABILITY);
  ecma_string_t *str_remaining = ecma_new_ecma_string_from_uint32 (ECMA_PROMISE_PROPERTY_REMAINING_ELEMENT);

  ecma_value_t undefined_val = ecma_make_simple_value (ECMA_SIMPLE_VALUE_UNDEFINED);
  /* String '1' indicates [[Resolve]] and '2' indicates [[Reject]]. */
  ecma_value_t resolve = ecma_op_object_get (ecma_get_object_from_value (capability),
                                             str_resolve);
  ecma_value_t reject = ecma_op_object_get (ecma_get_object_from_value (capability),
                                            str_reject);
  /* 3. */
  ecma_value_t result_array_length_val = ecma_make_uint32_value (0);
  ecma_value_t value_array = ecma_op_create_array_object (&result_array_length_val, 1, true);
  ecma_free_value (result_array_length_val);
  /* 4. */
  ecma_value_t remaining = ecma_op_create_number_object (ecma_make_integer_value (1));
  /* 5. */
  ecma_length_t index = 0;

  /* 6 */
  while (true)
  {
    JERRY_ASSERT (index <= len);
    /* d. */
    if (index == len)
    {
      /* ii. */
      if (ecma_builtin_promise_remaining_inc_or_dec (remaining, false) == 0)
      {
        /* iii. */
        ecma_value_t resolve_ret = ecma_op_function_call (ecma_get_object_from_value (resolve),
                                                          ecma_make_simple_value (ECMA_SIMPLE_VALUE_UNDEFINED),
                                                          &value_array,
                                                          1);

        if (ECMA_IS_VALUE_ERROR (resolve_ret))
        {
          ret = resolve_ret;
          break;
        }
      }

      /* iv. */
      ret = ecma_op_object_get (ecma_get_object_from_value (capability), str_promise);
      break;
    }

    /* e. h. */
    ecma_string_t *index_str = ecma_new_ecma_string_from_uint32 (index);
    ecma_value_t array_item = ecma_op_object_get (array_p, index_str);
    ecma_op_object_put (ecma_get_object_from_value (value_array),
                        index_str,
                        undefined_val,
                        false);
    ecma_deref_ecma_string (index_str);
    /* i. */
    ecma_value_t next_promise = ecma_builtin_promise_resolve (ctor, array_item);
    ecma_free_value (array_item);

    /* j. */
    if (ECMA_IS_VALUE_ERROR (next_promise))
    {
      ret = next_promise;
      break;
    }

    /* k. */
    ecma_object_t *res_ele_p;
    res_ele_p = ecma_op_create_external_function_object (ecma_builtin_promise_all_handler);
    /* l. */
    ecma_op_object_put (res_ele_p,
                        str_already_called,
                        ecma_make_boolean_value (false),
                        false);
    /* m. */
    ecma_op_object_put (res_ele_p,
                        str_index,
                        ecma_make_uint32_value (index),
                        false);
    /* n. */
    ecma_op_object_put (res_ele_p,
                        str_value,
                        value_array,
                        false);
    /* o. */
    ecma_op_object_put (res_ele_p,
                        str_capability,
                        capability,
                        false);
    /* p. */
    ecma_op_object_put (res_ele_p,
                        str_remaining,
                        remaining,
                        false);

    /* q. */
    ecma_builtin_promise_remaining_inc_or_dec (remaining, true);
    /* r. */
    ecma_value_t then_result = ecma_promise_then (next_promise,
                                                  ecma_make_object_value (res_ele_p),
                                                  reject);
    ecma_deref_object (res_ele_p);
    ecma_free_value (next_promise);

    /* s. */
    if (ECMA_IS_VALUE_ERROR (then_result))
    {
      ret = then_result;
      break;
    }

    ecma_free_value (then_result);
    index ++;
  }

  ecma_free_value (reject);
  ecma_free_value (resolve);
  ecma_free_value (remaining);
  ecma_free_value (value_array);
  ecma_deref_ecma_string (str_already_called);
  ecma_deref_ecma_string (str_index);
  ecma_deref_ecma_string (str_value);
  ecma_deref_ecma_string (str_capability);
  ecma_deref_ecma_string (str_remaining);
  ecma_deref_ecma_string (str_resolve);
  ecma_deref_ecma_string (str_reject);
  ecma_deref_ecma_string (str_promise);

  JERRY_ASSERT (!ecma_is_value_empty (ret));

  return ret;
} /* ecma_builtin_promise_do_all */

/**
 * The common function for both Promise.race and Promise.all.
 *
 * @return ecma value of the new promise.
 *         Returned value must be freed with ecma_free_value.
 */
static ecma_value_t
ecma_builtin_promise_race_or_all (ecma_value_t this_arg, /**< 'this' argument */
                                  ecma_value_t array, /**< the items to be resolved */
                                  bool is_race) /**< indicates whether it is race function */
{
  if (!ecma_is_value_object (this_arg))
  {
    return ecma_raise_type_error (ECMA_ERR_MSG ("'this' is not an object."));
  }

  uint8_t builtin_id = ecma_get_object_builtin_id (ecma_get_object_from_value (this_arg));

  if (builtin_id != ECMA_BUILTIN_ID_PROMISE)
  {
    return ecma_raise_type_error (ECMA_ERR_MSG ("'this' is not the Promise constructor."));
  }

  ecma_value_t capability = ecma_promise_new_capability ();
  ecma_value_t ret = ecma_make_simple_value (ECMA_SIMPLE_VALUE_EMPTY);

  if (!ecma_is_value_object (array)
      || ecma_get_object_type (ecma_get_object_from_value (array)) != ECMA_OBJECT_TYPE_ARRAY)
  {
    ecma_value_t error = ecma_raise_type_error (ECMA_ERR_MSG ("Second argument is not an array."));
    ret = ecma_builtin_promise_reject_abrupt (error, capability);
    ecma_free_value (error);
    ecma_free_value (capability);

    return ret;
  }

  if (is_race)
  {
    ret = ecma_builtin_promise_do_race (array, capability, this_arg);
  }
  else
  {
    ret = ecma_builtin_promise_do_all (array, capability, this_arg);
  }

  if (ECMA_IS_VALUE_ERROR (ret))
  {
    ret = ecma_get_value_from_error_value (ret);
  }

  ecma_free_value (capability);

  return ret;
} /* ecma_builtin_promise_race_or_all */

/**
 * The Promise.race routine.
 *
 * See also:
 *         ES2015 25.4.4.3
 *
 * @return ecma value of the new promise.
 *         Returned value must be freed with ecma_free_value.
 */
static ecma_value_t
ecma_builtin_promise_race (ecma_value_t this_arg, /**< 'this' argument */
                           ecma_value_t array) /**< the items to be resolved */
{
  return ecma_builtin_promise_race_or_all (this_arg, array, true);
} /* ecma_builtin_promise_race */

/**
 * The Promise.all routine.
 *
 * See also:
 *         ES2015 25.4.4.1
 *
 * @return ecma value of the new promise.
 *         Returned value must be freed with ecma_free_value.
 */
static ecma_value_t
ecma_builtin_promise_all (ecma_value_t this_arg, /**< 'this' argument */
                           ecma_value_t array) /**< the items to be resolved */
{
  return ecma_builtin_promise_race_or_all (this_arg, array, false);
} /* ecma_builtin_promise_all */

/**
 * Handle calling [[Call]] of built-in Promise object.
 *
 * ES2015 25.4.3 Promise is not intended to be called
 * as a function and will throw an exception when called
 * in that manner.
 *
 * @return ecma value
 */
ecma_value_t
ecma_builtin_promise_dispatch_call (const ecma_value_t *arguments_list_p, /**< arguments list */
                                    ecma_length_t arguments_list_len) /**< number of arguments */
{
  JERRY_ASSERT (arguments_list_len == 0 || arguments_list_p != NULL);

  return ecma_raise_type_error (ECMA_ERR_MSG ("Constructor Promise requires 'new'."));
} /* ecma_builtin_promise_dispatch_call */

/**
 * Handle calling [[Construct]] of built-in Promise object.
 *
 * @return ecma value
 */
ecma_value_t
ecma_builtin_promise_dispatch_construct (const ecma_value_t *arguments_list_p, /**< arguments list */
                                         ecma_length_t arguments_list_len) /**< number of arguments */
{
  JERRY_ASSERT (arguments_list_len == 0 || arguments_list_p != NULL);

  if (arguments_list_len == 0 || !ecma_op_is_callable (arguments_list_p[0]))
  {
    return ecma_raise_type_error (ECMA_ERR_MSG ("First parameter must be callable."));
  }

  return ecma_op_create_promise_object (arguments_list_p[0], ECMA_PROMISE_EXECUTOR_FUNCTION);
} /* ecma_builtin_promise_dispatch_construct */

/**
 * @}
 * @}
 * @}
 */

#endif /* !CONFIG_DISABLE_ES2015_PROMISE_BUILTIN */
