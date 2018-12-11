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

#include "ecma-alloc.h"
#include "ecma-globals.h"
#include "ecma-gc.h"
#include "ecma-lcache.h"
#include "jrt.h"
#include "jmem.h"

JERRY_STATIC_ASSERT (sizeof (ecma_property_value_t) == sizeof (ecma_value_t),
                     size_of_ecma_property_value_t_must_be_equal_to_size_of_ecma_value_t);
JERRY_STATIC_ASSERT (((sizeof (ecma_property_value_t) - 1) & sizeof (ecma_property_value_t)) == 0,
                     size_of_ecma_property_value_t_must_be_power_of_2);

JERRY_STATIC_ASSERT (sizeof (ecma_string_t) == sizeof (uint64_t),
                     size_of_ecma_string_t_must_be_less_than_or_equal_to_8_bytes);

JERRY_STATIC_ASSERT (sizeof (ecma_extended_object_t) - sizeof (ecma_object_t) <= sizeof (uint64_t),
                     size_of_ecma_extended_object_part_must_be_less_than_or_equal_to_8_bytes);

/** \addtogroup ecma ECMA
 * @{
 *
 * \addtogroup ecmaalloc Routines for allocation/freeing memory for ECMA data types
 * @{
 */

/**
 * Implementation of routines for allocation/freeing memory for ECMA data types.
 *
 * All allocation routines from this module have the same structure:
 *  1. Try to allocate memory.
 *  2. If allocation was successful, return pointer to the allocated block.
 *  3. Run garbage collection.
 *  4. Try to allocate memory.
 *  5. If allocation was successful, return pointer to the allocated block;
 *     else - shutdown engine.
 */

/**
 * Template of an allocation routine.
 */
#define ALLOC(ecma_type) ecma_ ## ecma_type ## _t * \
  ecma_alloc_ ## ecma_type (void) \
{ \
  ecma_ ## ecma_type ## _t *ecma_type ## _p; \
  ecma_type ## _p = (ecma_ ## ecma_type ## _t *) jmem_pools_alloc (sizeof (ecma_ ## ecma_type ## _t)); \
  \
  JERRY_ASSERT (ecma_type ## _p != NULL); \
  \
  return ecma_type ## _p; \
}

/**
 * Deallocation routine template
 */
#define DEALLOC(ecma_type) void \
  ecma_dealloc_ ## ecma_type (ecma_ ## ecma_type ## _t *ecma_type ## _p) \
{ \
  jmem_pools_free ((uint8_t *) ecma_type ## _p, sizeof (ecma_ ## ecma_type ## _t)); \
}

/**
 * Declaration of alloc/free routine for specified ecma-type.
 */
#define DECLARE_ROUTINES_FOR(ecma_type) \
  ALLOC (ecma_type) \
  DEALLOC (ecma_type)

DECLARE_ROUTINES_FOR (number)
DECLARE_ROUTINES_FOR (collection_header)
DECLARE_ROUTINES_FOR (collection_chunk)

/**
 * Allocate memory for ecma-object
 *
 * @return pointer to allocated memory
 */
inline ecma_object_t * __attr_always_inline___
ecma_alloc_object (void)
{
#ifdef JMEM_STATS
  jmem_stats_allocate_object_bytes (sizeof (ecma_object_t));
#endif /* JMEM_STATS */

  return (ecma_object_t *) jmem_pools_alloc (sizeof (ecma_object_t));
} /* ecma_alloc_object */

/**
 * Dealloc memory from an ecma-object
 */
inline void __attr_always_inline___
ecma_dealloc_object (ecma_object_t *object_p) /**< object to be freed */
{
#ifdef JMEM_STATS
  jmem_stats_free_object_bytes (sizeof (ecma_object_t));
#endif /* JMEM_STATS */

  jmem_pools_free (object_p, sizeof (ecma_object_t));
} /* ecma_dealloc_object */

/**
 * Allocate memory for extended object
 *
 * @return pointer to allocated memory
 */
inline ecma_extended_object_t * __attr_always_inline___
ecma_alloc_extended_object (size_t size) /**< size of object */
{
#ifdef JMEM_STATS
  jmem_stats_allocate_object_bytes (size);
#endif /* JMEM_STATS */

  return jmem_heap_alloc_block (size);
} /* ecma_alloc_extended_object */

/**
 * Dealloc memory of an extended object
 */
inline void __attr_always_inline___
ecma_dealloc_extended_object (ecma_extended_object_t *ext_object_p, /**< property pair to be freed */
                              size_t size) /**< size of object */
{
#ifdef JMEM_STATS
  jmem_stats_free_object_bytes (size);
#endif /* JMEM_STATS */

  jmem_heap_free_block (ext_object_p, size);
} /* ecma_dealloc_extended_object */

/**
 * Allocate memory for ecma-string descriptor
 *
 * @return pointer to allocated memory
 */
inline ecma_string_t * __attr_always_inline___
ecma_alloc_string (void)
{
#ifdef JMEM_STATS
  jmem_stats_allocate_string_bytes (sizeof (ecma_string_t));
#endif /* JMEM_STATS */

  return (ecma_string_t *) jmem_pools_alloc (sizeof (ecma_string_t));
} /* ecma_alloc_string */

/**
 * Dealloc memory from ecma-string descriptor
 */
inline void __attr_always_inline___
ecma_dealloc_string (ecma_string_t *string_p) /**< string to be freed */
{
#ifdef JMEM_STATS
  jmem_stats_free_string_bytes (sizeof (ecma_string_t));
#endif /* JMEM_STATS */

  jmem_pools_free (string_p, sizeof (ecma_string_t));
} /* ecma_dealloc_string */

/**
 * Allocate memory for string with character data
 *
 * @return pointer to allocated memory
 */
inline ecma_string_t * __attr_always_inline___
ecma_alloc_string_buffer (size_t size) /**< size of string */
{
#ifdef JMEM_STATS
  jmem_stats_allocate_string_bytes (size);
#endif /* JMEM_STATS */

  return jmem_heap_alloc_block (size);
} /* ecma_alloc_string_buffer */

/**
 * Dealloc memory of a string with character data
 */
inline void __attr_always_inline___
ecma_dealloc_string_buffer (ecma_string_t *string_p, /**< string with data */
                            size_t size) /**< size of string */
{
#ifdef JMEM_STATS
  jmem_stats_free_string_bytes (size);
#endif /* JMEM_STATS */

  jmem_heap_free_block (string_p, size);
} /* ecma_dealloc_string_buffer */

/**
 * Allocate memory for getter-setter pointer pair
 *
 * @return pointer to allocated memory
 */
inline ecma_getter_setter_pointers_t * __attr_always_inline___
ecma_alloc_getter_setter_pointers (void)
{
#ifdef JMEM_STATS
  jmem_stats_allocate_property_bytes (sizeof (ecma_property_pair_t));
#endif /* JMEM_STATS */

  return (ecma_getter_setter_pointers_t *) jmem_pools_alloc (sizeof (ecma_getter_setter_pointers_t));
} /* ecma_alloc_getter_setter_pointers */

/**
 * Dealloc memory from getter-setter pointer pair
 */
inline void __attr_always_inline___
ecma_dealloc_getter_setter_pointers (ecma_getter_setter_pointers_t *getter_setter_pointers_p) /**< pointer pair
                                                                                                * to be freed */
{
#ifdef JMEM_STATS
  jmem_stats_free_property_bytes (sizeof (ecma_property_pair_t));
#endif /* JMEM_STATS */

  jmem_pools_free (getter_setter_pointers_p, sizeof (ecma_getter_setter_pointers_t));
} /* ecma_dealloc_getter_setter_pointers */

/**
 * Allocate memory for ecma-property pair
 *
 * @return pointer to allocated memory
 */
inline ecma_property_pair_t * __attr_always_inline___
ecma_alloc_property_pair (void)
{
#ifdef JMEM_STATS
  jmem_stats_allocate_property_bytes (sizeof (ecma_property_pair_t));
#endif /* JMEM_STATS */

  return jmem_heap_alloc_block (sizeof (ecma_property_pair_t));
} /* ecma_alloc_property_pair */

/**
 * Dealloc memory of an ecma-property
 */
inline void __attr_always_inline___
ecma_dealloc_property_pair (ecma_property_pair_t *property_pair_p) /**< property pair to be freed */
{
#ifdef JMEM_STATS
  jmem_stats_free_property_bytes (sizeof (ecma_property_pair_t));
#endif /* JMEM_STATS */

  jmem_heap_free_block (property_pair_p, sizeof (ecma_property_pair_t));
} /* ecma_dealloc_property_pair */

/**
 * @}
 * @}
 */
