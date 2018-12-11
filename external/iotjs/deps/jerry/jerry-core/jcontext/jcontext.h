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

/**
 * Memory context for JerryScript
 */
#ifndef JCONTEXT_H
#define JCONTEXT_H

#include "debugger.h"
#include "ecma-builtins.h"
#include "ecma-jobqueue.h"
#include "jerryscript-port.h"
#include "jmem.h"
#include "re-bytecode.h"
#include "vm-defines.h"
#include "jerryscript.h"

/** \addtogroup context Context
 * @{
 */

/**
 * First member of the jerry context
 */
#define JERRY_CONTEXT_FIRST_MEMBER ecma_builtin_objects

/**
 * User context item
 */
typedef struct jerry_context_data_header
{
  struct jerry_context_data_header *next_p; /**< pointer to next context item */
  const jerry_context_data_manager_t *manager_p; /**< manager responsible for deleting this item */
} jerry_context_data_header_t;

#define JERRY_CONTEXT_DATA_HEADER_USER_DATA(item_p) \
  ((uint8_t *) (item_p + 1))

/**
 * JerryScript context
 *
 * The purpose of this header is storing
 * all global variables for Jerry
 */
typedef struct
{
  /* Update JERRY_CONTEXT_FIRST_MEMBER if the first member changes */
  ecma_object_t *ecma_builtin_objects[ECMA_BUILTIN_ID__COUNT]; /**< pointer to instances of built-in objects */
#ifndef CONFIG_DISABLE_REGEXP_BUILTIN
  const re_compiled_code_t *re_cache[RE_CACHE_SIZE]; /**< regex cache */
#endif /* !CONFIG_DISABLE_REGEXP_BUILTIN */
  ecma_object_t *ecma_gc_objects_lists[ECMA_GC_COLOR__COUNT]; /**< List of marked (visited during
                                                               *   current GC session) and umarked objects */
  jmem_heap_free_t *jmem_heap_list_skip_p; /**< This is used to speed up deallocation. */
  jmem_pools_chunk_t *jmem_free_8_byte_chunk_p; /**< list of free eight byte pool chunks */
#ifdef JERRY_CPOINTER_32_BIT
  jmem_pools_chunk_t *jmem_free_16_byte_chunk_p; /**< list of free sixteen byte pool chunks */
#endif /* JERRY_CPOINTER_32_BIT */
  jmem_free_unused_memory_callback_t jmem_free_unused_memory_callback; /**< Callback for freeing up memory. */
  const lit_utf8_byte_t **lit_magic_string_ex_array; /**< array of external magic strings */
  const lit_utf8_size_t *lit_magic_string_ex_sizes; /**< external magic string lengths */
  ecma_lit_storage_item_t *string_list_first_p; /**< first item of the literal string list */
  ecma_lit_storage_item_t *number_list_first_p; /**< first item of the literal number list */
  ecma_object_t *ecma_global_lex_env_p; /**< global lexical environment */
  vm_frame_ctx_t *vm_top_context_p; /**< top (current) interpreter context */
  jerry_context_data_header_t *context_data_p; /**< linked list of user-provided context-specific pointers */
  size_t ecma_gc_objects_number; /**< number of currently allocated objects */
  size_t ecma_gc_new_objects; /**< number of newly allocated objects since last GC session */
  size_t jmem_heap_allocated_size; /**< size of allocated regions */
  size_t jmem_heap_limit; /**< current limit of heap usage, that is upon being reached,
                           *   causes call of "try give memory back" callbacks */
  uint32_t lit_magic_string_ex_count; /**< external magic strings count */
  uint32_t jerry_init_flags; /**< run-time configuration flags */
  uint8_t ecma_gc_visited_flip_flag; /**< current state of an object's visited flag */
  uint8_t is_direct_eval_form_call; /**< direct call from eval */
  uint8_t jerry_api_available; /**< API availability flag */

#ifndef CONFIG_ECMA_PROPERTY_HASHMAP_DISABLE
  uint8_t ecma_prop_hashmap_alloc_state; /**< property hashmap allocation state: 0-4,
                                          *   if !0 property hashmap allocation is disabled */
  bool ecma_prop_hashmap_alloc_last_is_hs_gc; /**< true, if and only if the last gc action was a high severity gc */
#endif /* !CONFIG_ECMA_PROPERTY_HASHMAP_DISABLE */

#ifndef CONFIG_DISABLE_REGEXP_BUILTIN
  uint8_t re_cache_idx; /**< evicted item index when regex cache is full (round-robin) */
#endif /* !CONFIG_DISABLE_REGEXP_BUILTIN */

#ifndef CONFIG_DISABLE_ES2015_PROMISE_BUILTIN
  ecma_job_queueitem_t *job_queue_head_p; /**< points to the head item of the jobqueue */
  ecma_job_queueitem_t *job_queue_tail_p; /**< points to the tail item of the jobqueue*/
#endif /* CONFIG_DISABLE_ES2015_PROMISE_BUILTIN */

#ifdef JERRY_VM_EXEC_STOP
  uint32_t vm_exec_stop_frequency; /**< reset value for vm_exec_stop_counter */
  uint32_t vm_exec_stop_counter; /**< down counter for reducing the calls of vm_exec_stop_cb */
  void *vm_exec_stop_user_p; /**< user pointer for vm_exec_stop_cb */
  ecma_vm_exec_stop_callback_t vm_exec_stop_cb; /**< user function which returns whether the
                                                 *   ECMAScript execution should be stopped */
#endif /* JERRY_VM_EXEC_STOP */

#ifdef JERRY_DEBUGGER
  uint8_t debugger_send_buffer[JERRY_DEBUGGER_MAX_BUFFER_SIZE]; /**< buffer for sending messages */
  uint8_t debugger_receive_buffer[JERRY_DEBUGGER_MAX_BUFFER_SIZE]; /**< buffer for receiving messages */
  vm_frame_ctx_t *debugger_stop_context; /**< stop only if the current context is equal to this context */
  jmem_cpointer_t debugger_byte_code_free_head; /**< head of byte code free linked list */
  jmem_cpointer_t debugger_byte_code_free_tail; /**< tail of byte code free linked list */
  uint8_t debugger_flags; /**< debugger flags */
  uint8_t debugger_message_delay; /**< call receive message when reaches zero */
  uint16_t debugger_receive_buffer_offset; /**< receive buffer offset */
  int debugger_connection; /**< holds the file descriptor of the socket communication */
  uint16_t debugger_port; /**< debugger socket communication port */
#endif /* JERRY_DEBUGGER */

#ifdef JMEM_STATS
  jmem_heap_stats_t jmem_heap_stats; /**< heap's memory usage statistics */
#endif /* JMEM_STATS */

#ifdef JERRY_VALGRIND_FREYA
  uint8_t valgrind_freya_mempool_request; /**< Tells whether a pool manager
                                           *   allocator request is in progress */
#endif /* JERRY_VALGRIND_FREYA */
} jerry_context_t;

#ifndef CONFIG_ECMA_LCACHE_DISABLE
/**
 * Hash table for caching the last access of properties.
 */
typedef struct
{
  ecma_lcache_hash_entry_t table[ECMA_LCACHE_HASH_ROWS_COUNT][ECMA_LCACHE_HASH_ROW_LENGTH];
} jerry_hash_table_t;
#endif /* !CONFIG_ECMA_LCACHE_DISABLE */

#ifdef JERRY_ENABLE_EXTERNAL_CONTEXT

#ifndef JERRY_GET_CURRENT_INSTANCE

/**
 * Default function if JERRY_GET_CURRENT_INSTANCE is not defined.
 */
#define JERRY_GET_CURRENT_INSTANCE() (jerry_port_get_current_instance ())

#endif /* !JERRY_GET_CURRENT_INSTANCE */

/**
 * This part is for Jerry which enable external context.
 */
typedef struct
{
  jmem_heap_free_t first; /**< first node in free region list */
  uint8_t area[]; /**< heap area */
} jmem_heap_t;

/**
 * Description of jerry instance which is the header of the context space.
 */
struct jerry_instance_t
{
  jerry_context_t context; /**< the context of the instance */
#ifndef JERRY_SYSTEM_ALLOCATOR
  jmem_heap_t *heap_p; /**< point to the heap aligned to JMEM_ALIGNMENT. */
  uint32_t heap_size; /**< size of the heap */
#endif
#ifndef CONFIG_ECMA_LCACHE_DISABLE
  uint8_t *lcache_p; /**< point to the entrance of the lcache in buffer */
#endif /* !CONFIG_ECMA_LCACHE_DISABLE */
};

#define JERRY_CONTEXT(field) (JERRY_GET_CURRENT_INSTANCE ()->context.field)

#ifndef JERRY_SYSTEM_ALLOCATOR

static inline jmem_heap_t * __attr_always_inline___
jerry_context_get_current_heap (void)
{
  return JERRY_GET_CURRENT_INSTANCE ()->heap_p;
} /* jerry_context_get_current_heap */

#define JERRY_HEAP_CONTEXT(field) (jerry_context_get_current_heap ()->field)

#ifdef JMEM_HEAP_SIZE
#error "JMEM_HEAP_SIZE must not be defined if JERRY_ENABLE_EXTERNAL_CONTEXT is defined"
#endif /* JMEM_HEAP_SIZE */

#define JMEM_HEAP_SIZE (JERRY_GET_CURRENT_INSTANCE ()->heap_size)

#define JMEM_HEAP_AREA_SIZE (JERRY_GET_CURRENT_INSTANCE ()->heap_size - JMEM_ALIGNMENT)

#endif /* !JERRY_SYSTEM_ALLOCATOR */

#ifndef CONFIG_ECMA_LCACHE_DISABLE

static inline jerry_hash_table_t * __attr_always_inline___
jerry_context_get_current_lcache (void)
{
  return (jerry_hash_table_t *) (JERRY_GET_CURRENT_INSTANCE ()->lcache_p);
} /* jerry_context_get_current_lcache */

#define JERRY_HASH_TABLE_CONTEXT(field) (jerry_context_get_current_lcache ()->field)

#endif /* !CONFIG_ECMA_LCACHE_DISABLE */

#else /* !JERRY_ENABLE_EXTERNAL_CONTEXT */

/**
 * This part is for Jerry which use default context.
 */

/**
 * Calculate heap area size, leaving space for a pointer to the free list
 */
#define JMEM_HEAP_AREA_SIZE (JMEM_HEAP_SIZE - JMEM_ALIGNMENT)

/**
 * Heap structure
 *
 * Memory blocks returned by the allocator must not start from the
 * beginning of the heap area because offset 0 is reserved for
 * JMEM_CP_NULL. This special constant is used in several places,
 * e.g. it marks the end of the property chain list, so it cannot
 * be eliminated from the project. Although the allocator cannot
 * use the first 8 bytes of the heap, nothing prevents to use it
 * for other purposes. Currently the free region start is stored
 * there.
 */
typedef struct
{
  jmem_heap_free_t first; /**< first node in free region list */
  uint8_t area[JMEM_HEAP_AREA_SIZE]; /**< heap area */
} jmem_heap_t;

/**
 * Global context.
 */
extern jerry_context_t jerry_global_context;

#ifndef JERRY_SYSTEM_ALLOCATOR
/**
 * Global heap.
 */
extern jmem_heap_t jerry_global_heap;
#endif /* !JERRY_SYSTEM_ALLOCATOR */

#ifndef CONFIG_ECMA_LCACHE_DISABLE

/**
 * Global hash table.
 */
extern jerry_hash_table_t jerry_global_hash_table;

#endif /* !CONFIG_ECMA_LCACHE_DISABLE */

/**
 * Provides a reference to a field in the current context.
 */
#define JERRY_CONTEXT(field) (jerry_global_context.field)

#ifndef JERRY_SYSTEM_ALLOCATOR
/**
 * Provides a reference to the area field of the heap.
 */
#define JERRY_HEAP_CONTEXT(field) (jerry_global_heap.field)

#endif /* !JERRY_SYSTEM_ALLOCATOR */

#ifndef CONFIG_ECMA_LCACHE_DISABLE

/**
 * Provides a reference to the global hash table.
 */
#define JERRY_HASH_TABLE_CONTEXT(field) (jerry_global_hash_table.field)

#endif /* !CONFIG_ECMA_LCACHE_DISABLE */

#endif /* JERRY_ENABLE_EXTERNAL_CONTEXT */

/**
 * @}
 */

#endif /* !JCONTEXT_H */
