/******************************************************************
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

/****************************************************************************
 *
 *   Copyright (C) 2014-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The Samsung sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2016 Samsung Electronics, Inc.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/****************************************************************************
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

****************************************************************************/

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <semaphore.h>
#include <pthread.h>
#include <errno.h>
#include <sched.h>
#include <debug.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <tinyara/wdog.h>
#include <tinyara/arch.h>
#include <tinyara/cancelpt.h>

#include "esp_attr.h"
#include "esp_phy_init.h"
#include "esp_wifi_os_adapter.h"
#include "nvs.h"
#include "esp_system.h"
#include "rom/ets_sys.h"
#include <sched/sched.h>
#include <semaphore/semaphore.h>
#include "event_groups.h"

static irqstate_t wifi_int_disable_flags;

/*extern functions declare*/
extern uint32_t IRAM_ATTR esp_random(void);
int64_t get_instant_time(void);

/*=================seamphore API=================*/
static void *IRAM_ATTR semphr_create_wrapper(uint32_t max, uint32_t init)
{
	if (max > SEM_VALUE_MAX || init > SEM_VALUE_MAX) {
		return NULL;
	}

	sem_t *sem = (sem_t *)malloc(sizeof(*sem));
	if (!sem) {
		return NULL;
	}

	int status = sem_init(sem, 0, init);
	if (status != OK) {
		return NULL;
	}

	return (void *)sem;

}

static void IRAM_ATTR semphr_delete_wrapper(void *semphr)
{
	if (semphr == NULL) {
		dbg("semphr is NULL\n");
		return;
	}
	sem_destroy(semphr);
	free(semphr);
}

static int32_t IRAM_ATTR semphr_take_from_isr_wrapper(void *semphr, void *hptw)
{

	*(bool *) hptw = WIFI_ADAPTER_FALSE;
	FAR struct tcb_s *stcb = NULL;
	FAR struct tcb_s *rtcb = this_task();
	sem_t *sem = (sem_t *) semphr;
	irqstate_t saved_state;
	int ret = pdFAIL;
	saved_state = irqsave();

	if (enter_cancellation_point()) {
		set_errno(ECANCELED);
		leave_cancellation_point();
		irqrestore(saved_state);
		return ERROR;
	}

	/* Make sure we were supplied with a valid semaphore */
	if ((sem != NULL) && ((sem->flags & FLAGS_INITIALIZED) != 0)) {

		if (sem->semcount > 0) {
			/* It is, let the task take the semaphore. */
			sem->semcount--;
			sem_addholder(sem);
			rtcb->waitsem = NULL;
			ret = pdPASS;
			for (stcb = (FAR struct tcb_s *)g_waitingforsemaphore.head; (stcb && stcb->waitsem != sem); stcb = stcb->flink) ;
			if (stcb) {
				if (stcb->sched_priority >= rtcb->sched_priority) {
					*(bool *) hptw = WIFI_ADAPTER_TRUE;
				}
			}
		}
		/* The semaphore is NOT available */

		else {

			return ret;
		}
	} else {
		set_errno(EINVAL);
	}

	leave_cancellation_point();
	irqrestore(saved_state);
	return ret;

}

static int32_t IRAM_ATTR semphr_take_wrapper(void *semphr, uint32_t block_time_tick)
{
	int ret;
	if (semphr == NULL) {
		dbg("semphr is NULL\n");
		return pdFAIL;
	}

	if (block_time_tick == OSI_FUNCS_TIME_BLOCKING) {
		ret = sem_wait(semphr);
		if (ret == OK) {
			return pdPASS;
		} else {
			return pdFAIL;
		}

	} else {
		clock_t msecs;
		clock_t secs;
		clock_t nsecs;
		struct timespec abstime;
		(void)clock_gettime(CLOCK_REALTIME, &abstime);

		msecs = TICK2MSEC(block_time_tick);
		secs = msecs / MSEC_PER_SEC;
		nsecs = (msecs - (secs * MSEC_PER_SEC)) * NSEC_PER_MSEC;
		abstime.tv_sec += secs;
		abstime.tv_nsec += nsecs;
		ret = sem_timedwait(semphr, &abstime);
		if (ret == OK) {
			return pdPASS;
		} else {
			return pdFAIL;
		}
	}
}

static int32_t IRAM_ATTR semphr_give_wrapper(void *semphr)
{
	int ret;
	if (semphr == NULL) {
		dbg("semphr is NULL\n");
		return EINVAL;
	}
	ret = sem_post(semphr);
	if (ret == OK) {
		return pdPASS;
	} else {
		return pdFAIL;
	}
}

static int32_t IRAM_ATTR semphr_give_from_isr_wrapper(void *semphr, void *hptw)
{
	*(int *)hptw = WIFI_ADAPTER_FALSE;
	return semphr_give_wrapper(semphr);
}

/*=================mutx API=================*/
static void *IRAM_ATTR recursive_mutex_create_wrapper(void)
{
	pthread_mutexattr_t mattr;
	int status = 0;

	pthread_mutex_t *mutex = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
	if (mutex == NULL) {
		return NULL;
	}
	pthread_mutexattr_init(&mattr);
	status = pthread_mutexattr_settype(&mattr, PTHREAD_MUTEX_RECURSIVE);
	if (status != 0) {
		dbg("recursive_mutex_test: ERROR pthread_mutexattr_settype failed, status=%d\n", status);
		return NULL;
	}
	status = pthread_mutex_init(mutex, &mattr);
	if (status) {
		return NULL;
	}
	return (void *)mutex;
}

static void *IRAM_ATTR mutex_create_wrapper(void)
{
	int status = 0;
	pthread_mutex_t *mutex = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));

	if (mutex == NULL) {
		return NULL;
	}

	status = pthread_mutex_init(mutex, NULL);
	if (status) {
		return NULL;
	}
	return (void *)mutex;

}

static void IRAM_ATTR mutex_delete_wrapper(void *mutex)
{
	if (mutex == NULL) {
		dbg("mutex is NULL\n");
		return;
	}
	pthread_mutex_destroy(mutex);
	free(mutex);
}

static int32_t IRAM_ATTR mutex_lock_wrapper(void *mutex)
{
	if (mutex == NULL) {
		dbg("mutex is NULL\n");
		return EINVAL;
	}
	int ret = pthread_mutex_lock(mutex);
	if (ret) {
		return pdFAIL;
	}
	return pdPASS;

}

static int32_t IRAM_ATTR mutex_unlock_wrapper(void *mutex)
{
	if (mutex == NULL) {
		dbg("mutex is NULL\n");
		return EINVAL;
	}
	int ret = pthread_mutex_unlock(mutex);
	if (ret) {
		return pdFAIL;
	}
	return pdPASS;
}

/*=================task control API=================*/
static int32_t IRAM_ATTR task_create_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle)
{
	int pid = task_create(name, prio, stack_depth, task_func, param);
	if (pid < 0) {
		return pdFAIL;
	}

	task_handle = (void *)pid;
	return pdPASS;
}

static int32_t IRAM_ATTR task_create_pinned_to_core_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle, uint32_t core_id)
{
#ifndef CONFIG_SMP
	return task_create_wrapper(task_func, name, stack_depth, param, prio, task_handle);
#else
	/*currently TizenRT not support SMP */
	return pdFAIL;
#endif
}

static void IRAM_ATTR task_delete_wrapper(void *task_handle)
{
	if (task_handle < 0) {
		return;
	}
	int pid = (int)task_handle;

	task_delete(pid);

}

static void IRAM_ATTR task_delay_wrapper(uint32_t tick)
{
	uint64_t usecs = TICK2USEC(tick);
	usleep(usecs);
}

static int curpid;
static void *IRAM_ATTR task_get_current_task_wrapper(void)
{
	curpid = getpid();
	return (void *)&curpid;
}

static inline int32_t IRAM_ATTR task_ms_to_tick_wrapper(uint32_t ms)
{
	return (int32_t) MSEC2TICK(ms);
}

static inline int32_t IRAM_ATTR task_get_max_priority_wrapper(void)
{
	return (int32_t)(SCHED_PRIORITY_MAX);
}

static inline int32_t IRAM_ATTR is_in_isr_wrapper(void)
{
	return (int32_t) up_interrupt_context();
}

/*=================wifi RF API=================*/

static inline int32_t IRAM_ATTR esp_phy_rf_deinit_wrapper(uint32_t module)
{
	return esp_phy_rf_deinit((phy_rf_module_t) module);
}

static inline int32_t IRAM_ATTR phy_rf_init_wrapper(const void *init_data, uint32_t mode, void *calibration_data, uint32_t module)
{
	return esp_phy_rf_init(init_data, mode, calibration_data, module);
}

static inline void IRAM_ATTR esp_phy_load_cal_and_init_wrapper(uint32_t module)
{
	return esp_phy_load_cal_and_init((phy_rf_module_t) module);
}

static inline int32_t esp_read_mac_wrapper(uint8_t *mac, uint32_t type)
{
	return esp_read_mac(mac, (esp_mac_type_t) type);
}

/*=================soft timer API=================*/

void ets_timer_init(void)
{

}

void ets_timer_deinit(void)
{

}

static void IRAM_ATTR timer_arm_wrapper(void *timer, uint32_t tmout, bool repeat)
{
	ETSTimer *etimer = (ETSTimer *) timer;
	if (etimer == NULL || etimer->wdog == NULL) {
		dbg("timer is NULL\n");
		return;
	}
	int delay = MSEC2TICK(tmout);
	wd_start(etimer->wdog, delay, etimer->func, 0, NULL);

}

static void IRAM_ATTR timer_disarm_wrapper(void *timer)
{
	ETSTimer *etimer = (ETSTimer *) timer;
	if (etimer == NULL || etimer->wdog == NULL) {
		dbg("timer is NULL\n");
		return;
	}
	wd_cancel(etimer->wdog);
}

static void IRAM_ATTR timer_done_wrapper(void *ptimer)
{
	ETSTimer *etimer = (ETSTimer *) ptimer;
	if (etimer == NULL || etimer->wdog == NULL) {
		dbg("timer is NULL\n");
		return;
	}
	etimer->func = NULL;
	wd_delete(etimer->wdog);
}

static void IRAM_ATTR timer_setfn_wrapper(void *ptimer, void *pfunction, void *parg)
{
	ETSTimer *etimer = (ETSTimer *) ptimer;
	if (etimer == NULL) {
		dbg("timer is NULL\n");
		return;
	}
	etimer->wdog = wd_create();
	if (!etimer->wdog) {
		return;
	}
	etimer->func = (wdentry_t) pfunction;
}

static void IRAM_ATTR timer_arm_us_wrapper(void *ptimer, uint32_t us, bool repeat)
{
	ETSTimer *etimer = (ETSTimer *) ptimer;
	if (etimer == NULL) {
		dbg("timer is NULL\n");
		return;
	}
	int delay = USEC2TICK(us);
	wd_start(etimer->wdog, delay, etimer->func, 0, NULL);
}

static inline int32_t IRAM_ATTR get_time_wrapper(void *t)
{
	return (int32_t) gettimeofday((struct timeval *)t, NULL);
}

/*=================Miscellaneous API================================*/
static inline int32_t IRAM_ATTR esp_os_get_random_wrapper(uint8_t *buf, size_t len)
{
	return (int32_t) os_get_random((unsigned char *)buf, len);

}

typedef enum {
	ESP_LOG_NONE,				/*!< No log output */
	ESP_LOG_ERROR,				/*!< Critical errors, software module can not recover on its own */
	ESP_LOG_WARN,				/*!< Error conditions from which recovery measures have been taken */
	ESP_LOG_INFO,				/*!< Information messages which describe normal flow of events */
	ESP_LOG_DEBUG,				/*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
	ESP_LOG_VERBOSE				/*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
} esp_log_level_t;

static void IRAM_ATTR log_write_wrapper(uint32_t level, const char *tag, const char *format, ...)
{
	switch (level) {
	case ESP_LOG_ERROR:
	case ESP_LOG_DEBUG:
		dbg(format);
		break;

	case ESP_LOG_WARN:
		wdbg(format);
		break;

	case ESP_LOG_INFO:
	case ESP_LOG_VERBOSE:
		vdbg(format);
		break;
	case ESP_LOG_NONE:
		return;
	}
}

/*=================espwifi modem API=========================*/

static inline esp_err_t esp_modem_sleep_enter_wrapper(uint32_t module)
{
	return esp_modem_sleep_enter((modem_sleep_module_t) module);
}

static inline esp_err_t esp_modem_sleep_exit_wrapper(uint32_t module)
{
	return esp_modem_sleep_exit((modem_sleep_module_t) module);

}

static inline esp_err_t esp_modem_sleep_register_wrapper(uint32_t module)
{
	return esp_modem_sleep_register((modem_sleep_module_t) module);
}

static inline esp_err_t esp_modem_sleep_deregister_wrapper(uint32_t module)
{
	return esp_modem_sleep_deregister((modem_sleep_module_t) module);
}

/*=================espwifi NVS API=========================*/

static inline int32_t esp_nvs_open_wrapper(const char *name, uint32_t open_mode, uint32_t *out_handle)
{
	return nvs_open(name, (nvs_open_mode) open_mode, (nvs_handle *) out_handle);
}

/*=================espwifi smart config API=========================*/
/* Will complete next stage, not block WIFI ENABLE*/

void IRAM_ATTR esp_dport_access_stall_other_cpu_start_wrap(void)
{
	//useless for signle CPU
	return;
}

void IRAM_ATTR esp_dport_access_stall_other_cpu_end_wrap(void)
{
	//useless for signle CPU
	return;
}

static void IRAM_ATTR set_isr_wrapper(int32_t n, void *f, void *arg)
{
	irq_attach(n, f, arg);
	extern void up_enable_irq(int cpuint);
	up_enable_irq(n);
}

static void *IRAM_ATTR spin_lock_create_wrapper(void)
{
	pthread_mutex_t *mux = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
	if (mux) {
		pthread_mutex_init(mux, NULL);
		return mux;
	}

	return NULL;
}

static irqstate_t wifi_int_disable_flags;

static uint32_t IRAM_ATTR wifi_int_disable_wrapper(void *wifi_int_mux)
{
	if (wifi_int_mux) {
		wifi_int_disable_flags = irqsave();
		pthread_mutex_lock((pthread_mutex_t *) wifi_int_mux);
	}
	return 0;
}

static void IRAM_ATTR wifi_int_restore_wrapper(void *wifi_int_mux, uint32_t tmp)
{
	//tmp is useless.
	if (wifi_int_mux) {
		pthread_mutex_unlock((pthread_mutex_t *) wifi_int_mux);
		irqrestore(wifi_int_disable_flags);
	}
	return;
}

static void IRAM_ATTR task_yield_from_isr_wrapper(void)
{
	return;
}

static uint32_t IRAM_ATTR event_group_wait_bits_wrapper(void *event, uint32_t bits_to_wait_for, int32_t clear_on_exit, int32_t wait_for_all_bits, uint32_t block_time_tick)
{
	if (block_time_tick == OSI_FUNCS_TIME_BLOCKING) {
		return (uint32_t) event_group_wait_bits(event, bits_to_wait_for, clear_on_exit, wait_for_all_bits, port_max_delay);
	}

	return (uint32_t) event_group_wait_bits(event, bits_to_wait_for, clear_on_exit, wait_for_all_bits, block_time_tick);
}

uint32_t IRAM_ATTR esp_get_free_heap_size(void)
{
	struct mallinfo mem_info;

#ifdef CONFIG_CAN_PASS_STRUCTS
	mem_info = mallinfo();
#else
	(void)mallinfo(&mem_info);
#endif

	return mem_info.fordblks;
}

static void *IRAM_ATTR malloc_internal_wrapper(size_t size)
{
	return malloc(size);
}

static void *IRAM_ATTR realloc_internal_wrapper(void *ptr, size_t size)
{
	return realloc(ptr, size);
}

static void *IRAM_ATTR calloc_internal_wrapper(size_t n, size_t size)
{
	return calloc(n, size);
}

static void *IRAM_ATTR zalloc_internal_wrapper(size_t size)
{
	return zalloc(size);
}

/*
 If CONFIG_WIFI_LWIP_ALLOCATION_FROM_SPIRAM_FIRST is enabled. Prefer to allocate a chunk of memory in SPIRAM firstly.
 If failed, try to allocate it in internal memory then.
 */
void *IRAM_ATTR wifi_malloc(size_t size)
{
	return malloc(size);
}

/*
 If CONFIG_WIFI_LWIP_ALLOCATION_FROM_SPIRAM_FIRST is enabled. Prefer to allocate a chunk of memory in SPIRAM firstly.
 If failed, try to allocate it in internal memory then.
 */
void *IRAM_ATTR wifi_realloc(void *ptr, size_t size)
{
	return realloc(ptr, size);
}

/*
 If CONFIG_WIFI_LWIP_ALLOCATION_FROM_SPIRAM_FIRST is enabled. Prefer to allocate a chunk of memory in SPIRAM firstly.
 If failed, try to allocate it in internal memory then.
*/
void *IRAM_ATTR wifi_calloc(size_t n, size_t size)
{
	return calloc(n, size);
}

static void *IRAM_ATTR wifi_zalloc_wrapper(size_t size)
{
	return zalloc(size);
}

wifi_static_queue_t *IRAM_ATTR wifi_create_queue(int queue_len, int item_size)
{
	wifi_static_queue_t *wifi_queue = (wifi_static_queue_t *)malloc(sizeof(wifi_static_queue_t));
	if (!wifi_queue) {
		return NULL;
	}

	wifi_queue->handle = queue_create_wrapper(queue_len, item_size);
	return wifi_queue;
}

void IRAM_ATTR wifi_delete_queue(wifi_static_queue_t *queue)
{
	if (queue && queue->handle) {
		queue_delete_wrapper(queue->handle);
	}
	free(queue);
}

static void *IRAM_ATTR wifi_create_queue_wrapper(int32_t queue_len, int32_t item_size)
{
	return wifi_create_queue(queue_len, item_size);
}

static void IRAM_ATTR wifi_delete_queue_wrapper(void *queue)
{
	wifi_delete_queue(queue);
}

void IRAM_ATTR task_yield_wrapper(void)
{
	extern int sched_yield(void);
	sched_yield();
}

int32_t IRAM_ATTR get_random_wrapper(uint8_t *buf, size_t len)
{
	extern int os_get_random(unsigned char * buf, size_t len);
	return (int32_t) os_get_random(buf, len);
}

extern void xtensa_enable_cpuint(uint32_t mask);
extern void xtensa_disable_cpuint(uint32_t mask);
extern int os_get_random(unsigned char *buf, size_t len);
extern unsigned long os_random(void);

/*=================espwifi os adapter interface =====================*/

wifi_osi_funcs_t g_wifi_osi_funcs = {
	._version = ESP_WIFI_OS_ADAPTER_VERSION,
	._set_isr = set_isr_wrapper,
	._ints_on = xtensa_enable_cpuint,
	._ints_off = xtensa_disable_cpuint,
	._spin_lock_create = spin_lock_create_wrapper,
	._spin_lock_delete = free,
	._wifi_int_disable = wifi_int_disable_wrapper,
	._wifi_int_restore = wifi_int_restore_wrapper,
	._task_yield = task_yield_wrapper,
	._task_yield_from_isr = task_yield_from_isr_wrapper,
	._semphr_create = semphr_create_wrapper,
	._semphr_delete = semphr_delete_wrapper,
	._semphr_take_from_isr = semphr_take_from_isr_wrapper,
	._semphr_give_from_isr = semphr_give_from_isr_wrapper,
	._semphr_take = semphr_take_wrapper,
	._semphr_give = semphr_give_wrapper,
	._mutex_create = mutex_create_wrapper,
	._recursive_mutex_create = recursive_mutex_create_wrapper,
	._mutex_delete = mutex_delete_wrapper,
	._mutex_lock = mutex_lock_wrapper,
	._mutex_unlock = mutex_unlock_wrapper,
	._queue_create = queue_create_wrapper,
	._queue_delete = queue_delete_wrapper,
	._queue_send = queue_send_wrapper,
	._queue_send_from_isr = queue_send_from_isr_wrapper,
	._queue_send_to_back = queue_send_to_back_wrapper,
	._queue_send_to_front = queue_send_to_front_wrapper,
	._queue_recv = queue_recv_wrapper,
	//._queue_recv_from_isr = xQueueReceiveFromISR,
	._queue_msg_waiting = queue_msg_waiting_wrapper,
	._event_group_create = event_group_create,
	._event_group_delete = event_group_delete,
	._event_group_set_bits = event_group_set_bits,
	._event_group_clear_bits = event_group_clear_bits,
	._event_group_wait_bits = event_group_wait_bits_wrapper,
	._task_create_pinned_to_core = task_create_pinned_to_core_wrapper,
	._task_create = task_create_wrapper,
	._task_delete = task_delete_wrapper,
	._task_delay = task_delay_wrapper,
	._task_ms_to_tick = task_ms_to_tick_wrapper,
	._task_get_current_task = task_get_current_task_wrapper,
	._task_get_max_priority = task_get_max_priority_wrapper,
	._is_in_isr = is_in_isr_wrapper,
	._malloc = malloc,
	._free = free,
	._get_free_heap_size = esp_get_free_heap_size,
	._rand = esp_random,
	._dport_access_stall_other_cpu_start_wrap = esp_dport_access_stall_other_cpu_start_wrap,
	._dport_access_stall_other_cpu_end_wrap = esp_dport_access_stall_other_cpu_end_wrap,
	//._phy_rf_init = phy_rf_init_wrapper,
	//._phy_rf_deinit = esp_phy_rf_deinit_wrapper,
	//._phy_load_cal_and_init = esp_phy_load_cal_and_init_wrapper,
	._read_mac = esp_read_mac_wrapper,
	._timer_init = ets_timer_init,
	._timer_deinit = ets_timer_deinit,
	._timer_arm = timer_arm_wrapper,
	._timer_disarm = timer_disarm_wrapper,
	._timer_done = timer_done_wrapper,
	._timer_setfn = timer_setfn_wrapper,
	._timer_arm_us = timer_arm_us_wrapper,
	._esp_timer_get_time = get_instant_time,
	._nvs_set_i8 = nvs_set_i8,
	._nvs_get_i8 = nvs_get_i8,
	._nvs_set_u8 = nvs_set_u8,
	._nvs_get_u8 = nvs_get_u8,
	._nvs_set_u16 = nvs_set_u16,
	._nvs_get_u16 = nvs_get_u16,
	._nvs_open = esp_nvs_open_wrapper,
	._nvs_close = nvs_close,
	._nvs_commit = nvs_commit,
	._nvs_set_blob = nvs_set_blob,
	._nvs_get_blob = nvs_get_blob,
	._nvs_erase_key = nvs_erase_key,
	._get_random = esp_os_get_random_wrapper,
	._get_time = get_time_wrapper,
	._random = esp_random,
	._log_write = log_write_wrapper,
	._log_timestamp = esp_log_timestamp,
	._malloc_internal = malloc_internal_wrapper,
	._realloc_internal = realloc_internal_wrapper,
	._calloc_internal = calloc_internal_wrapper,
	._zalloc_internal = zalloc_internal_wrapper,
	._wifi_malloc = wifi_malloc,
	._wifi_realloc = wifi_realloc,
	._wifi_calloc = wifi_calloc,
	._wifi_zalloc = wifi_zalloc_wrapper,
	._wifi_create_queue = wifi_create_queue_wrapper,
	._wifi_delete_queue = wifi_delete_queue_wrapper,
	//._modem_sleep_enter = esp_modem_sleep_enter_wrapper,
	//._modem_sleep_exit = esp_modem_sleep_exit_wrapper,
	//._modem_sleep_register = esp_modem_sleep_register_wrapper,
	//._modem_sleep_deregister = esp_modem_sleep_deregister_wrapper,
	//._sc_ack_send = sc_ack_send_wrapper,
	//._sc_ack_send_stop = sc_ack_send_stop,
	._magic = ESP_WIFI_OS_ADAPTER_MAGIC,
};
