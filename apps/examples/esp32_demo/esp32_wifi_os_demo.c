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

#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>
#include <rom/ets_sys.h>
#include <time.h>
#include "esp_wifi_os_adapter.h"
#include "event_groups.h"
#include "esp32_queue_api.h"
#include <rom/ets_sys.h>
#include <time.h>
#include <nvs.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_system.h>

//Will enable later.
#define QUEUE_SEND_HANDLER_STACKSIZE (1024)
#define RANDOM_TEST_TIME (20)
#define MALLOC_CAP_SPIRAM           (1<<10)
#define MALLOC_CAP_DMA              (1<<3)
extern wifi_osi_funcs_t g_wifi_osi_funcs;

void test_nvs(void)
{
	nvs_handle handle_1;
	g_wifi_osi_funcs._nvs_open("test_namespace1", NVS_READONLY, &handle_1);
}

/*testcase*/
void test_rand(void)
{
	uint32_t ret;
	for (int i = 0; i < RANDOM_TEST_TIME; i++) {
		ret = g_wifi_osi_funcs._rand();
		printf("random %d =  %u\n", i + 1, ret);
	}
	printf("====Test random sucess====\n");
}

void test_mutex(void)
{
	void *mutex = g_wifi_osi_funcs._mutex_create();
	if (!mutex) {
		printf("mutex is NULL\n");
	}
	static int num = 0;
	g_wifi_osi_funcs._mutex_lock(mutex);
	num++;
	g_wifi_osi_funcs._mutex_unlock(mutex);
	g_wifi_osi_funcs._mutex_delete(mutex);
	printf("====Test mutex sucess====\n");
}

void test_recursive_mutex(void)
{
	void *mutex = g_wifi_osi_funcs._recursive_mutex_create();
	if (!mutex) {
		printf("mutex is NULL\n");
	}
	static int num = 0;
	g_wifi_osi_funcs._mutex_lock(mutex);
	g_wifi_osi_funcs._mutex_lock(mutex);
	num++;
	g_wifi_osi_funcs._mutex_unlock(mutex);
	g_wifi_osi_funcs._mutex_unlock(mutex);
	g_wifi_osi_funcs._mutex_delete(mutex);
	printf("====Test recursive mutex sucess====\n");
}

void test_sem(void)
{
	void *sem = g_wifi_osi_funcs._semphr_create(1, 1);
	if (!sem) {
		printf("sem is NULL\n");
	}
	static int num = 0;
	g_wifi_osi_funcs._semphr_take(sem, 100000);
	num++;
	g_wifi_osi_funcs._semphr_give(sem);
	g_wifi_osi_funcs._semphr_delete(sem);
	printf("====Test sem sucess====\n");

}

int handle;
void *taskfunc(void *parm)
{
	int32_t ticks = g_wifi_osi_funcs._task_ms_to_tick(1000);
	if (ticks != 100) {
		printf("task_ms_to_tick error\n");
	}
	int32_t maxprio = g_wifi_osi_funcs._task_get_max_priority();

	if (maxprio != 255) {
		printf("task_get_max_priority error\n");
	}
	void *taskhandle = g_wifi_osi_funcs._task_get_current_task();

	int pid = *(int *)taskhandle;
	if (pid < 0) {
		printf("task_get_current_task error\n");
	}

	g_wifi_osi_funcs._task_delay(100);
	if (g_wifi_osi_funcs._is_in_isr() != 0) {
		printf("Test isr error\n");
	}
	g_wifi_osi_funcs._task_delete(&handle);
	printf("====Test task success====\n");
	return NULL;
}

void test_task(void)
{
	g_wifi_osi_funcs._task_create(taskfunc, "test_task_control", 1024, NULL, 100, &handle);

}

ETSTimer mytimer;

void *timer_func(void *parm)
{
	g_wifi_osi_funcs._timer_disarm(&mytimer);
	g_wifi_osi_funcs._timer_done(&mytimer);
	printf("===Test timer success====\n");
	return NULL;
}

void test_timer(void)
{
	g_wifi_osi_funcs._timer_setfn(&mytimer, timer_func, NULL);
	g_wifi_osi_funcs._timer_arm(&mytimer, 0, 0);
}

void test_time(void)
{
	/*rtc is not enabled, currently basetime is 2012-01-06, 00:00:00 */
	struct timeval tv;
	struct tm *ptm;
	char time_string[40];
	long milliseconds;
	g_wifi_osi_funcs._get_time(&tv);
	ptm = localtime(&tv.tv_sec);
	ptm = (struct tm *)localtime(&tv.tv_sec);
	(void)strftime(time_string, 40, "%Y-%m-%d %H:%M:%SUTC", ptm);
	milliseconds = tv.tv_usec / 1000;
	printf("%s.%03ld\n", time_string, milliseconds);
	int64_t bootime_ms = g_wifi_osi_funcs._esp_timer_get_time();
	printf("%lld\n", bootime_ms);
	printf("===Test gettime success====\n");
}

typedef struct {
	int type;
	char buff[10];
} queue_item_t;

static int stop;

static void *queue_send_thread(void *queue_handle)
{
	queue_item_t data;
	char *str[20] = { "send 1", "send 2", "send 3", "send 4", "send 5", "send 6", "send 7", "send 8", "send 9", "send 10",
					  "send 11", "send 12", "send 13", "send 14", "send 15", "send 16", "send 17", "send 18", "send 19", "send 20"
					};
	char *stop_str = "stop";

	if (!queue_handle) {
		printf("%s input null piont!\n", __FUNCTION__);
		return NULL;
	}
	int i;

	if (g_wifi_osi_funcs._queue_send) {
		for (i = 0; i < 20; i++) {
			data.type = i;
			strcpy(data.buff, str[i]);
			g_wifi_osi_funcs._queue_send(queue_handle, &data, 1);
		}
		usleep(1000 * 1000);
	} else {
		printf("g_wifi_osi_funcs._queue_send null piont!\n");
	}

	if (g_wifi_osi_funcs._queue_send_to_back) {
		for (i = 0; i < 20; i++) {
			data.type = i;
			strcpy(data.buff, str[i]);
			g_wifi_osi_funcs._queue_send_to_back(queue_handle, &data, 1);
		}
		usleep(1000 * 1000);
	} else {
		printf("g_wifi_osi_funcs._queue_send_to_back null piont!\n");
	}

	if (g_wifi_osi_funcs._queue_send_to_front) {
		for (i = 0; i < 20; i++) {
			data.type = i;
			strcpy(data.buff, str[i]);
			g_wifi_osi_funcs._queue_send_to_front(queue_handle, &data, 1);
		}
		usleep(1000 * 1000);
	} else {
		printf("g_wifi_osi_funcs._queue_send_to_front null piont!\n");
	}

	//send stop to reciever
	if (stop == 0) {
		stop = 1;

		data.type = 0;
		strcpy(data.buff, stop_str);
		if (g_wifi_osi_funcs._queue_send) {
			g_wifi_osi_funcs._queue_send(queue_handle, &data, 1);
		} else {
			printf("g_wifi_osi_funcs._queue_send null piont!\n");
		}	
	}
	return NULL;
}

static void *queue_recieve_thread(void *queue_handle)
{
	queue_item_t item;
	if (!queue_handle) {
		printf("%s input null piont!\n", __FUNCTION__);
		return NULL;
	}

	if (!g_wifi_osi_funcs._queue_recv) {
		printf("g_wifi_osi_funcs._queue_recv null piont!\n");
		return NULL;
	}

	if (stop == 1) {
		stop = 0;
	}

	while (!stop) {
		if (g_wifi_osi_funcs._queue_recv(queue_handle, &item, 1) != 0) {
			printf("Recieve message:\n	type: %d, string: %s\n", item.type, item.buff);
		}
	}
	return NULL;
}

void queue_operate_demo(void)
{
	pthread_t queue_send_thread_handle, queue_recieve_thread_handle;
	uint32_t queue_len = 20;
	pthread_attr_t attr;
	int status = -1;
	
	if (g_wifi_osi_funcs._queue_create) {
		queue_info_t *queue_handle = g_wifi_osi_funcs._queue_create(queue_len, sizeof(queue_item_t));
		if (!queue_handle) {
			printf("%s input null piont!\n", __FUNCTION__);
			return;
		}

		if (pthread_attr_init(&attr) != 0) {
			printf("Error: Cannot initialize ptread attribute\n");
			return;
		}
		pthread_attr_setschedpolicy(&attr, SCHED_RR);
		pthread_attr_setstacksize(&attr, QUEUE_SEND_HANDLER_STACKSIZE);


		status = pthread_create(&queue_recieve_thread_handle, &attr, queue_recieve_thread, (void *)queue_handle);
		if (status != 0) {
			printf("queue recieve: pthread_create failed, status=%d\n", status);
		}

		pthread_setname_np(queue_recieve_thread_handle, "esp32 wifi queue reciever");

		status = pthread_create(&queue_send_thread_handle, &attr, queue_send_thread, (void *)queue_handle);
		if (status != 0) {
			printf("queue send: pthread_create failed, status=%d\n", status);
		}

		pthread_setname_np(queue_send_thread_handle, "esp32 wifi queue sender");
		
		status = pthread_join(queue_send_thread_handle, NULL);
		if (status != OK) {
			printf("esp32 wifi queue sender: ERROR: pthread_join failed: %d\n", status);
		}
		printf("jion send thread!\n");

		status = pthread_join(queue_recieve_thread_handle, NULL);
		if (status != OK) {
			printf("esp32 wifi queue recieve: ERROR: pthread_join failed: %d\n", status);
		}
		printf("jion recieve thread!\n");
	} else {
		printf("g_wifi_osi_funcs._queue_create null pionter!\n");
	}
	
}

//event group test code

static event_group_handle_t test_event_group;

static void *event_group_setbit_thread(void *param)
{
	event_group_handle_t event_group = (event_group_handle_t) param;

	event_bits_t event_bit[10] = { BIT9, BIT8, BIT7, BIT6, BIT5, BIT4, BIT3, BIT2, BIT1, BIT0 };

	if (event_group && g_wifi_osi_funcs._event_group_set_bits) {
		for (int i = 0; i < 10; i++) {
			printf("Setbit_thread: %x\n", event_bit[i]);
			g_wifi_osi_funcs._event_group_set_bits(event_group, event_bit[i]);
		}
	}
	return NULL;
}

typedef struct {
	event_group_handle_t event_group;
	event_bits_t bits_to_wait_for;
	base_type_t clear_on_exit;
	base_type_t wait_for_all_bits;
	tick_type_t ticks_to_wait;
} test_group_event_struct_t;

static void *event_group_waitbit_thread(void *param)
{
	test_group_event_struct_t *test_group_event_parameter = (test_group_event_struct_t *) param;
	if (test_group_event_parameter) {
		printf("Wait for 0x%x!, zzzZZZZ!\n", test_group_event_parameter->bits_to_wait_for);
		g_wifi_osi_funcs._event_group_wait_bits(test_group_event_parameter->event_group, test_group_event_parameter->bits_to_wait_for, test_group_event_parameter->clear_on_exit, test_group_event_parameter->wait_for_all_bits, test_group_event_parameter->ticks_to_wait);

		printf("Week up because 0x%x!, do nothing!\n", test_group_event_parameter->bits_to_wait_for);
	}
	return NULL;
}

pthread_t event_group_waitbit_thread_handle[10];
test_group_event_struct_t test_group_event_parameter[10];

static void create_wait_event_threads(void)
{
	int status = -1;
	pthread_attr_t attr;

	event_bits_t event_bit[10] = { BIT0, BIT1, BIT2, BIT3, BIT4, BIT5, BIT6, BIT7, BIT8, BIT9 };
	if (pthread_attr_init(&attr) != 0) {
		printf("Error: Cannot initialize ptread attribute\n");
		return;
	}

	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, QUEUE_SEND_HANDLER_STACKSIZE);

	if (test_event_group) {
		for (int i = 0; i < 10; i++) {
			test_group_event_parameter[i].event_group = test_event_group;
			test_group_event_parameter[i].bits_to_wait_for = event_bit[i];
			test_group_event_parameter[i].clear_on_exit = true;
			test_group_event_parameter[i].wait_for_all_bits = false;
			test_group_event_parameter[i].ticks_to_wait = 0xffff;
			printf("creat event_group_waitbit_thread\n");
			status = pthread_create(&event_group_waitbit_thread_handle[i], &attr, event_group_waitbit_thread, &test_group_event_parameter[i]);
			if (status != 0) {
				printf("event group waitbit: pthread_create %d failed, status=%d\n", i, status);
			} else {
				char buff[10];
				sprintf(buff, "%s%d", "event_group_wait", i);
				pthread_setname_np(event_group_waitbit_thread_handle[i], buff);
			}
		}
	}
}

static void join_wait_event_threads(void)
{
	for (int i = 0; i < 10; i++) {
		pthread_join(event_group_waitbit_thread_handle[i], NULL);
	}
}

void event_group_demo(void)
{
	pthread_t event_group_setbit_thread_handle;
	int status = -1;
	pthread_attr_t attr;
	if (g_wifi_osi_funcs._event_group_create) {
		test_event_group = g_wifi_osi_funcs._event_group_create();
		if (!test_event_group) {
			printf("Create event group failed!\n");
			return;
		}
	} else {
		printf("g_wifi_osi_funcs._event_group_create NULL!\n");
		return;
	}

	create_wait_event_threads();

	usleep(3 * 1000 * 1000);

	if (pthread_attr_init(&attr) != 0) {
		printf("Error: Cannot initialize ptread attribute\n");
		return;
	}
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, QUEUE_SEND_HANDLER_STACKSIZE);

	status = pthread_create(&event_group_setbit_thread_handle, &attr, event_group_setbit_thread, test_event_group);
	if (status != 0) {
		printf("event_group_setbit_thread_handle: pthread_create failed, status=%d\n", status);
	}

	usleep(3 * 1000 * 1000);

	join_wait_event_threads();

	event_group_delete(test_event_group);

	printf("end event group test!\n");

}

#ifdef CONFIG_SPIRAM_USE_CAPS_ALLOC
extern int _sheap;
void test_spiram_alloc()
{
	/*test SPIRAM allocator */
	int *p = heap_caps_malloc(1024, MALLOC_CAP_SPIRAM);
	if ((p >= SOC_EXTRAM_DATA_LOW) && (p < SOC_EXTRAM_DATA_LOW + CONFIG_SPIRAM_SIZE)) {
		printf("heap_caps_malloc SPIRAM success\n");
	} else {
		printf("heap_caps_malloc SPIRAM failed\n");
	}
	heap_caps_free(p);

	/*test Reserve DMA allocator */
	p = heap_caps_malloc(1024, MALLOC_CAP_DMA);
	if ((p > (&_sheap)) && (p < 0x40000000)) {
		printf("heap_caps_malloc DMA success\n");
	} else {
		printf("heap_caps_malloc DMA failed\n");
	}
	heap_caps_free(p);
}
#endif

static int event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            printf("Receive event SYSTEM_EVENT_STA_START\n");
            break;
        default:
            break;
    }   
    return OK;
}

void *sendevent_func(void *arg)
{
    system_event_t event;
    for( int i = 0; i < 10; i++) {
        event.event_id = SYSTEM_EVENT_STA_START;
        esp_event_send(&event);
        printf("send event SYSTEM_EVENT_STA_START\n");
    }
    sleep(10);
    return NULL;
}

void test_event_loop()
{
    esp_event_loop_init(event_handler, NULL);
    pthread_t thread_handle = NULL;
    pthread_create(&thread_handle, NULL, sendevent_func, NULL);
    pthread_join(thread_handle, NULL); 
}

void get_wifi_mac_address()
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    for(int i = 0; i < 6; i++)
        printf("%x ", mac[i]);
    printf("\n");

}

extern int esp_spiram_test();
pthread_addr_t esp32_demo_entry(pthread_addr_t arg)
{
	printf("start esp32 demo!\n");
	//test_timer();
    //get_wifi_mac_address();
#ifdef CONFIG_ESP_WIFI_MODE_STATION
    wifi_station_entry();
#else
    wifi_softap_entry();
#endif
    
#if 0
#ifdef CONFIG_SPIRAM_SUPPORT
	esp_spiram_test();
#endif

#ifdef CONFIG_SPIRAM_USE_CAPS_ALLOC
	test_spiram_alloc();
#endif

#ifdef CONFIG_LEDC
	test_ledc_soft();
#endif
	test_nvs();
	test_rand();
	test_mutex();
	test_recursive_mutex();
	test_sem();
	test_time();
	test_task();
	//wait child task exit
	sleep(2);
	test_timer();
	//wait timer handler exit
	sleep(2);

	queue_operate_demo();
	usleep(3 * 1000 * 1000);
	event_group_demo();

/*  event loop test*/
    //test_event_loop();
#endif
	return NULL;
}
