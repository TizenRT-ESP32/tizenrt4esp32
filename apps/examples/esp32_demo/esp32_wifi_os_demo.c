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


//Will enable later.
#define QUEUE_SEND_HANDLER_STACKSIZE (1024 * 4)
#define RANDOM_TEST_TIME (20)

extern wifi_osi_funcs_t g_wifi_osi_funcs;

/*testcase*/
void test_rand(void)
{
    uint32_t ret;
    for(int i = 0; i < RANDOM_TEST_TIME; i++)
    {
        ret = g_wifi_osi_funcs._rand();
        printf("random %d =  %u\n", i+1, ret);
    }
    printf("====Test random sucess====\n");
}

void test_mutex(void)
{
    void *mutex = g_wifi_osi_funcs._mutex_create();
    if(!mutex)
        printf("mutex is NULL\n");
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
    if(!mutex)
        printf("mutex is NULL\n");
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
    if(!sem)
        printf("sem is NULL\n");
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
    if(ticks != 100)
        printf("task_ms_to_tick error\n");
    int32_t maxprio = g_wifi_osi_funcs._task_get_max_priority();
    
    if(maxprio != 255)
        printf("task_get_max_priority error\n");
    void *taskhandle = g_wifi_osi_funcs._task_get_current_task();
    
    int pid = *(int*)taskhandle;
    if(pid < 0)
        printf("task_get_current_task error\n");

    g_wifi_osi_funcs._task_delay(100);      
    if(g_wifi_osi_funcs._is_in_isr() != 0)
        printf("Test isr error\n");
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
  g_wifi_osi_funcs._timer_arm(&mytimer, 10, 0);
  g_wifi_osi_funcs._timer_setfn(&mytimer, timer_func, NULL);
}

void test_time(void)
{ 
  struct timeval t;    
  g_wifi_osi_funcs._get_time(&t);
  //printf("%ds, %dus\n", t.tv_sec, t.tv_usec);  
  //int64_t period = get_instant_time();
  //printf("elapsed time from booting %lu\n", period);
  printf("===Test gettime success====\n");
}

typedef struct {
	int type;
	char buff[10];
}queue_item_t;


static int stop;

static void *queue_send_thread(void *queue_handle)
{
	queue_item_t data;
	char * str[20] = {"send 1","send 2","send 3","send 4","send 5","send 6","send 7","send 8","send 9","send 10",
		              "send 11","send 12","send 13","send 14","send 15","send 16","send 17","send 18","send 19","send 20"};
	char * stop_str = "stop";

	if (!queue_handle) {
		printf("%s input null piont!\n",__FUNCTION__);
		return NULL;
	}
	int i;

	for (i=0;i<20;i++) {
		data.type = i;
		strcpy(data.buff,str[i]);
		//g_wifi_osi_funcs._queue_send(queue_handle, &data,0);
		queue_send_wrapper(queue_handle, &data,0);
	}

	usleep(1000*1000);

	for (i=0;i<20;i++) {
		data.type = i;
		strcpy(data.buff,str[i]);
		//g_wifi_osi_funcs._queue_send_to_back(queue_handle, &data,0);
		queue_send_to_back_wrapper(queue_handle, &data,0);
	}

	usleep(1000*1000);

	for (i=0;i<20;i++) {
		data.type = i;
		strcpy(data.buff,str[i]);
		//g_wifi_osi_funcs._queue_send_to_front(queue_handle, &data,0);
		queue_send_to_front_wrapper(queue_handle, &data,0);
	}

	usleep(1000*1000);

	//send stop to reciever
	if (stop == 0) {
		stop =1;

		data.type = 0;
		strcpy(data.buff,stop_str);
		//g_wifi_osi_funcs._queue_send(queue_handle, &data,0);
		queue_send_wrapper(queue_handle, &data,0);
	}
	return NULL;
}


static void *queue_recieve_thread(void *queue_handle)
{
	queue_item_t item;
	if (!queue_handle) {
		printf("%s input null piont!\n",__FUNCTION__);
		return NULL;
	}

	if (stop == 1) {
		stop = 0;
	}

	while (!stop) {
		//g_wifi_osi_funcs._queue_recv(queue_handle, &item,0);
		queue_recv_wrapper(queue_handle, &item,0);
		printf("Recieve message:\n  type: %d, string: %s\n",item.type,item.buff);
	}
	return NULL;
}


void queue_operate_demo(void)
{
	pthread_t queue_send_thread_handle, queue_recieve_thread_handle;
	uint32_t queue_len  = 20;
	pthread_attr_t attr;
	int status = -1;

	//queue_info_t * queue_handle = g_wifi_osi_funcs._queue_create(queue_len,sizeof(queue_item_t));
	queue_info_t * queue_handle = queue_create_wrapper(queue_len,sizeof(queue_item_t));
	if (!queue_handle) {
		printf("%s input null piont!\n",__FUNCTION__);
		return;
	}

	if (pthread_attr_init(&attr) != 0) {
		printf("Error: Cannot initialize ptread attribute\n");
		return;
	}
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, QUEUE_SEND_HANDLER_STACKSIZE);

	status = pthread_create(&queue_send_thread_handle,&attr, queue_send_thread, (void *)queue_handle);
	if (status != 0) {
		printf("queue send: pthread_create failed, status=%d\n", status);
	}

	status = pthread_create(&queue_recieve_thread_handle,&attr, queue_recieve_thread, (void *)queue_handle);
	if (status != 0) {
		printf("queue recieve: pthread_create failed, status=%d\n", status);
	}

	pthread_setname_np(queue_send_thread_handle, "esp32 wifi queue sender");
	pthread_setname_np(queue_recieve_thread_handle, "esp32 wifi queue sender");

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
}


//event group test code

static EventGroupHandle_t test_event_group;

static void *event_group_setbit_thread(void *param)
{
	EventGroupHandle_t xEventGroup = (EventGroupHandle_t)param;

	EventBits_t event_bit[10] = {BIT9,BIT8,BIT7,BIT6,BIT5,BIT4,BIT3,BIT2,BIT1,BIT0};

	if (xEventGroup) {
		for (int i=0;i<10;i++) {
			printf("Setbit_thread: %x\n",event_bit[i]);
			xEventGroupSetBits(xEventGroup, event_bit[i]);
		}
	}
	return NULL;
}



typedef struct {
	EventGroupHandle_t xEventGroup;
	EventBits_t uxBitsToWaitFor;
	BaseType_t xClearOnExit;
	BaseType_t xWaitForAllBits;
	TickType_t xTicksToWait;
}test_group_event_struct_t;



static void *event_group_waitbit_thread(void *param)
{
	test_group_event_struct_t *test_group_event_parameter = (test_group_event_struct_t *)param;
	if (test_group_event_parameter) {
		printf("Wait for 0x%x!, zzzZZZZ!\n",test_group_event_parameter->uxBitsToWaitFor);
		xEventGroupWaitBits(test_group_event_parameter->xEventGroup,test_group_event_parameter->uxBitsToWaitFor,
			                test_group_event_parameter->xClearOnExit,test_group_event_parameter->xWaitForAllBits,
			                test_group_event_parameter->xTicksToWait);

		printf("Week up because 0x%x!, do nothing!\n",test_group_event_parameter->uxBitsToWaitFor);
	}
	return NULL;
}


pthread_t event_group_waitbit_thread_handle[10];
test_group_event_struct_t test_group_event_parameter[10];

static void create_wait_event_threads(void)
{
	int status = -1;
	pthread_attr_t attr;


	EventBits_t event_bit[10] = {BIT0,BIT1,BIT2,BIT3,BIT4,BIT5,BIT6,BIT7,BIT8,BIT9};
	if (pthread_attr_init(&attr) != 0) {
		printf("Error: Cannot initialize ptread attribute\n");
		return;
	}

	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, QUEUE_SEND_HANDLER_STACKSIZE);

	if(test_event_group) {
		for (int i =0;i<10;i++) {
			test_group_event_parameter[i].xEventGroup = test_event_group;
			test_group_event_parameter[i].uxBitsToWaitFor = event_bit[i];
			test_group_event_parameter[i].xClearOnExit = true;
			test_group_event_parameter[i].xWaitForAllBits = false;
			test_group_event_parameter[i].xTicksToWait = 0xffff;
			printf("creat event_group_waitbit_thread\n");
			status = pthread_create(&event_group_waitbit_thread_handle[i],&attr, event_group_waitbit_thread, &test_group_event_parameter[i]);
			if (status != 0) {
				printf("event group waitbit: pthread_create %d failed, status=%d\n",i, status);
			}
			else {
				char buff[10];
				sprintf(buff,"%s%d","event_group_wait",i);
				pthread_setname_np(event_group_waitbit_thread_handle[i], buff);
			}
		}
	}
}

void join_wait_event_threads(void)
{
	for (int i =0;i<10;i++) {
		pthread_join(event_group_waitbit_thread_handle[i],NULL);
	}
}

void event_group_demo(void)
{
	pthread_t event_group_setbit_thread_handle;
	int status = -1;
	pthread_attr_t attr;

	test_event_group = xEventGroupCreate();
	if (!test_event_group) {
		printf("Create event group failed!\n");
		return;
	}

	create_wait_event_threads();

	usleep(3*1000*1000);

	if (pthread_attr_init(&attr) != 0) {
		printf("Error: Cannot initialize ptread attribute\n");
		return;
	}
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, QUEUE_SEND_HANDLER_STACKSIZE);

	status = pthread_create(&event_group_setbit_thread_handle,&attr, event_group_setbit_thread, test_event_group);
	if (status != 0) {
		printf("event_group_setbit_thread_handle: pthread_create failed, status=%d\n", status);
	}

	usleep(3*1000*1000);

	join_wait_event_threads();

	vEventGroupDelete(test_event_group);

	printf("end event group test!\n");

}

pthread_addr_t esp32_demo_entry(pthread_addr_t arg)
{
	printf("start esp32 demo!\n");
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
    usleep(3*1000*1000);
    event_group_demo();
    return NULL;
}
