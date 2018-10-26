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

/*testcase*/

#define RANDOM_TEST_TIME (20)
void test_rand(void)
{
    uint32_t ret;
    for(int i = 0; i < RANDOM_TEST_TIME; i++)
    {
        ret = esp_random();
        printf("random %d =  %u\n", i+1, ret);
    }
    printf("====Test random sucess====\n");
}

void test_mutex(void)
{
    void *mutex = mutex_create_wrapper();
    if(!mutex)
        printf("mutex is NULL\n");
    static int num = 0; 
    mutex_lock_wrapper(mutex);
    num++;
    mutex_unlock_wrapper(mutex);
    mutex_delete_wrapper(mutex);
    printf("====Test mutex sucess====\n");
}


void test_sem(void)
{
    void *sem = semphr_create_wrapper(1, 1);
    if(!sem)
        printf("sem is NULL\n");
    static int num = 0; 
    semphr_take_wrapper(sem, 100000);
    num++;
    semphr_give_wrapper(sem);
    semphr_delete_wrapper(sem);
    printf("====Test sem sucess====\n");
}

int handle; 
void *taskfunc(void *parm)
{
    task_delay_wrapper(100);      
    if(is_in_isr_wrapper() != 0)
        printf("Test isr error\n");
    task_delete_wrapper(&handle);
    printf("====Test task success====\n");
}

void test_task(void)
{
    task_create_wrapper(taskfunc, "test_task_control", 1024, NULL, 100, &handle);
        
}

ETSTimer mytimer;

void *timer_func(void *parm)
{
    timer_disarm_wrapper(&mytimer);
    timer_done_wrapper(&mytimer);
    printf("===Test timer success====\n");
}

void test_timer(void)
{
  timer_arm_wrapper(&mytimer, 10, 0);
  timer_setfn_wrapper(&mytimer, timer_func, NULL);
}

void test_time(void)
{ 
  struct timeval t;    
  get_time_wrapper(&t);
  //printf("%ds, %dus\n", t.tv_sec, t.tv_usec);  
  //int64_t period = get_instant_time();
  //printf("elapsed time from booting %lu\n", period);
  printf("===Test gettime success====\n");
}


pthread_addr_t esp32_demo_entry(pthread_addr_t arg)
{
    test_rand();
    test_mutex();
    test_sem();
    test_time();
    test_task();
    //wait child task exit
    sleep(2);
    test_timer();
    //wait timer handler exit
    sleep(2);
	return NULL;
}

