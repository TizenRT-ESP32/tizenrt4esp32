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
#include <tinyara/arch.h>
#include "esp_attr.h"
#include "esp32_queue_api.h"
#include "mq_tryreceive.h"

#include <tinyara/clock.h>
/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/
#define MAX_QUEUE_INFO 10

/************************************************************************
 * Private Type Declarations
 ************************************************************************/
enum {
	NORMAL = 0,
	MIDDLE,
	HIGH,
} queue_prio_e;

/************************************************************************
 * Public Variables
 ************************************************************************/

/************************************************************************
 * Private Variables
 ************************************************************************/
const char *mq_name = "mq_wifi";
static queue_info_t queues_info[MAX_QUEUE_INFO];

static unsigned long long diff_time(struct timeval *x, struct timeval *y)
{
    
    unsigned long long start = x->tv_sec * 1000 + x->tv_usec/1000;
    unsigned long long end = y->tv_sec * 1000 + y->tv_usec/1000; 
    return (end-start);
}

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/
/****************************************************************************
 * Name: queue_create_wrapper
 *
 * Description:
 *	This function is the wrapper of mq_open to establish a connection between a named 
 * message queue and the calling task.
 *   
 * Inputs:
 *	queue_len - The length of the queue.
 *	item_size - The size of each node of the queue.
 *
 * Return:
 *	Returns the pionter of the created mqueue, but NULL in a failure.
 *
 ****************************************************************************/
void *IRAM_ATTR queue_create_wrapper(uint32_t queue_len, uint32_t item_size)
{
#define NAME_LEN 20
	char name[NAME_LEN];
	bool flag = false;
	uint32_t mq_id = 0;

	for (int i = 0; i < MAX_QUEUE_INFO; i++) {
		if (!queues_info[i].valid) {
			flag = true;
			mq_id = i;
			break;
		}
	}

	if (!flag) {
		printf("queue_create_wrapper create failed!\n");
		return NULL;
	}

	struct mq_attr attr;
	attr.mq_maxmsg = queue_len;
	attr.mq_msgsize = item_size;
	attr.mq_flags = 0;

	sprintf(name, "%s%d", mq_name, mq_id);

	/*Invalid param */
	queues_info[mq_id].mqd_fd_send = mq_open(name, O_RDWR | O_CREAT, 0666, &attr);
	if (queues_info[mq_id].mqd_fd_send == (mqd_t)ERROR) {
		printf("queue_create_wrapper FAIL: mq_open\n");
		return NULL;
	}

	queues_info[mq_id].mqd_fd_recv = mq_open(name, O_RDWR | O_CREAT, 0666, &attr);
	if (queues_info[mq_id].mqd_fd_recv == (mqd_t)ERROR) {
		printf("queue_create_wrapper FAIL: mq_open\n");
		return NULL;
	}	
	queues_info[mq_id].valid = true;
	queues_info[mq_id].mq_item_size = item_size;
	return &queues_info[mq_id];
}

/****************************************************************************
 * Name: queue_delete_wrapper
 *
 * Description:
 *	This function is the wrapper of mq_close to close a connection between a named 
 * message queue and the calling task.
 *   
 * Inputs:
 *	queue - The queue to be closed.
 *
 * Return:
 *
 ****************************************************************************/
void IRAM_ATTR queue_delete_wrapper(void *queue)
{
	queue_info_t *queue_info = NULL;
	
	if (queue) {
		queue_info = (queue_info_t *)queue;
		if (queue_info->mqd_fd_send != (mqd_t)ERROR) {
			mq_close(queue_info->mqd_fd_send);
			queue_info->mqd_fd_send = NULL;
		}
		
		if (queue_info->mqd_fd_recv != (mqd_t)ERROR) {
			mq_close(queue_info->mqd_fd_recv);
			queue_info->mqd_fd_recv = NULL;
		}

		queue_info->mq_item_size = 0;
		queue_info->valid = false;
	}
	return;
}

/****************************************************************************
 * Name: queue_send_wrapper
 *
 * Description:
 *	This function is the wrapper of mq_timedsend to adds the specificied message (msg) to 
 * the message queue (mqdes).  
 *   
 * Inputs:
 *	queue - The queue to send messages.
 *	item - The message to be send.
 *	block_time_tick - the time (in Ticks) to wait until a timeout is decleared
 * Return:
 *	Returns pdPASS if success, pdFAIL if failure.
 *
 ****************************************************************************/
static bool queue_is_full(void *queue)
{
    queue_info_t *queue_info = (queue_info_t *)queue;
#if 0
    if(queue_info->mqd_fd_send->msgq->nmsgs >= queue_info->mqd_fd_send->msgq->maxmsgs)
    {
        ets_printf("queue full\n");
    }
#endif
   // ets_printf("queue size %d\n", queue_info->mqd_fd_send->msgq->nmsgs);
    return false;
}

int32_t IRAM_ATTR queue_send_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
    int32_t ret;
	queue_info_t *queue_info = NULL;
	struct timespec abstime;
	uint32_t secs;
	uint32_t msecs;
	uint32_t nsecs;
    uint32_t carry;

	if (!queue || !item) {
		return pdFAIL;
	}
    //for debug
//    queue_is_full(queue);


    unsigned long long consume = 0;
    struct timeval start_t;
    struct timeval end_t; 
    gettimeofday(&start_t, NULL); 

	queue_info = (queue_info_t *)queue;
	if (queue_info->mqd_fd_send != (mqd_t)ERROR) {
		if (block_time_tick == 0xFFFFFFFF) {
			ret = mq_send(queue_info->mqd_fd_send, (char *)item, queue_info->mq_item_size, NORMAL);
			if (ret == ERROR) {
               // ets_printf("queue_send_wrapper fail, errcode = %d\n",  get_errno());
				return pdFAIL;
			}
       
        }
       
        else {
			(void)clock_gettime(CLOCK_REALTIME, &abstime);
			msecs = TICK2MSEC(block_time_tick);
			secs = msecs / MSEC_PER_SEC;
			nsecs = (msecs - (secs * MSEC_PER_SEC)) * NSEC_PER_MSEC;
			abstime.tv_sec += secs;
			abstime.tv_nsec += nsecs;

           // ets_printf("1 tick %u, to msecs %u, secs %u, usecs %ld\n", block_time_tick,  msecs, abstime.tv_sec, abstime.tv_nsec);
            if (abstime.tv_nsec >= NSEC_PER_SEC) {
                  //  ets_printf("over 1s\n");
                    carry       = abstime.tv_nsec / NSEC_PER_SEC;
                    abstime.tv_sec  += carry;
                    abstime.tv_nsec -= (carry * NSEC_PER_SEC);
            }

           // ets_printf("2 tick %u, to msecs %u, secs %u, usecs %ld\n", block_time_tick,  msecs, abstime.tv_sec, abstime.tv_nsec);

			ret = mq_timedsend(queue_info->mqd_fd_send, (char *)item, queue_info->mq_item_size, NORMAL, &abstime);
			if (ret == ERROR) {
                ets_printf(" queue_send_wrapper fail,  errcode = %d\n", get_errno());
				return pdFAIL;
			}
		}
	} else {
		return pdFAIL;
	}
    gettimeofday(&end_t, NULL);
    consume = diff_time(&start_t, &end_t);
//    if(consume > 5) 
  //      ets_printf("queue_send_wrapper run  %llu ms\n", consume); 
	return pdPASS;;
}

/****************************************************************************
 * Name: queue_send_from_isr_wrapper
 *
 * Description:
 *	This function is the wrapper of mq_send to adds the specificied message (msg) to 
 * the message queue (mqdes).
 *   
 * Inputs:
 *	queue - The queue to send messages.
 *	item - The message to be send.
 *	hptw - useless in TizenRT, but needed in the declaration of esp32 wifi os adapter.
 * Return:
 *	Returns pdPASS if success, pdFAIL if failure.
 *
 ****************************************************************************/
int32_t IRAM_ATTR queue_send_from_isr_wrapper(void *queue, void *item, void *hptw)
{
	int32_t ret;
	queue_info_t *queue_info = NULL;

	if (!queue || !item) {
		return pdFAIL;
	}
	//for debug	
    //queue_is_full(queue);
	queue_info = (queue_info_t *)queue;
	if (queue_info->mqd_fd_send != (mqd_t)ERROR) {
		ret = mq_send(queue_info->mqd_fd_send, (char *)item, queue_info->mq_item_size, NORMAL);
		if (ret == ERROR) {
            ets_printf("queue_send_from_isr_wrapper error\n");
			return pdFAIL;
		}
		return pdPASS;
	}
	return pdFAIL;
}

/****************************************************************************
 * Name: queue_send_to_back_wrapper
 *
 * Description:
 *	This function is the wrapper of queue_send_wrapper to adds the specificied message (msg) to 
 * the message queue (mqdes) end.  
 *   
 * Inputs:
 *	queue - The queue to send messages.
 *	item - The message to be send.
 *	block_time_tick - the time (in Ticks) to wait until a timeout is decleared
 * Return:
 *	Returns pdPASS if success, pdFAIL if failure.
 *
 ****************************************************************************/
int32_t IRAM_ATTR queue_send_to_back_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
	if (!queue || !item) {
		return pdFAIL;
	}
	return queue_send_wrapper(queue, item, block_time_tick);
}

/****************************************************************************
 * Name: queue_send_to_front_wrapper
 *
 * Description:
 *	This function is the wrapper of mq_timedsend to adds the specificied message (msg) to 
 * the message queue (mqdes) head.  
 *   
 * Inputs:
 *	queue - The queue to send messages.
 *	item - The message to be send.
 *	block_time_tick - the time (in Ticks) to wait until a timeout is decleared
 * Return:
 *	Returns pdPASS if success, pdFAIL if failure.
 *
 ****************************************************************************/
int32_t IRAM_ATTR queue_send_to_front_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
	int32_t ret;
	queue_info_t *queue_info = NULL;
	struct timespec abstime;
	uint32_t msecs;
	uint32_t secs;
	uint32_t nsecs;

	if (!queue || !item) {
		return pdFAIL;
	}
	
	queue_info = (queue_info_t *)queue;
	if (queue_info->mqd_fd_send != (mqd_t)ERROR) {
		if (block_time_tick == 0xFFFFFFFF) {
			ret = mq_send(queue_info->mqd_fd_send, (char *)item, queue_info->mq_item_size, NORMAL);
			if (ret == ERROR) {
				return pdFAIL;
			}
		} else {
			(void)clock_gettime(CLOCK_REALTIME, &abstime);
			msecs = TICK2MSEC(block_time_tick);
			secs = msecs / MSEC_PER_SEC;
			nsecs = (msecs - (secs * MSEC_PER_SEC)) * NSEC_PER_MSEC;
			abstime.tv_sec += secs;
			abstime.tv_nsec += nsecs;
		
			ret = mq_timedsend(queue_info->mqd_fd_send, (char *)item, queue_info->mq_item_size, HIGH, &abstime);
			if (ret == ERROR) {
				return pdFAIL;
			}
		}
	} else {
		return pdFAIL;
	}	
	return pdPASS;
}

/****************************************************************************
 * Name: queue_recv_wrapper
 *
 * Description:
 *	This function is the wrapper of mq_timedreceive to receives the oldest of the highest priority  
 *messages from the message queue. 
 *   
 * Inputs:
 *	queue - The queue to receive messages.
 *	item - The received message.
 *	block_time_tick - the time (in Ticks) to wait until a timeout is decleared
 * Return:
 *	Returns pdPASS if success, pdFAIL if failure.
 *
 ****************************************************************************/
int32_t IRAM_ATTR queue_recv_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
	queue_info_t *queue_info = NULL;
	size_t msglen = 0;
	int prio = 0;
	int32_t ret;
	struct timespec abstime;
	clock_t msecs;
	clock_t secs;
	clock_t nsecs;

	if (!queue || !item) {
		return pdFAIL;
	}

    //queue_is_full(queue);
#if 0 
    unsigned long long consume = 0;
    struct timeval start_t;
    struct timeval end_t; 
    gettimeofday(&start_t, NULL); 

#endif
	
	queue_info = (queue_info_t *)queue;
	if (queue_info->mqd_fd_recv != (mqd_t)ERROR) {
		msglen = queue_info->mq_item_size;
		if (block_time_tick == 0xFFFFFFFF) {
			ret = mq_receive(queue_info->mqd_fd_recv, (char *)item, msglen, &prio);
			if (ret == ERROR) {
                ets_printf("queue_recv_wrapper failed\n");
				return pdFAIL;
			}
		} else {
			(void)clock_gettime(CLOCK_REALTIME, &abstime);
			msecs = TICK2MSEC(block_time_tick);
			secs = msecs / MSEC_PER_SEC;
			nsecs = (msecs - (secs * MSEC_PER_SEC)) * NSEC_PER_MSEC;
			abstime.tv_sec += secs;
			abstime.tv_nsec += nsecs;
			ret = mq_timedreceive(queue_info->mqd_fd_recv, (char *)item, msglen, &prio, &abstime);
			if (ret == ERROR) {
                ets_printf("queue_recv_wrapper failed\n");
				return pdFAIL;
			}
		}	
	} else {
		return pdFAIL;	
	}	

#if 0
    gettimeofday(&end_t, NULL);
    consume = diff_time(&start_t, &end_t);
    if(consume > 5)
      ets_printf("queue_recv_wrapper run  %llu ms\n", consume); 
#endif
    return pdPASS;
}

/****************************************************************************
 * Name: queue_recv_from_isr_wrapper
 *
 * Description:
 *	This function is the wrapper of mq_timedreceive to receives the oldest of the highest priority  
 *messages from the message queue in an interrupt handle. 
 *   
 * Inputs:
 *	queue - The queue to receive messages.
 *	item - The received message.
 *	hptw - useless in TizenRT, but needed in the declaration of esp32 wifi os adapter.
 * Return:
 *	Returns pdPASS if success, pdFAIL if failure.
 *
 ****************************************************************************/
int32_t IRAM_ATTR queue_recv_from_isr_wrapper(void *queue, void *item, int32_t *const hptw)
{
	queue_info_t *queue_info = NULL;
	size_t msglen = 0;
	int prio = 0;
	int32_t ret;
	
	if (!queue || !item) {
		return pdFAIL;
	}
	
	queue_info = (queue_info_t *)queue;
	if (queue_info->mqd_fd_recv != (mqd_t)ERROR) {
		msglen = queue_info->mq_item_size;
		ret = mq_tryreceive_isr(queue_info->mqd_fd_recv, (char *)item, msglen, &prio);
		if (ret == ERROR) {
			return pdFAIL;
		}
		return pdPASS;
	}
	return pdFAIL;
}

/****************************************************************************
 * Name: queue_msg_waiting_wrapper
 *
 * Description:
 *	This function gets the message count of message queue. 
 *   
 * Inputs:
 *	queue - The queue to send messages.
 * Return:
 *	Returns the count of messages in the queue if success,  returns pdFAIL if failure.
 *
 ****************************************************************************/
uint32_t IRAM_ATTR queue_msg_waiting_wrapper(void *queue)
{
    ets_printf("queue_msg_waiting_wrapper\n"); 

    queue_info_t *queue_info = NULL;
	
	if (queue) {
		queue_info = (queue_info_t *)queue;
		if ((queue_info->mqd_fd_send != (mqd_t)ERROR) && 
			(queue_info->mqd_fd_send->msgq != NULL)) {
            ets_printf("wait msg = %d\n",  queue_info->mqd_fd_send->msgq->nmsgs); 
			return queue_info->mqd_fd_send->msgq->nmsgs;
		}
	}
	return pdFAIL;
}
