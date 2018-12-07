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

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/
#define MAX_QUEUE_INFO 20

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
	queues_info[mq_id].mqd_fd = mq_open(name, O_RDWR | O_CREAT, 0666, &attr);
	if (queues_info[mq_id].mqd_fd == (mqd_t)-1) {
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
		if (queue_info->mqd_fd) {
			mq_close(queue_info->mqd_fd);
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
int32_t IRAM_ATTR queue_send_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
	int32_t ret;
	queue_info_t *queue_info = NULL;
	struct timespec abstime;
	clock_t msecs;
	clock_t secs;
	clock_t nsecs;

	if (!queue || !item) {
		return pdFAIL;
	}

	(void)clock_gettime(CLOCK_REALTIME, &abstime);
	msecs = TICK2MSEC(block_time_tick);
	secs = msecs / MSEC_PER_SEC;
	nsecs = (msecs - (secs * MSEC_PER_SEC)) * NSEC_PER_MSEC;
	abstime.tv_sec += secs;
	abstime.tv_nsec += nsecs;
	
	queue_info = (queue_info_t *)queue;
	if (queue_info->mqd_fd) {
		ret = mq_timedsend(queue_info->mqd_fd, (char *)item, queue_info->mq_item_size, NORMAL, &abstime);
		if (ret) {
			return pdFAIL;
		}
		return pdPASS;
	}

	return pdFAIL;
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
		
	queue_info = (queue_info_t *)queue;
	if (queue_info->mqd_fd) {
		ret = mq_send(queue_info->mqd_fd, (char *)item, queue_info->mq_item_size, NORMAL);
		if (ret) {
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
	clock_t msecs;
	clock_t secs;
	clock_t nsecs;

	if (!queue || !item) {
		return pdFAIL;
	}
	
	(void)clock_gettime(CLOCK_REALTIME, &abstime);
	msecs = TICK2MSEC(block_time_tick);
	secs = msecs / MSEC_PER_SEC;
	nsecs = (msecs - (secs * MSEC_PER_SEC)) * NSEC_PER_MSEC;
	abstime.tv_sec += secs;
	abstime.tv_nsec += nsecs;
	
	queue_info = (queue_info_t *)queue;
	if (queue_info->mqd_fd) {
		ret = mq_timedsend(queue_info->mqd_fd, (char *)item, queue_info->mq_item_size, HIGH, &abstime);
		if (ret) {
			return pdFAIL;
		}
		return pdPASS;
	}

	return pdFAIL;
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

	(void)clock_gettime(CLOCK_REALTIME, &abstime);
	msecs = TICK2MSEC(block_time_tick);
	secs = msecs / MSEC_PER_SEC;
	nsecs = (msecs - (secs * MSEC_PER_SEC)) * NSEC_PER_MSEC;
	abstime.tv_sec += secs;
	abstime.tv_nsec += nsecs;

	queue_info = (queue_info_t *)queue;
	if (queue_info->mqd_fd) {
		msglen = queue_info->mq_item_size;
		ret = mq_timedreceive(queue_info->mqd_fd, (char *)item, msglen, &prio, &abstime);
		if (ret) {
			return pdFAIL;
		}
		return pdPASS;
	}
	
	return pdFAIL;
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
	if (queue_info->mqd_fd) {
		msglen = queue_info->mq_item_size;
		ret = mq_tryreceive_isr(queue_info->mqd_fd, (char *)item, msglen, &prio);
		if (ret) {
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
 *	queue - The queue to receive messages.
 * Return:
 *	Returns the count of messages in the queue if success,  returns pdFAIL if failure.
 *
 ****************************************************************************/
uint32_t IRAM_ATTR queue_msg_waiting_wrapper(void *queue)
{
	queue_info_t *queue_info = NULL;
	
	if (queue) {
		queue_info = (queue_info_t *)queue;
		if (queue_info->mqd_fd && queue_info->mqd_fd->msgq) {
			return queue_info->mqd_fd->msgq->nmsgs;
		}
	}
	return pdFAIL;
}
