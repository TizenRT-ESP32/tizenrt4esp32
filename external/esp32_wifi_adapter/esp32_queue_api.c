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


#define MAX_QUEUE_INFO 20
const char *mq_name = "mq_wifi";
enum {
	NORMAL = 0,
	MIDDLE,
	HIGH,
}queue_prio_e;



queue_info_t queues_info[MAX_QUEUE_INFO];


void * IRAM_ATTR queue_create_wrapper(uint32_t queue_len, uint32_t item_size)
{
#define NAME_LEN 20
	char name[NAME_LEN];
	bool flag = false;
	uint32_t mq_id = 0;

	for(int i=0;i<MAX_QUEUE_INFO;i++) {
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
	attr.mq_maxmsg  = queue_len;
	attr.mq_msgsize = item_size; //max msg size
	attr.mq_flags   = 0;

	sprintf(name,"%s%d",mq_name,mq_id);

	/*Invalid param*/
	queues_info[mq_id].mqd_fd = mq_open(name,O_RDWR|O_CREAT, 0666, &attr);
	if (queues_info[mq_id].mqd_fd == (mqd_t)-1) {
		printf("queue_create_wrapper FAIL: mq_open\n");
		return NULL;
	}
	queues_info[mq_id].valid = true;
	queues_info[mq_id].mq_item_size = item_size;
	return &queues_info[mq_id];
}

void IRAM_ATTR  queue_delete_wrapper(void *queue)
{
	queue_info_t *queue_info = NULL;
	if (queue) {
		queue_info = (queue_info_t *)queue;
		if(queue_info->mqd_fd) {
			mq_close(queue_info->mqd_fd);
		}
		queue_info->mq_item_size = 0;
		queue_info->valid = false;
	}
	return;
}

int32_t IRAM_ATTR queue_send_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
	int32_t ret;
	queue_info_t *queue_info = NULL;
	if (queue) {
		queue_info = (queue_info_t *)queue;
		if(queue_info->mqd_fd) {
			ret = mq_send(queue_info->mqd_fd,(char *)item,queue_info->mq_item_size,NORMAL);
			if(ret){
				return pdFAIL;
			}
			return pdPASS;
		}
	}
	return pdFAIL;
}

int32_t IRAM_ATTR queue_send_from_isr_wrapper(void *queue, void *item, void *hptw)
{
	if(!queue || !item) {
		return pdFAIL;
	}
	return queue_send_wrapper(queue,item,0);
}

int32_t IRAM_ATTR queue_send_to_back_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
	if(!queue || !item) {
		return pdFAIL;
	}
	return queue_send_wrapper(queue,item,0);
}

int32_t IRAM_ATTR queue_send_to_front_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
	int32_t ret;
	queue_info_t *queue_info = NULL;
	if(queue) {
		queue_info = (queue_info_t *)queue;
		if(queue_info->mqd_fd) {
			ret = mq_send(queue_info->mqd_fd,(char *)item,queue_info->mq_item_size,HIGH);
			if(ret) {
				return pdFAIL;
			}
			return pdPASS;
		}
	}
	return pdFAIL;
}

int32_t IRAM_ATTR queue_recv_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
	queue_info_t *queue_info = NULL;
	size_t msglen = 0;
	int prio = 0;
	int32_t ret;
	if(queue) {
		queue_info = (queue_info_t *)queue;
		if (queue_info->mqd_fd) {
			msglen = queue_info->mq_item_size;
			ret = mq_receive(queue_info->mqd_fd,(char *)item,msglen,&prio);
			if(ret) {
				return pdFAIL;
			}
			return pdPASS;
		}
	}
	return pdFAIL;
}

uint32_t IRAM_ATTR queue_msg_waiting_wrapper(void *queue)
{
	queue_info_t *queue_info = NULL;
	if (queue) {
		queue_info = (queue_info_t *)queue;
		if(queue_info->mqd_fd && queue_info->mqd_fd->msgq) {
			return queue_info->mqd_fd->msgq->nmsgs;
		}
	}
	return pdFAIL;
}