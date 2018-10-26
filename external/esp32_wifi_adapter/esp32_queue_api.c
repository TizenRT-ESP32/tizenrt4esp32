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