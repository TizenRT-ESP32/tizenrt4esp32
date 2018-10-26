#ifndef ESP_QUEUE_API_H
#define ESP_QUEUE_API_H


#ifdef __cplusplus
extern "C" {
#endif

#include "esp_define.h"


typedef struct {
	bool valid;
	uint32_t mq_item_size;
	mqd_t mqd_fd;
}queue_info_t;


void *queue_create_wrapper(uint32_t queue_len, uint32_t item_size);
void  queue_delete_wrapper(void *queue);
int32_t queue_send_wrapper(void *queue, void *item, uint32_t block_time_tick);
int32_t queue_send_from_isr_wrapper(void *queue, void *item, void *hptw);
int32_t queue_send_to_back_wrapper(void *queue, void *item, uint32_t block_time_tick);
int32_t queue_send_to_front_wrapper(void *queue, void *item, uint32_t block_time_tick);
int32_t queue_recv_wrapper(void *queue, void *item, uint32_t block_time_tick);
uint32_t queue_msg_waiting_wrapper(void *queue);



#ifdef __cplusplus
}
#endif

#endif /* ESP_QUEUE_API_H */