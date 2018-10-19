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
#include <tinyara/wdog.h>
#include <tinyara/arch.h>
#include <tinyara/cancelpt.h>

#include "esp_attr.h"
#include "esp_phy_init.h"
#include "esp_wifi_os_adapter.h"
#include "nvs.h"
#include "esp_system.h"
#include <sched/sched.h>
#include <semaphore/semaphore.h>

#define pdFALSE         ( 0 )
#define pdTRUE          ( 1 )
#define pdPASS          ( pdTRUE )
#define pdFAIL          ( pdFALSE)
#define OSI_FUNCS_TIME_BLOCKING  0xffffffff


/*extern functions declare*/
extern uint32_t IRAM_ATTR esp_random(void);
int64_t get_instant_time(void);

/*=================seamphore API=================*/

static void *IRAM_ATTR semphr_create_wrapper(uint32_t max, uint32_t init)
{
    if(max > SEM_VALUE_MAX || init > SEM_VALUE_MAX)
        return NULL; 
    
    sem_t *sem = (sem_t*)malloc(sizeof(*sem));
    if(!sem)
        return NULL;
    
    int status =  sem_init(sem, 0, init);
    if(status != OK)
        return NULL;

    return (void*)sem;

}

static void IRAM_ATTR semphr_delete_wrapper(void *semphr)
{
    if(semphr == NULL)
    {
        printf("semphr is NULL\n");
        return;
    }
    sem_destroy(semphr);
    free(semphr);
}

static int32_t IRAM_ATTR semphr_take_from_isr_wrapper(void *semphr, void *hptw)
{
                    
    *(bool*)hptw = pdFALSE;             
    FAR struct tcb_s *stcb = NULL;
    FAR struct tcb_s *rtcb = this_task();
    sem_t *sem = (sem_t *)semphr;
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
            for (stcb = (FAR struct tcb_s *)g_waitingforsemaphore.head; (stcb && stcb->waitsem != sem); stcb = stcb->flink);
            if(stcb)
            {
                if(stcb->sched_priority >= rtcb->sched_priority)
                    *(bool*)hptw = pdTRUE;          
            } 
        }
        /* The semaphore is NOT available*/

        else {
           
            return ret; 
        }
    } 
     else {
        set_errno(EINVAL); 
    }

    leave_cancellation_point();
    irqrestore(saved_state);
    return ret;

}

static int32_t IRAM_ATTR semphr_take_wrapper(void *semphr, uint32_t block_time_tick)
{
    int ret; 
    if(semphr == NULL)
    {
        printf("semphr is NULL\n");
        return pdFAIL;
    }

    if (block_time_tick == OSI_FUNCS_TIME_BLOCKING) { 
        ret = sem_wait(semphr);
        if(ret == OK) 
            return pdPASS;
        else
            return pdFAIL;  
    
    } else {
        clock_t msecs;
        clock_t secs;
        clock_t nsecs;
        struct timespec abstime;
        (void)clock_gettime(CLOCK_REALTIME, &abstime);   

        msecs = TICK2MSEC(block_time_tick);
        secs  = msecs / MSEC_PER_SEC;
        nsecs = (msecs - (secs * MSEC_PER_SEC)) * NSEC_PER_MSEC;
        abstime.tv_sec += secs;
        abstime.tv_nsec += nsecs;   
        ret = sem_timedwait(semphr, &abstime); 
        if(ret == OK) 
            return pdPASS;
        else
            return pdFAIL;  
    }
}

static int32_t IRAM_ATTR semphr_give_wrapper(void *semphr)
{
        int ret;
        if(semphr == NULL)
        {
            printf("semphr is NULL\n");
            return EINVAL;
        }       
        ret = sem_post(semphr); 
        if(ret == OK) 
            return pdPASS;
        else
            return pdFAIL;  
}

static int32_t IRAM_ATTR semphr_give_from_isr_wrapper(void *semphr, void *hptw)
{
       *(int*)hptw = pdFALSE; 
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
        printf("recursive_mutex_test: ERROR pthread_mutexattr_settype failed, status=%d\n", status);
        return NULL;
    }   
    status =  pthread_mutex_init(mutex, &mattr);
    if (status) {
        return NULL;
    }  
    return (void*)mutex;
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
    return (void*)mutex; 

}

static void IRAM_ATTR mutex_delete_wrapper(void *mutex)
{
        if(mutex == NULL)
        {
            printf("mutex is NULL\n");
            return;
        }
        pthread_mutex_destroy(mutex);
        free(mutex);
}


static int32_t IRAM_ATTR mutex_lock_wrapper(void *mutex)
{
        if(mutex == NULL)
        {
            printf("mutex is NULL\n");
            return EINVAL;
        }
        int ret = pthread_mutex_lock(mutex);
        if(ret)
            return pdFAIL;
        return pdPASS;

}

static int32_t IRAM_ATTR mutex_unlock_wrapper(void *mutex)
{
        if(mutex == NULL)
        {
            printf("mutex is NULL\n");
            return EINVAL;
        }
        int ret = pthread_mutex_unlock(mutex);
        if(ret)
            return pdFAIL;
        return pdPASS;
}

/*=================task control API=================*/
static int32_t IRAM_ATTR task_create_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *    
task_handle)
{
    int pid = task_create(name, prio, stack_depth, task_func, param);
    if(pid < 0)
        return pdFAIL;
 
    task_handle = (void*)pid;
    return pdPASS;
}

static int32_t IRAM_ATTR task_create_pinned_to_core_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio,
void *task_handle, uint32_t core_id)
{
#ifndef CONFIG_SMP
    return task_create_wrapper(task_func, name, stack_depth, param, prio, task_handle);
#else
/*currently TizenRT not support SMP*/
    return pdFAIL;
#endif
}

static void IRAM_ATTR task_delete_wrapper(void *task_handle)
{
    if(task_handle < 0 )
        return;
    int pid = (int)task_handle;

    task_delete(pid); 
 
}

static void IRAM_ATTR task_delay_wrapper(uint32_t tick)
{
    uint64_t usecs = TICK2USEC(tick);
    usleep(usecs);
}

static int curpid;
static void* IRAM_ATTR task_get_current_task_wrapper(void)
{
    curpid = getpid();
    return (void*)&curpid;
}

static inline int32_t IRAM_ATTR task_ms_to_tick_wrapper(uint32_t ms)
{
    return (int32_t)MSEC2TICK(ms);
}

static inline int32_t IRAM_ATTR task_get_max_priority_wrapper(void)
{
    return (int32_t)(SCHED_PRIORITY_MAX);
}

uint32_t get_free_heap_size( void )
{
    struct mallinfo hinfo = mallinfo();
    return hinfo.ordblks;
}

static inline int32_t IRAM_ATTR _is_in_isr_wrapper(void)
{
    return (int32_t)up_interrupt_context();
}


/*=================wifi RF API=================*/

static inline int32_t IRAM_ATTR esp_phy_rf_deinit_wrapper(uint32_t module)
{
   return  esp_phy_rf_deinit((phy_rf_module_t)module);
}

static inline int32_t IRAM_ATTR phy_rf_init_wrapper(const void* init_data, uint32_t mode, void* calibration_data, uint32_t module)
{
    return esp_phy_rf_init( init_data, mode, calibration_data, module);
}

static inline void IRAM_ATTR esp_phy_load_cal_and_init_wrapper(uint32_t module)
{
    return esp_phy_load_cal_and_init((phy_rf_module_t)module);
}

static inline int32_t esp_read_mac_wrapper(uint8_t* mac, uint32_t type)
{
    return esp_read_mac(mac, (esp_mac_type_t)type);
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
    if(timer == NULL)
    {   
        printf("timer is NULL\n");
        return;
    }   

    WDOG_ID wdog =(WDOG_ID)timer;
    int delay = MSEC2TICK(tmout);
    wd_start(wdog, delay, NULL, 0, NULL);
}

static void IRAM_ATTR timer_disarm_wrapper(void *timer)
{
    if(timer == NULL)
    {   
        printf("timer is NULL\n");
        return;
    }   
    WDOG_ID wdog =(WDOG_ID)timer;
    wd_cancel(wdog);

}

static void IRAM_ATTR timer_done_wrapper(void *ptimer)
{
    if(ptimer == NULL)
    {   
        printf("timer is NULL\n");
        return;
    }   
    WDOG_ID wdog =(WDOG_ID)ptimer;
    wd_delete(wdog);
}

static void IRAM_ATTR timer_setfn_wrapper(void *ptimer, void *pfunction, void *parg)
{
    if(ptimer == NULL)
    {   
        printf("timer is NULL\n");
        return;
    }   
    WDOG_ID wdog =(WDOG_ID)ptimer;
    wd_start(wdog, 0, pfunction, 1, parg);
}

static void IRAM_ATTR timer_arm_us_wrapper(void *ptimer, uint32_t us, bool repeat)
{
    if(ptimer == NULL)
    {   
        printf("timer is NULL\n");
        return;
    }   

    WDOG_ID wdog =(WDOG_ID)ptimer;
    int delay = USEC2TICK(us);
    wd_start(wdog, delay, NULL, 0, NULL);
}

static inline int32_t IRAM_ATTR get_time_wrapper(void *t)
{
    return (int32_t)gettimeofday((struct timeval*) t, NULL);
}

/*=================Miscellaneous API================================*/

static inline int32_t IRAM_ATTR esp_os_get_random_wrapper(uint8_t *buf, size_t len)
{
    return (int32_t)os_get_random((unsigned char *)buf, len);

}

typedef enum {
    ESP_LOG_NONE,       /*!< No log output */
    ESP_LOG_ERROR,      /*!< Critical errors, software module can not recover on its own */
    ESP_LOG_WARN,       /*!< Error conditions from which recovery measures have been taken */
    ESP_LOG_INFO,       /*!< Information messages which describe normal flow of events */
    ESP_LOG_DEBUG,      /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
    ESP_LOG_VERBOSE     /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
} esp_log_level_t;

static void IRAM_ATTR log_write_wrapper(uint32_t level,
        const char* tag,
        const char* format, ...)
{
    switch(level)
    {
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
    return esp_modem_sleep_enter((modem_sleep_module_t)module);
}

static inline esp_err_t esp_modem_sleep_exit_wrapper(uint32_t module)
{
    return esp_modem_sleep_exit((modem_sleep_module_t)module);

}
static inline esp_err_t esp_modem_sleep_register_wrapper(uint32_t module)
{
    return esp_modem_sleep_register((modem_sleep_module_t)module); 
}

static inline esp_err_t esp_modem_sleep_deregister_wrapper(uint32_t module)
{
    return esp_modem_sleep_deregister((modem_sleep_module_t)module); 
}


/*=================espwifi NVS API=========================*/

static inline int32_t esp_nvs_open_wrapper(const char* name, uint32_t open_mode, uint32_t *out_handle)
{
    return nvs_open(name, (nvs_open_mode)open_mode, (nvs_handle*)out_handle);
}

/*=================espwifi smart config API=========================*/
/* Will complete next stage, not block WIFI ENABLE*/



/*=================espwifi os adapter interface =====================*/

wifi_osi_funcs_t g_wifi_osi_funcs = {
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
    ._task_create_pinned_to_core = task_create_pinned_to_core_wrapper,
    ._task_create = task_create_wrapper,
    ._task_delete = task_delete_wrapper,
    ._task_delay = task_delay_wrapper,
    ._task_ms_to_tick = task_ms_to_tick_wrapper,
    ._task_get_current_task = task_get_current_task_wrapper,
    ._task_get_max_priority = task_get_max_priority_wrapper,
    ._is_in_isr = _is_in_isr_wrapper,
    ._malloc = malloc,
    ._free = free,
    ._get_free_heap_size = get_free_heap_size,
    ._rand = esp_random,
    ._phy_rf_init = phy_rf_init_wrapper,
    ._phy_rf_deinit = esp_phy_rf_deinit_wrapper,
    ._phy_load_cal_and_init = esp_phy_load_cal_and_init_wrapper,
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
    ._modem_sleep_enter = esp_modem_sleep_enter_wrapper,
    ._modem_sleep_exit = esp_modem_sleep_exit_wrapper,
    ._modem_sleep_register = esp_modem_sleep_register_wrapper,
    ._modem_sleep_deregister = esp_modem_sleep_deregister_wrapper,

};
