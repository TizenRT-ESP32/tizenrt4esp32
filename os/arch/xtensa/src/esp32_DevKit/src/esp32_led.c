#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/ledc.h>
#include "esp32_ledc.h"

FAR struct ledc_lowerhalf_s *esp32_ledcinitialize(int i);

int board_ledc_setup(void)
{
#ifdef CONFIG_LEDC
    FAR struct ledc_lowerhalf_s *ledc;
    char path[10];
    int ret = 0;
    int i;
    for (i = 0; i < 1; i++) {
    
        ledc = esp32_ledcinitialize(i);
        if (!ledc) {
            printf("Failed to get the PWM lower half\n");
            return -ENODEV;
        }   
        
        snprintf(path, sizeof(path), "/dev/ledc%d", i); 

        printf("path = %s\n", path);

        ret = ledc_register(path, ledc);
        if (ret < 0) {
            printf("ledc_register failed: %d\n", ret);
            return ret;
        }   
    }   
#endif
    return OK; 
}
