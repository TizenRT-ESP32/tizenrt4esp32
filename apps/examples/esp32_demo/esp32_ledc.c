#include <stdio.h>
#include <tinyara/ledc.h>
#include <fcntl.h>

/* select gpio pin4 as ledc pwm output, connect to LED power*/

void test_ledc_soft(void)
{
    int fd1;
    struct ledc_info_s ledc_info;
    printf("test_ledc_soft\n");
    fd1 = open("/dev/ledc", O_RDWR);
    if (fd1 < 0) {
        printf("fd open fail\n");
        return;
    }   
    ledc_info.frequency = 1000; 
    ledc_info.ishwfade = 0;
    int dutys1[] = {256, 1024, 4096, 8191};
    for( int i = 0; i < 4; i++) {   

        ledc_info.duty = dutys1[i];  
        ioctl(fd1, LEDCIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&ledc_info));
        ioctl(fd1, LEDCIOC_START);
        printf("current level : %d\n",i+1);
        up_mdelay(300);
    }   
    ioctl(fd1, LEDCIOC_STOP);
    close(fd1);
}


