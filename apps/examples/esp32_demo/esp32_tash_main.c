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
 * examples/opus_thread/hello_tash_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>
#include <unistd.h>
#include <apps/shell/tash.h>

#include <tinyara/fs/fs.h>
#include <tinyara/fs/ioctl.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

#if defined(CONFIG_ADC)
#include <tinyara/analog/adc.h>
#define ADC_MAX_SAMPLES	4
#define TEST_TIME_SEC   30
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/
#define ESP32_TASH_PRI      100
#define ESP32_TASH_STAKSIZE (1024*64)

/****************************************************************************
 * Private Data & Functions
 ****************************************************************************/
/* example */

/*  Call-back function registered in TASH.
 *   This creates pthread to run an example with ASYNC TASH excution type.
 *   Only three points can be modified
 *   1. priority
 *   2. stacksize
 *   3. register entry function of pthread (example)
 */

extern pthread_addr_t esp32_demo_entry(pthread_addr_t arg);

static int esp32_wifi_os_cb(int argc, char **args)
{
	pthread_t opus_thread;

	pthread_attr_t attr;
	struct sched_param sparam;
	int status;

	/* Initialize the attribute variable */
	status = pthread_attr_init(&attr);
	if (status != 0) {
		printf("opus_thread : pthread_attr_init failed, status=%d\n", status);
	}

	/* 1. set a priority */
	sparam.sched_priority = ESP32_TASH_PRI;
	status = pthread_attr_setschedparam(&attr, &sparam);
	if (status != OK) {
		printf("opus_thread : pthread_attr_setschedparam failed, status=%d\n", status);
	}

	/* 2. set a stacksize */
	status = pthread_attr_setstacksize(&attr, ESP32_TASH_STAKSIZE);
	if (status != OK) {
		printf("opus_thread : pthread_attr_setstacksize failed, status=%d\n", status);
	}
	//pthread_attr_set
	//schedpolicy(&attr, SCHED_RR);
	/* 3. create pthread with entry function */

	status = pthread_create(&opus_thread, &attr, esp32_demo_entry, NULL);
	if (status != 0) {
		printf("opus_thread: pthread_create failed, status=%d\n", status);
	}

	/* Wait for the threads to stop */
	pthread_join(opus_thread, NULL);
	printf("esp32_demo_thread is finished\n");

	return 0;

}

#if defined(CONFIG_ADC)
#define CONVERT_PERIODICALLY		1
static int esp32_adc_os_cb(int argc, char **args)
{
	int ret;
	struct adc_msg_s samples[ADC_MAX_SAMPLES];
	ssize_t nbytes;

	int fd_adc = open("/dev/adc0", O_RDONLY);
	if (fd_adc < 0) {
		printf("%s: open failed: %d\n", __func__, errno);
		return -errno;
	}

	int i = 0;
#if defined(CONVERT_PERIODICALLY) && (0 < CONVERT_PERIODICALLY)
	ret = ioctl(fd_adc, ANIOC_TRIGGER, 0);
	if (ret < 0) {
		printf("%s: ioctl failed: %d\n", __func__, errno);
		close(fd_adc);
		return ret;
	}
	while (i++ < TEST_TIME_SEC) {
		nbytes = read(fd_adc, samples, sizeof(samples));
		if (nbytes < 0) {
			if (errno != EINTR) {
				printf("%s: read failed: %d\n", __func__, errno);
				close(fd_adc);
				return -errno;
			}
		} else if (nbytes == 0) {
			printf("%s: No data read, Ignoring\n", __func__);
		} else {
			int nsamples = nbytes / sizeof(struct adc_msg_s);
			if (nsamples * sizeof(struct adc_msg_s) != nbytes) {
				printf("%s: read size=%ld is not a multiple of sample size=%d, Ignoring\n", __func__, (long)nbytes, sizeof(struct adc_msg_s));
			} else {
				printf("Sample:\n");
				int i;
				for (i = 0; i < nsamples; i++) {
					printf("%d: channel: %d, value: %d\n", i + 1, samples[i].am_channel, samples[i].am_data);
				}
				printf("\n\n");
			}
		}

		sleep(1);
	}
#else
	while (i++ < TEST_TIME_SEC) {
		ret = ioctl(fd_adc, ANIOC_TRIGGER, 0);
		if (ret < 0) {
			printf("%s: ioctl failed: %d\n", __func__, errno);
			close(fd_adc);
			return ret;
		}

		nbytes = read(fd_adc, samples, sizeof(struct adc_msg_s));	//Read one
		if (nbytes < 0) {
			if (errno != EINTR) {
				printf("%s: read failed: %d\n", __func__, errno);
				close(fd_adc);
				return -errno;
			}
		} else if (nbytes == 0) {
			printf("%s: No data read, Ignoring\n", __func__);
		} else {
			int nsamples = nbytes / sizeof(struct adc_msg_s);
			if (nsamples * sizeof(struct adc_msg_s) != nbytes) {
				printf("%s: read size=%ld is not a multiple of sample size=%d, Ignoring\n", __func__, (long)nbytes, sizeof(struct adc_msg_s));
			} else {
				printf("Sample:\n");
				int i;
				for (i = 0; i < nsamples; i++) {
					printf("%d: channel: %d, value: %d\n", i + 1, samples[i].am_channel, samples[i].am_data);
				}
				printf("\n\n");
			}
		}

		//usleep(3000000);
		sleep(1);
	}
#endif
	close(fd_adc);
	printf("ADC test ends!\n\n\n");
	return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * opus_thread_main
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int esp32_tash_main(int argc, char **args)
{
	tash_cmd_install("esp32_demo", esp32_wifi_os_cb, TASH_EXECMD_ASYNC);
#if defined(CONFIG_ADC)
	tash_cmd_install("esp32_adc_demo", esp32_adc_os_cb, TASH_EXECMD_ASYNC);
#endif

	return 0;
}
#endif
