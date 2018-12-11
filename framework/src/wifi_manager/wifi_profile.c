/****************************************************************************
 *
 * Copyright 2018 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <dirent.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <wifi_manager/wifi_manager.h>
#ifdef CONFIG_WIFI_PROFILE_SECURESTORAGE
#include <mbedtls/see_api.h>
#endif
#include "wifi_common.h"
#include "wifi_profile.h"

//#define WIFI_PROFILE_USE_ETC


#define WIFI_PROFILE_PATH "/mnt/"
#define WIFI_PROFILE_FILENAME "wifi.conf"
#define WIFI_PROFILE_FILENAME_INTERNAL "wifi_connected.conf"

#define DELIMITER "\t"
#define DELI_LEN 1

#define WIFI_PROFILE_BUFSIZE 128

#ifdef CONFIG_WIFI_PROFILE_SECURESTORAGE
#define WIFI_PROFILE_SS_INDEX 1
#define WIFI_PROFILE_SS_INDEX_INTERNAL 2
#endif

#define ENCODE_STRING(buf, size, data, pos)					\
	do {													\
		snprintf(buf + pos, size, "%s" DELIMITER, data);	\
		pos += (strlen(data) + DELI_LEN);					\
	} while (0)

#define ENCODE_INTEGER(buf, size, data, pos)				\
	do {													\
		char conv[12];										\
		snprintf(buf + pos, size, "%d" DELIMITER, data);	\
		sprintf(conv, "%d", data);							\
		pos += (strlen(conv) + DELI_LEN);					\
	} while (0)

#define DECODE_STRING(buf, data, pos, len)			\
	do {											\
		strncpy(data, buf + pos, len);				\
		data[len] = '\0';				\
		pos += ((int)(len) + DELI_LEN);	            \
	} while (0)

#define DECODE_INTEGER(buf, data, pos)			\
	do {										\
		char conv[12];							\
		int tmp;								\
		sscanf(buf + pos, "%d", &tmp);			\
		sprintf(conv, "%d", tmp);				\
		data = tmp;								\
		pos += (strlen(conv) + DELI_LEN);		\
	} while (0)

/*
 * Internal Functions
 */

static int _wifi_profile_serialize(char *buf, uint32_t buf_size, wifi_manager_ap_config_s *config)
{
	memset(buf, 0, buf_size);
	int pos = 0;
	ENCODE_INTEGER(buf, buf_size, config->ssid_length, pos);
	ENCODE_STRING(buf, buf_size, config->ssid, pos);
	int auth_type = (int)config->ap_auth_type;
	ENCODE_INTEGER(buf, buf_size, auth_type, pos);

	if (config->ap_auth_type == WIFI_MANAGER_AUTH_OPEN) {
		return strlen(buf) + 1;
	}

	ENCODE_INTEGER(buf, buf_size, config->passphrase_length, pos);
	ENCODE_STRING(buf, buf_size, config->passphrase, pos);

	int crypto_type = (int)config->ap_crypto_type;
	ENCODE_INTEGER(buf, buf_size, crypto_type, pos);

	return strlen(buf) + 1;
}

static void _wifi_profile_deserialize(wifi_manager_ap_config_s *config, char *buf)
{
	int pos = 0;
	DECODE_INTEGER(buf, config->ssid_length, pos);
	DECODE_STRING(buf, config->ssid, pos, config->ssid_length);

	int auth_type = 0;
	DECODE_INTEGER(buf, auth_type, pos);
	config->ap_auth_type = (wifi_manager_ap_auth_type_e)auth_type;
	if (config->ap_auth_type == WIFI_MANAGER_AUTH_OPEN) {
		return;
	}
	
	DECODE_INTEGER(buf, config->passphrase_length, pos);
	DECODE_STRING(buf, config->passphrase, pos, config->passphrase_length);

	int crypto_type = 0;
	DECODE_INTEGER(buf, crypto_type, pos);
	config->ap_crypto_type = (wifi_manager_ap_crypto_type_e)crypto_type;
}

#ifndef CONFIG_WIFI_PROFILE_SECURESTORAGE
static int _wifi_profile_store_file(char *buf, unsigned int buf_size, int internal)
{
	FILE *fp;
	if (internal > 0) {
		fp = fopen(WIFI_PROFILE_PATH WIFI_PROFILE_FILENAME_INTERNAL, "w+");
	} else {
		fp = fopen(WIFI_PROFILE_PATH WIFI_PROFILE_FILENAME, "w+");
	}

	if (!fp) {
		ndbg("file open error(%d)\n", errno);
		return -1;
	}

	int ret = fwrite(buf, 1, buf_size, fp);
	if (ret < 0) {
		ndbg("file write error(%d)\n", errno);
		fclose(fp);
		return -1;
	}
	fclose(fp);
	return 0;
}


static int _wifi_profile_read_file(char *buf, unsigned int buf_size, int internal)
{
	FILE *fp;
	if (internal > 0) {
		fp = fopen(WIFI_PROFILE_PATH WIFI_PROFILE_FILENAME_INTERNAL, "r");
	} else {
		fp = fopen(WIFI_PROFILE_PATH WIFI_PROFILE_FILENAME, "r");
	}
	if (!fp) {
		ndbg("file open error(%d)\n", errno);
		return -1;
	}

	int ret = fread(buf, 1, buf_size, fp);
	if (ret < 0) {
		ndbg("fread fail\n");
		fclose(fp);
		return -1;
	} else if (ret > 107) {
		ndbg("file is corrupted\n");
		fclose(fp);
		return -1;
	}
	fclose(fp);
	return 0;
}
#endif

/*
 * Public Functions
 */

wifi_utils_result_e wifi_profile_init(void)
{
#ifndef CONFIG_WIFI_PROFILE_SECURESTORAGE
#ifdef WIFI_PROFILE_USE_ETC
	DIR *dir = opendir(WIFI_PROFILE_PATH);
	if (!dir) {
		ndbg("error reason (%d)\n", errno);
		if (errno == ENOENT || errno == ENOTDIR) {
			ret = mkdir(WIFI_PROFILE_PATH, 0777);
			if (ret < 0) {
				return WIFI_UTILS_FILE_ERROR;
			}
		} else {
			return WIFI_UTILS_FILE_ERROR;
		}
	}
	closedir(dir);
#endif
#endif
	return WIFI_UTILS_SUCCESS;
}

wifi_utils_result_e wifi_profile_reset(int internal)
{
	int ret = -1;
#ifdef CONFIG_WIFI_PROFILE_SECURESTORAGE
	char buf = '\0';
	if (internal > 0) {
		ret = see_write_secure_storage((unsigned char *)&buf, (unsigned int)1, WIFI_PROFILE_SS_INDEX_INTERNAL);
	} else {
		ret = see_write_secure_storage((unsigned char *)&buf, (unsigned int)1, WIFI_PROFILE_SS_INDEX);
	}
	if (ret != SEE_OK) {
		ndbg("Write SS fail(%d)\n", ret);
		return WIFI_UTILS_FILE_ERROR;
	}
#else
	if (internal > 0) {
		ret = unlink(WIFI_PROFILE_PATH WIFI_PROFILE_FILENAME_INTERNAL);
	} else {
		ret = unlink(WIFI_PROFILE_PATH WIFI_PROFILE_FILENAME);
	}
	if (ret < 0) {
		ndbg("Delete Wi-Fi profile fail(%d)\n", errno);
		return WIFI_UTILS_FILE_ERROR;
	}
#endif
	return WIFI_UTILS_SUCCESS;
}

wifi_utils_result_e wifi_profile_write(wifi_manager_ap_config_s *config, int internal)
{
	char buf[WIFI_PROFILE_BUFSIZE];
	int ret = 0;
	int len = 0;
	len = _wifi_profile_serialize(buf, WIFI_PROFILE_BUFSIZE	, config);
	if (len < 0) {
		return WIFI_UTILS_FAIL;
	}
	nvdbg("store data to file: buffer len(%d)\n", len);
#ifdef CONFIG_WIFI_PROFILE_SECURESTORAGE
	if (internal > 0) {
		ret = see_write_secure_storage((unsigned char *)buf, (unsigned int)len, WIFI_PROFILE_SS_INDEX_INTERNAL);
	} else {
		ret = see_write_secure_storage((unsigned char *)buf, (unsigned int)len, WIFI_PROFILE_SS_INDEX);
	}
	if (ret != SEE_OK) {
		ndbg("Write SS fail(%d)\n", ret);
		return WIFI_UTILS_FILE_ERROR;
	}
#else
	ret = _wifi_profile_store_file(buf, len, internal);
	if (ret < 0) {
		return WIFI_UTILS_FILE_ERROR;
	}
#endif

	return WIFI_UTILS_SUCCESS;
}


wifi_utils_result_e wifi_profile_read(wifi_manager_ap_config_s *config, int internal)
{
	char buf[WIFI_PROFILE_BUFSIZE] = {0,};
	int ret = -1;
#ifdef CONFIG_WIFI_PROFILE_SECURESTORAGE
	/////////////////////////////////////////////////////////////////////////////////////////
	//
	// max size:
	// ssid(31) + tab(1) + ssid_length(2) + tab(1) + ap_type(1) + tab(1) +
	// passphrase(63) + tab(1) + passphrase length(2) + tab(1) + crypto type(1) == 105
	// file size of wifi.conf should not exceed 107(+EOF)
	// ssid_length and passphrase length are stored as character.
	//
	/////////////////////////////////////////////////////////////////////////////////////////
	unsigned int readlen = WIFI_PROFILE_BUFSIZE;
	if (internal > 0) {
		ret = see_read_secure_storage((unsigned char *)buf, &readlen, WIFI_PROFILE_SS_INDEX_INTERNAL);
	} else {
		ret = see_read_secure_storage((unsigned char *)buf, &readlen, WIFI_PROFILE_SS_INDEX);
	}
	if (ret != SEE_OK) {
		return WIFI_UTILS_FILE_ERROR;
	}
#else
	ret = _wifi_profile_read_file(buf, WIFI_PROFILE_BUFSIZE, internal);
	if (ret < 0) {
		return WIFI_UTILS_FILE_ERROR;
	}
#endif
	nvdbg("readed data len(%d)\n", readlen);

	_wifi_profile_deserialize(config, buf);

	return WIFI_UTILS_SUCCESS;
}
