/****************************************************************************
 *
 * Copyright 2017 Samsung Electronics All Rights Reserved.
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
#include <pthread.h>
#include <sys/time.h>
#include <errno.h>
#include <wifi_manager/wifi_manager.h>
#include "tc_common.h"

static pthread_mutex_t g_wifi_manager_test_mutex = PTHREAD_MUTEX_INITIALIZER;;
static pthread_cond_t g_wifi_manager_test_cond;

#define WIFITEST_SIGNAL			\
do {										\
	pthread_mutex_lock(&g_wifi_manager_test_mutex);			\
	pthread_cond_signal(&g_wifi_manager_test_cond);			\
	pthread_mutex_unlock(&g_wifi_manager_test_mutex);			\
} while (0)

#define WIFITEST_WAIT						\
do {										\
	pthread_mutex_lock(&g_wifi_manager_test_mutex);			\
	pthread_cond_wait(&g_wifi_manager_test_cond, &g_wifi_manager_test_mutex);	\
	pthread_mutex_unlock(&g_wifi_manager_test_mutex);			\
} while (0)
void wifi_sta_connected(wifi_manager_result_e result);		// in station mode, connected to ap
void wifi_sta_disconnected(wifi_manager_disconnect_e disconn);	// in station mode, disconnected from ap
void wifi_softap_sta_joined(void);	// in softap mode, a station joined
void wifi_softap_sta_left(void);		// in softap mode, a station left
void wifi_scan_ap_done(wifi_manager_scan_info_s **scan_info, wifi_manager_scan_result_e res); // called when scanning ap is done

static wifi_manager_cb_s wifi_callbacks = {
	wifi_sta_connected,
	wifi_sta_disconnected,
	NULL,	//wifi_softap_sta_joined,
	NULL,	//wifi_softap_sta_left,
	wifi_scan_ap_done,	// this callback function is called when scanning ap is done.
};

static wifi_manager_cb_s wifi_null_callbacks = {
	NULL,	//wifi_sta_connected,
	NULL,	//wifi_sta_disconnected,
	NULL,	//wifi_softap_sta_joined,
	NULL,	//wifi_softap_sta_left,
	NULL,	// in station mode, this callback function is called when scanning ap is done.
};

void wifi_sta_connected(wifi_manager_result_e result)
{
	printf("wifi_sta_connected: send signal!!!(%d) \n", result);
	WIFITEST_SIGNAL;
}

void wifi_sta_disconnected(wifi_manager_disconnect_e disconn)
{
	printf("wifi_sta_disconnected: send signal!!! \n");
	WIFITEST_SIGNAL;
}

void wifi_scan_ap_done(wifi_manager_scan_info_s **scan_info, wifi_manager_scan_result_e res)
{
	/* Make sure you copy the scan results onto a local data structure.
	 * It will be deleted soon eventually as you exit this function.
	 */
	if (res == WIFI_SCAN_FAIL) {
		printf("WiFi scan failed\n");
		WIFITEST_SIGNAL;
		return;
	}
	wifi_manager_scan_info_s *wifi_scan_iter = *scan_info;
	while (wifi_scan_iter != NULL) {
	printf("SSID: %-20s, BSSID: %-20s, RSSI: %d, CH: %d, Phy_type: %d\n", \
						wifi_scan_iter->ssid, wifi_scan_iter->bssid, wifi_scan_iter->rssi, \
						wifi_scan_iter->channel, wifi_scan_iter->phy_mode);
		wifi_scan_iter = wifi_scan_iter->next;
	}
	WIFITEST_SIGNAL;
}

static int wifi_test_signal_init(void)
{
	int res = pthread_mutex_init(&g_wifi_manager_test_mutex, NULL);
	if (res != 0) {
		printf("wifi test signal init fail(%d) (%d)\n", res, errno);
		return -1;
	}
	res = pthread_cond_init(&g_wifi_manager_test_cond, NULL);
	if (res != 0) {
		printf("wifi test condition init fail(%d) (%d)\n", res, errno);
		pthread_mutex_destroy(&g_wifi_manager_test_mutex);
		return -1;
	}
	return 0;
}

static void wifi_test_signal_deinit(void)
{
	pthread_mutex_destroy(&g_wifi_manager_test_mutex);
	pthread_cond_destroy(&g_wifi_manager_test_cond);
}

static void utc_wifi_manager_init_n(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;

	ret = wifi_manager_init(NULL);

	TC_ASSERT_EQ("wifi_manager_init", ret, WIFI_MANAGER_INVALID_ARGS);
	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_init_p(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;

	ret = wifi_manager_init(&wifi_callbacks);

	TC_ASSERT_EQ("wifi_manager_init", ret, WIFI_MANAGER_SUCCESS);
	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_set_mode_n(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;

	ret = wifi_manager_set_mode(SOFTAP_MODE, NULL);

	TC_ASSERT_EQ("wifi_manager_set_mode", ret, WIFI_MANAGER_INVALID_ARGS);

	wifi_manager_softap_config_s ap_config;
	strncpy(ap_config.ssid, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_SSID, \
					strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_SSID) + 1);
	ap_config.channel = CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_CHANNEL;
	strncpy(ap_config.passphrase, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_PASSPHRASE, \
					strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_PASSPHRASE) + 1);

	ret = wifi_manager_set_mode(WIFI_NONE, &ap_config);

	TC_ASSERT_EQ("wifi_manager_set_mode", ret, WIFI_MANAGER_INVALID_ARGS);
	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_set_mode_p(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;

	wifi_manager_softap_config_s ap_config;
	strncpy(ap_config.ssid, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_SSID, \
					strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_SSID) + 1);
	ap_config.channel = CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_CHANNEL;
	strncpy(ap_config.passphrase, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_PASSPHRASE, \
					strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_PASSPHRASE) + 1);
	printf("SoftAP config: %s(%d), %s(%d), %d\n", ap_config.ssid, strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_SSID), \
					ap_config.passphrase, strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_PASSPHRASE), ap_config.channel);

	ret = wifi_manager_set_mode(SOFTAP_MODE, &ap_config);

	TC_ASSERT_EQ("wifi_manager_set_mode", ret, WIFI_MANAGER_SUCCESS);

	ret = wifi_manager_set_mode(STA_MODE, NULL);

	TC_ASSERT_EQ("wifi_manager_set_mode", ret, WIFI_MANAGER_SUCCESS);

	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_get_mode_n(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;
	wifi_manager_info_s *info = NULL;

	ret = wifi_manager_get_info(info);

	TC_ASSERT_EQ("wifi_manager_get_info", ret, WIFI_MANAGER_INVALID_ARGS);
	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_get_mode_p(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;
	wifi_manager_info_s info;
	int ret_cmp = 0;
	int i;

	ret = wifi_manager_get_info(&info);

	TC_ASSERT_EQ("wifi_manager_get_info", ret, WIFI_MANAGER_SUCCESS);
	TC_ASSERT_EQ("wifi_manager_get_info", info.mode, STA_MODE);

	wifi_manager_softap_config_s ap_config;
	strncpy(ap_config.ssid, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_SSID, \
					strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_SSID) + 1);
	ap_config.channel = CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_CHANNEL;
	strncpy(ap_config.passphrase, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_PASSPHRASE, \
					strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_PASSPHRASE) + 1);
	ret = wifi_manager_set_mode(SOFTAP_MODE, &ap_config);

	TC_ASSERT_EQ("wifi_manager_set_mode", ret, WIFI_MANAGER_SUCCESS);

	ret = wifi_manager_get_info(&info);

	TC_ASSERT_EQ("wifi_manager_get_info", ret, WIFI_MANAGER_SUCCESS);
	TC_ASSERT_EQ("wifi_manager_get_info", info.mode, SOFTAP_MODE);
	printf("\nMAC Address: ");
	for (i = 0; i < 5; i++) {
		printf("%x:", info.mac_address[i]);
	}
	printf("%x\n", info.mac_address[5]);
	ret_cmp = strncmp(info.ssid, ap_config.ssid, strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_SSID));
	TC_ASSERT_EQ("wifi_manager_get_mode", ret_cmp, 0);

	TC_SUCCESS_RESULT();
}


static void utc_wifi_manager_connect_ap_n(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;

	/* Connect to AP
	 * This negative tc function will fail
	 * so that you do not need to change the ap configuration
	 */
	wifi_manager_ap_config_s config;
	config.ssid_length = strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SSID);
	config.passphrase_length = strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_PASSPHRASE);
	strncpy(config.ssid, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SSID, config.ssid_length + 1);
	strncpy(config.passphrase, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_PASSPHRASE, config.passphrase_length + 1);
	config.ap_auth_type = (wifi_manager_ap_auth_type_e)CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_AUTHENTICATION;
	config.ap_crypto_type = (wifi_manager_ap_crypto_type_e)CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_CRYPTO;

	/* current wifi mode is softap, then this try will fail */
	ret = wifi_manager_connect_ap(&config);

	TC_ASSERT_EQ("wifi_manager_connect_ap", ret, WIFI_MANAGER_FAIL);
	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_connect_ap_p(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;

	/* change to station mode */
	ret = wifi_manager_set_mode(STA_MODE, NULL);
	TC_ASSERT_EQ("wifi_manager_set_mode", ret, WIFI_MANAGER_SUCCESS);

	/* Connect to AP
	 * You need to change configuration of ap which you actually try to connect to.
	 * Otherwise, this function will fail.
	 */
	wifi_manager_ap_config_s config;
	config.ssid_length = strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SSID);
	config.passphrase_length = strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_PASSPHRASE);
	strncpy(config.ssid, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SSID, config.ssid_length + 1);
	strncpy(config.passphrase, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_PASSPHRASE, config.passphrase_length + 1);
	config.ap_auth_type = (wifi_manager_ap_auth_type_e)CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_AUTHENTICATION;
	config.ap_crypto_type = (wifi_manager_ap_crypto_type_e)CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_CRYPTO;
	printf("AP config: %s(%d), %s(%d), %d %d\n", config.ssid, config.ssid_length, config.passphrase, config.passphrase_length, config.ap_auth_type, config.ap_crypto_type);

	/* current wifi mode is station, then this try will succeed */
	ret = wifi_manager_connect_ap(&config);

	TC_ASSERT_EQ("wifi_manager_connect_ap", ret, WIFI_MANAGER_SUCCESS);
	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_disconnect_ap_p(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;

	ret = wifi_manager_disconnect_ap();

	TC_ASSERT_EQ("wifi_manager_disconnect_ap", ret, WIFI_MANAGER_SUCCESS);
	TC_SUCCESS_RESULT();
}

/* second disconnection trial will get failed */
static void utc_wifi_manager_disconnect_ap_n(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;

	ret = wifi_manager_disconnect_ap();

	TC_ASSERT_EQ("wifi_manager_disconnect_ap", ret, WIFI_MANAGER_FAIL);
	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_deinit_p(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;

	ret = wifi_manager_deinit();

	TC_ASSERT_EQ("wifi_manager_deinit", ret, WIFI_MANAGER_SUCCESS);
	TC_SUCCESS_RESULT();
}

/* Initialize wifi manager without callback for scan results
 * this leads to failure.
 */
static void utc_wifi_manager_deinit_n(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;

	ret = wifi_manager_deinit();

	TC_ASSERT_EQ("wifi_manager_deinit", ret, WIFI_MANAGER_FAIL);
	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_scan_ap_n(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;

	ret = wifi_manager_init(&wifi_null_callbacks);
	TC_ASSERT_EQ("wifi_manager_init", ret, WIFI_MANAGER_SUCCESS);

	ret = wifi_manager_scan_ap();

	TC_ASSERT_EQ("wifi_manager_scan_ap", ret, WIFI_MANAGER_FAIL);
	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_scan_ap_p(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;

	ret = wifi_manager_deinit();
	TC_ASSERT_EQ("wifi_manager_deinit", ret, WIFI_MANAGER_SUCCESS);

	ret = wifi_manager_init(&wifi_callbacks);
	TC_ASSERT_EQ("wifi_manager_init", ret, WIFI_MANAGER_SUCCESS);

	ret = wifi_manager_scan_ap();
	TC_ASSERT_EQ("wifi_manager_scan_ap", ret, WIFI_MANAGER_SUCCESS);

	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_save_config_n(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;
	ret = wifi_manager_save_config(NULL);

	TC_ASSERT_EQ("wifi_manager_save_config", ret, WIFI_MANAGER_INVALID_ARGS);
	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_save_config_p(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;
	
	wifi_manager_ap_config_s config;
	config.ssid_length = strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SSID);
	config.passphrase_length = strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_PASSPHRASE);
	strncpy(config.ssid, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SSID, config.ssid_length + 1);
	strncpy(config.passphrase, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_PASSPHRASE, config.passphrase_length + 1);
	config.ap_auth_type = (wifi_manager_ap_auth_type_e)CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_AUTHENTICATION;
	config.ap_crypto_type = (wifi_manager_ap_crypto_type_e)CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_CRYPTO;
	printf("AP config: %s(%d), %s(%d), %d %d\n", config.ssid, config.ssid_length, config.passphrase, config.passphrase_length, config.ap_auth_type, config.ap_crypto_type);

	ret = wifi_manager_save_config(&config);

	TC_ASSERT_EQ("wifi_manager_save_config", ret, WIFI_MANAGER_SUCCESS);
	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_get_config_p(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;
	int res = 0;
	wifi_manager_ap_config_s config;
	ret = wifi_manager_get_config(&config);
	if (ret == WIFI_MANAGER_SUCCESS) {
		printf("====================================\n");
		printf("SSID: %s\n", config.ssid);
		printf("SECURITY TYPE: %d\n", config.ap_auth_type);
		printf("====================================\n");
		res = strncmp(config.ssid, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SSID, strlen(config.ssid));
		res = strncmp(config.passphrase, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_PASSPHRASE, strlen(config.passphrase));
		if (res < 0) {
			ret = WIFI_MANAGER_FAIL;
		}
	}
	TC_ASSERT_EQ("wifi_manager_get_config", ret, WIFI_MANAGER_SUCCESS);
	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_get_config_n(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;
	int res = 0;
	wifi_manager_ap_config_s config;
	ret = wifi_manager_get_config(&config);
	if (ret == WIFI_MANAGER_SUCCESS) {
		printf("====================================\n");
		printf("SSID: %s\n", config.ssid);
		printf("SECURITY TYPE: %d\n", config.ap_auth_type);
		printf("====================================\n");
		res = strncmp(config.ssid, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SSID, strlen(config.ssid));
		res = strncmp(config.passphrase, CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_PASSPHRASE, strlen(config.passphrase));
		if (res < 0) {
			ret = WIFI_MANAGER_FAIL;
		}
	}
	TC_ASSERT_EQ("wifi_manager_get_config", ret, WIFI_MANAGER_FAIL);
	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_remove_config_n(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;
	ret = wifi_manager_remove_config();
	TC_ASSERT_EQ("wifi_manager_remove_config", ret, WIFI_MANAGER_FAIL);
	TC_SUCCESS_RESULT();
}

static void utc_wifi_manager_remove_config_p(void)
{
	wifi_manager_result_e ret = WIFI_MANAGER_FAIL;
	ret = wifi_manager_remove_config();
	TC_ASSERT_EQ("wifi_manager_remove_config", ret, WIFI_MANAGER_SUCCESS);
	TC_SUCCESS_RESULT();
}


int wifi_manager_utc(int argc, FAR char *argv[])
{
	if (tc_handler(TC_START, "WiFiManager UTC") == ERROR) {
		return ERROR;
	}

	utc_wifi_manager_init_n();
	utc_wifi_manager_init_p();

	utc_wifi_manager_set_mode_n();
	utc_wifi_manager_set_mode_p();

	utc_wifi_manager_get_mode_n();
	utc_wifi_manager_get_mode_p();	// set softap mode inside this function

	/* shoud define ap config first in utc_wifi_manager_connect_ap_n()
	 * and utc_wifi_manager_connect_ap_p() before trying these two tests
	 * Otherwise, you will get failed.
	 */
	 
	utc_wifi_manager_connect_ap_n();	// try to connect to ap in softap mode
	utc_wifi_manager_connect_ap_p();	// change to station mode and try to connect to ap

	WIFITEST_WAIT;

	sleep(5);
	
	utc_wifi_manager_save_config_n();   
	utc_wifi_manager_get_config_n();	
	utc_wifi_manager_remove_config_n(); 
	utc_wifi_manager_save_config_p();	// save correct wifi config
	utc_wifi_manager_get_config_p();    
	utc_wifi_manager_remove_config_p();

	utc_wifi_manager_disconnect_ap_p();

	WIFITEST_WAIT;

	utc_wifi_manager_disconnect_ap_n();	//  Should be run after positive tc, that is, the second disconnect gets failed.

	utc_wifi_manager_deinit_p();
	utc_wifi_manager_deinit_n(); // Should be run after positive tc, that is, the second deinit gets failed.

	utc_wifi_manager_scan_ap_n(); // Get failed becasue there is no callback hander for scan results
	utc_wifi_manager_scan_ap_p(); // Reinitialized wifi manager with the callback hander for scan results

	WIFITEST_WAIT;

	utc_wifi_manager_deinit_p(); // End of UTC

	(void)tc_handler(TC_END, "WiFiManager UTC");

	return 0;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int utc_wifi_manager_main(int argc, char *argv[])
#endif
{

	int ssid_len = strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SSID);
	int passphrase_len = strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_PASSPHRASE);
	int softap_ssid_len =	strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_SSID);
	int softap_passphrase_len = strlen(CONFIG_EXAMPLES_TESTCASE_WIFI_MANAGER_UTC_SOFTAP_PASSPHRASE);

	if ((ssid_len > 31) || (softap_ssid_len > 31) || (passphrase_len > 63) || (softap_passphrase_len > 63)) {
		printf("AP or SoftAP configuration fails: too long ssid or passphrase\n");
		printf("Make sure that length of SSID < 32 and length of passphrase < 64\n");
		return -1;
	}

	int res = wifi_test_signal_init();
	if (res < 0) {
		return -1;
	}

	wifi_manager_utc(argc, argv);

	wifi_test_signal_deinit();

	return 0;
}
