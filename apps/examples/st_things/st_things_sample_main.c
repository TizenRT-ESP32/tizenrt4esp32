/* ****************************************************************
*
* Copyright 2017 Samsung Electronics All Rights Reserved.
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

#include <tinyara/config.h>

#include <stdio.h>
#include "st_things_sample.h"

//#define SETUP_WIFI

#ifdef SETUP_WIFI

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_wifi_internal.h"
#include <string.h>

/*======config for wifi softap mode=====*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_MAX_STA_CONN       CONFIG_MAX_STA_CONN


#define CONFIG_ESP_WIFI_SSID "SRCN-2.4G"
#define CONFIG_ESP_WIFI_PASSWORD "wc6rn9v3"
#define CONFIG_MAX_STA_CONN 4

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
 switch(event->event_id) {
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI("AP", "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI("AP", "station:"MACSTR"leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        break;
    default:
        break;
    }
    return ESP_OK;
}

/* Initialize Wi-Fi as sotfap mode */
static void wifi_init_softap(void)
{
    esp_err_t ret;
    tcpip_adapter_init();
    ret = esp_event_loop_init(event_handler, NULL);
    if(ret) {
        ets_printf("esp_event_loop_init failed, %d\n", ret);
        return;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if(ret) {
        ets_printf("esp_wifi_init failed, %d\n", ret);
        return;
    }
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if(ret) {
        ets_printf("esp_wifi_set_mode failed, %d\n", ret);
        return;
    }

    ret = esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
    if(ret) {
        ets_printf("esp_wifi_set_config failed, %d\n", ret);
        return;
    }
     ret = esp_wifi_start();
    if(ret) {
        return;
    }

    ESP_LOGI("AP", "wifi_init_softap finished.SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
   //keep event hander
    //while(1);
}
#endif






#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int st_things_sample_main(int argc, char *argv[])
#endif
{
	printf("st_things_sample!!\n");

#ifdef SETUP_WIFI
	wifi_init_softap();
#endif

	printf("ess_process!!\n");

	ess_process();

	return 0;
}
