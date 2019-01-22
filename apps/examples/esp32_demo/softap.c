/* Scan Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
    This example shows how to use the All Channel Scan or Fast Scan to connect
    to a Wi-Fi network.

    In the Fast Scan mode, the scan will stop as soon as the first network matching
    the SSID is found. In this mode, an application can set threshold for the
    authentication mode and the Signal strength. Networks that do not meet the
    threshold requirements will be ignored.

    In the All Channel Scan mode, the scan will end only after all the channels
    are scanned, and connection will start with the best network. The networks
    can be sorted based on Authentication Mode or Signal Strength. The priority
    for the Authentication mode is:  WPA2 > WPA > WEP > Open
*/
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


#define CONFIG_ESP_WIFI_SSID "ESP32"
#define CONFIG_ESP_WIFI_PASSWORD "12345678"
#define CONFIG_MAX_STA_CONN 4

static const char *TAG = "AP";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
 switch(event->event_id) {
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
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
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
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

    ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
   //keep event hander
    while(1); 
}

void wifi_softap_entry()
{
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();
}
