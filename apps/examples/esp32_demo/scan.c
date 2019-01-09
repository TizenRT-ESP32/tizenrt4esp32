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

/*
CONFIG_WIFI_SSID="myssid"
CONFIG_WIFI_PASSWORD="mypassword"
CONFIG_WIFI_FAST_SCAN=y
CONFIG_WIFI_ALL_CHANNEL_SCAN=
CONFIG_WIFI_CONNECT_AP_BY_SIGNAL=y
CONFIG_WIFI_CONNECT_AP_BY_SECURITY=
CONFIG_FAST_SCAN_THRESHOLD=y
CONFIG_FAST_SCAN_MINIMUM_SIGNAL=-127
CONFIG_EXAMPLE_OPEN=y
CONFIG_EXAMPLE_WEP=
CONFIG_EXAMPLE_WPA=
CONFIG_EXAMPLE_WPA2=
*/

/*Set the SSID and Password via "make menuconfig"*/
#define DEFAULT_SSID CONFIG_WIFI_SSID
#define DEFAULT_PWD CONFIG_WIFI_PASSWORD

#define DEFAULT_PS_MODE WIFI_PS_MIN_MODEM

#ifdef CONFIG_WIFI_ALL_CHANNEL_SCAN
#define DEFAULT_SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
#elif CONFIG_WIFI_FAST_SCAN
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#else
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#endif /*CONFIG_SCAN_METHOD*/

#ifdef CONFIG_WIFI_CONNECT_AP_BY_SIGNAL
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#elif CONFIG_WIFI_CONNECT_AP_BY_SECURITY
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SECURITY
#else
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#endif /*CONFIG_SORT_METHOD*/

#ifdef CONFIG_FAST_SCAN_THRESHOLD
#define DEFAULT_RSSI CONFIG_FAST_SCAN_MINIMUM_SIGNAL
#ifdef CONFIG_EXAMPLE_OPEN
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#elif CONFIG_EXAMPLE_WEP
#define DEFAULT_AUTHMODE WIFI_AUTH_WEP
#elif CONFIG_EXAMPLE_WPA
#define DEFAULT_AUTHMODE WIFI_AUTH_WPA_PSK
#elif CONFIG_EXAMPLE_WPA2
#define DEFAULT_AUTHMODE WIFI_AUTH_WPA2_PSK
#else
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#endif
#else
#define DEFAULT_RSSI -127
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#endif /*CONFIG_FAST_SCAN_THRESHOLD*/


static const char *TAG = "scan";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
            if(esp_wifi_connect()) {
                printf("esp_wifi_connect failed\n");
            }
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
            ESP_LOGI(TAG, "Got IP: %s\n",
                     ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
            if(esp_wifi_connect()) {
                printf("esp_wifi_connect failed\n");
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

/* Initialize Wi-Fi as sta and set scan method */
static void do_wifi_scan(void)
{
    esp_err_t ret;
    tcpip_adapter_init();
    //printf("esp_event_loop_init\n");
    ret = esp_event_loop_init(event_handler, NULL);
    if(ret) {
        ets_printf("esp_event_loop_init failed, %d\n", ret);
        return;
    }
   

   // printf("esp_wifi_init \n");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if(ret) {
        ets_printf("esp_wifi_init failed, %d\n", ret);
        return;
    }

#if 1
     esp_wifi_internal_set_log_level(WIFI_LOG_VERBOSE);

        esp_wifi_internal_set_log_mod(WIFI_LOG_MODULE_ALL, WIFI_LOG_MODULE_ALL | WIFI_LOG_SUBMODULE_INIT | WIFI_LOG_SUBMODULE_IOCTL | WIFI_LOG_SUBMODULE_CONN | WIFI_LOG_SUBMODULE_SCAN, true);

#endif
        
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = DEFAULT_SSID,
            .password = DEFAULT_PWD,
            .scan_method = DEFAULT_SCAN_METHOD,
            .sort_method = DEFAULT_SORT_METHOD,
            .threshold.rssi = DEFAULT_RSSI,
            .threshold.authmode = DEFAULT_AUTHMODE,
        },
    };

//    lldbg("esp_wifi_set_mode\n");
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if(ret) {
        ets_printf("esp_wifi_set_mode failed, %d\n", ret);
        return;
    }
    
  //  lldbg("esp_wifi_set_config\n");
    ret = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    if(ret) {
        ets_printf("esp_wifi_set_config failed, %d\n", ret);
        return;
    }
    //lldbg("esp_wifi_start\n");
    ret = esp_wifi_start();
    if(ret) { 
        return;
    }

//    esp_wifi_set_ps(DEFAULT_PS_MODE);
}

void wifi_station_entry()
{

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STATION");
    do_wifi_scan();
}
