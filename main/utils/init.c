#include "init.h"
#include <string.h>

#define DEFAULT_SSID "ESP32_PLOT"
#define DEFAULT_PASSWORD ""
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN

static const char *TAG = "init";

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = DEFAULT_SSID,
            .ssid_len = 0,
            .channel = 1,
            .password = DEFAULT_PASSWORD,
            .max_connection = 4,
            .authmode = DEFAULT_AUTHMODE},
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "SoftAP started SSID:%s", ap_config.ap.ssid);
}

const char *wifi_softap_ssid(void)
{
    return DEFAULT_SSID;
}

const char *wifi_softap_password(void)
{
    return DEFAULT_PASSWORD;
}

bool wifi_softap_is_open(void)
{
    return DEFAULT_AUTHMODE == WIFI_AUTH_OPEN || strlen(DEFAULT_PASSWORD) == 0;
}

void system_init_all(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(spiffs_init());
    wifi_init_softap();
    ESP_ERROR_CHECK(server_init());

    db_init();
}
