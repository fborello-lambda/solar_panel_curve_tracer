#include "init.h"
#include <string.h>
#include <nvs.h>

#define AP_SSID "ESP32_PLOT"
#define AP_PASSWORD ""
#define AP_AUTHMODE WIFI_AUTH_OPEN

static const char *TAG = "init";

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "STA disconnected, retrying...");
        esp_wifi_connect();
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "STA got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void wifi_load_sta_credentials(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    ssid[0] = '\0';
    pass[0] = '\0';
    nvs_handle_t h;
    if (nvs_open("wifi_cfg", NVS_READONLY, &h) != ESP_OK)
        return;
    nvs_get_str(h, "sta_ssid", ssid, &ssid_len);
    nvs_get_str(h, "sta_pass", pass, &pass_len);
    nvs_close(h);
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    char sta_ssid[64] = {0};
    char sta_pass[64] = {0};
    wifi_load_sta_credentials(sta_ssid, sizeof(sta_ssid), sta_pass, sizeof(sta_pass));

    bool has_sta = sta_ssid[0] != '\0';
    ESP_ERROR_CHECK(esp_wifi_set_mode(has_sta ? WIFI_MODE_APSTA : WIFI_MODE_AP));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = 0,
            .channel = 1,
            .password = AP_PASSWORD,
            .max_connection = 4,
            .authmode = AP_AUTHMODE,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    if (has_sta)
    {
        wifi_config_t sta_config = {0};
        strlcpy((char *)sta_config.sta.ssid, sta_ssid, sizeof(sta_config.sta.ssid));
        strlcpy((char *)sta_config.sta.password, sta_pass, sizeof(sta_config.sta.password));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
        ESP_LOGI(TAG, "STA credentials loaded for SSID: %s", sta_ssid);
    }

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "AP started SSID:%s%s", ap_config.ap.ssid, has_sta ? " + STA enabled" : "");
}

const char *wifi_softap_ssid(void)
{
    return AP_SSID;
}

const char *wifi_softap_password(void)
{
    return AP_PASSWORD;
}

bool wifi_softap_is_open(void)
{
    return AP_AUTHMODE == WIFI_AUTH_OPEN || strlen(AP_PASSWORD) == 0;
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
