#include "init.h"
#include <string.h>
#include <nvs.h>
#include <esp_timer.h>

#define AP_SSID "ESP32_PLOT"
#define AP_PASSWORD ""
#define AP_AUTHMODE WIFI_AUTH_OPEN

// STA reconnect backoff: give up after this many consecutive
// WIFI_EVENT_STA_DISCONNECTED without a GOT_IP in between. The counter is
// reset on GOT_IP and whenever new credentials are applied.
#define STA_MAX_RETRIES 5
#define STA_RETRY_BASE_MS 500

static const char *TAG = "init";

static int s_sta_retry_count = 0;
static esp_timer_handle_t s_sta_retry_timer = NULL;

static void sta_retry_timer_cb(void *arg)
{
    esp_wifi_connect();
}

static void sta_reset_retry(void)
{
    s_sta_retry_count = 0;
    if (s_sta_retry_timer)
        esp_timer_stop(s_sta_retry_timer); // ok if not currently running
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_sta_retry_count >= STA_MAX_RETRIES)
        {
            ESP_LOGE(TAG, "STA disconnected, giving up after %d attempts", STA_MAX_RETRIES);
            return;
        }

        uint32_t delay_ms = STA_RETRY_BASE_MS << s_sta_retry_count; // 500,1000,2000,4000,8000
        s_sta_retry_count++;
        ESP_LOGW(TAG, "STA disconnected, retry %d/%d in %lu ms",
                 s_sta_retry_count, STA_MAX_RETRIES, (unsigned long)delay_ms);

        if (s_sta_retry_timer == NULL)
        {
            const esp_timer_create_args_t targs = {
                .callback = sta_retry_timer_cb,
                .name = "sta_retry",
            };
            if (esp_timer_create(&targs, &s_sta_retry_timer) != ESP_OK)
            {
                ESP_LOGW(TAG, "sta retry: failed to create backoff timer, retrying immediately");
                esp_wifi_connect();
                return;
            }
        }
        esp_timer_stop(s_sta_retry_timer); // ignore ESP_ERR_INVALID_STATE if not running
        esp_timer_start_once(s_sta_retry_timer, (uint64_t)delay_ms * 1000);
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "STA got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        sta_reset_retry();
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

esp_err_t wifi_apply_sta_credentials(const char *ssid, const char *password)
{
    sta_reset_retry();

    wifi_mode_t mode;
    esp_err_t err = esp_wifi_get_mode(&mode);
    if (err != ESP_OK)
        return err;

    if (mode == WIFI_MODE_AP)
    {
        err = esp_wifi_set_mode(WIFI_MODE_APSTA);
        if (err != ESP_OK)
            return err;
    }

    wifi_config_t sta_config = {0};
    strlcpy((char *)sta_config.sta.ssid, ssid, sizeof(sta_config.sta.ssid));
    strlcpy((char *)sta_config.sta.password, password, sizeof(sta_config.sta.password));
    err = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    if (err != ESP_OK)
        return err;

    err = esp_wifi_connect();
    // Already connected/connecting is not a failure worth reporting.
    if (err == ESP_ERR_WIFI_CONN)
        return ESP_OK;
    return err;
}

esp_err_t wifi_clear_sta_credentials(void)
{
    sta_reset_retry();

    esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_STARTED && err != ESP_ERR_WIFI_NOT_CONNECT)
        return err;

    wifi_mode_t mode;
    err = esp_wifi_get_mode(&mode);
    if (err != ESP_OK)
        return err;

    if (mode == WIFI_MODE_APSTA)
        return esp_wifi_set_mode(WIFI_MODE_AP);

    return ESP_OK;
}

void system_init_all(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();
    ESP_ERROR_CHECK(server_init());

    db_init();
}
