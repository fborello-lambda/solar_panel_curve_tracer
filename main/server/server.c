#include "server.h"
#include <math.h>
#include <esp_app_desc.h>
#include <esp_ota_ops.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern bool measurement_is_running(void);
extern bool measurement_request(bool);

static const char *TAG = "server";

// Web UI files are gzip-compressed at build time and embedded directly in
// the firmware image (see main/CMakeLists.txt), so there is no filesystem
// to serve them from any more. Each embedded blob is exposed by the
// linker-provided _binary_<name>_start/_end symbols below.
extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[] asm("_binary_index_html_gz_end");
extern const uint8_t script_js_gz_start[] asm("_binary_script_js_gz_start");
extern const uint8_t script_js_gz_end[] asm("_binary_script_js_gz_end");
extern const uint8_t chart_umd_min_js_gz_start[] asm("_binary_chart_umd_min_js_gz_start");
extern const uint8_t chart_umd_min_js_gz_end[] asm("_binary_chart_umd_min_js_gz_end");
extern const uint8_t ota_html_gz_start[] asm("_binary_ota_html_gz_start");
extern const uint8_t ota_html_gz_end[] asm("_binary_ota_html_gz_end");

// The user guide PDF is not embedded (see main/CMakeLists.txt): it barely
// gzips and its size threatened the secure-boot variant's flash budget.
// /guide instead redirects to the hosted copy in the repo.
#define GUIDE_REDIRECT_URL "https://github.com/fborello-lambda/solar_panel_curve_tracer/blob/main/docs/quick_guide.pdf"

typedef struct
{
    const char *uri;
    const uint8_t *start;
    const uint8_t *end;
    const char *content_type;
} static_route_t;

static const static_route_t s_static_routes[] = {
    {"/", index_html_gz_start, index_html_gz_end, "text/html; charset=utf-8"},
    {"/script.js", script_js_gz_start, script_js_gz_end, "application/javascript"},
    {"/chart.js", chart_umd_min_js_gz_start, chart_umd_min_js_gz_end, "application/javascript"},
    {"/ota", ota_html_gz_start, ota_html_gz_end, "text/html; charset=utf-8"},
};
#define STATIC_ROUTE_COUNT (sizeof(s_static_routes) / sizeof(s_static_routes[0]))

static esp_err_t static_get_handler(httpd_req_t *req)
{
    const static_route_t *route = (const static_route_t *)req->user_ctx;

    const esp_app_desc_t *desc = esp_app_get_description();
    char etag[40];
    snprintf(etag, sizeof(etag), "\"%s\"", desc->version);

    char if_none_match[40] = {0};
    if (httpd_req_get_hdr_value_str(req, "If-None-Match", if_none_match, sizeof(if_none_match)) == ESP_OK &&
        strcmp(if_none_match, etag) == 0)
    {
        httpd_resp_set_status(req, "304 Not Modified");
        httpd_resp_set_hdr(req, "ETag", etag);
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
        return httpd_resp_send(req, NULL, 0);
    }

    httpd_resp_set_type(req, route->content_type);
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "ETag", etag);
    return httpd_resp_send(req, (const char *)route->start, route->end - route->start);
}

static esp_err_t guide_get_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", GUIDE_REDIRECT_URL);
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t data_get_handler(httpd_req_t *req)
{
    // Ensure data responses are not cached by clients/proxies
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    // POST fetch: ?have=<number>
    int client_have = 0;
    if (httpd_req_get_url_query_len(req) > 0)
    {
        char qbuf[64];
        if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK)
        {
            char val[16];
            if (httpd_query_key_value(qbuf, "have", val, sizeof(val)) == ESP_OK)
            {
                client_have = atoi(val);
            }
        }
    }

    // Snapshot state of the system
    float x[DB_MAX_SAMPLES];
    float y[DB_MAX_SAMPLES];
    size_t count;
    bool success = db_snapshot(x, y, &count, DB_MAX_SAMPLES);
    if (!success)
    {
        return httpd_resp_send(req, "{\"error\":\"Server error. Please try again later.\"}", HTTPD_RESP_USE_STRLEN);
    }

    // If client already has all points, return small JSON with just the count
    // JSON: {count:...}
    // This allows client to stop polling if there is no new data.
    //
    // Additionally, if count is zero, which means no data yet, it also returns JSON with the count=0.
    // The client will then know there is no data yet, and keeps fetching periodically.
    if (client_have >= count)
    {
        char small[64];
        int len = snprintf(small, sizeof(small), "{\"count\":%d}", count);
        return httpd_resp_send(req, small, len);
    }

    // Otherwise return JSON array of points: [{x:...,y:...}, ...]
    char buf[1024];
    ssize_t len = build_x_y_samples_json(buf, sizeof(buf), x, y, DB_MAX_SAMPLES, count);

    return httpd_resp_send(req, buf, len);
}

static esp_err_t start_measurement_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    ESP_LOGI(TAG, "Start measurement requested");

    if (measurement_is_running())
    {
        measurement_request(false);
        return httpd_resp_send(req, "{\"running\":false}", HTTPD_RESP_USE_STRLEN);
    }
    // else is not running, start it
    if (!measurement_request(true))
    {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_send(req, "{\"error\":\"Failed to start measurement\"}", HTTPD_RESP_USE_STRLEN);
    }

    return httpd_resp_send(req, "{\"running\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    if (req->content_len == 0)
    {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"Empty body\"}", HTTPD_RESP_USE_STRLEN);
    }

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition)
    {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"No OTA partition found\"}", HTTPD_RESP_USE_STRLEN);
    }

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"OTA begin failed\"}", HTTPD_RESP_USE_STRLEN);
    }

    char buf[1024];
    int remaining = (int)req->content_len;
    bool write_ok = true;

    while (remaining > 0)
    {
        int to_recv = (remaining < (int)sizeof(buf)) ? remaining : (int)sizeof(buf);
        int recv_len = httpd_req_recv(req, buf, to_recv);
        if (recv_len < 0)
        {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
                continue;
            write_ok = false;
            break;
        }
        err = esp_ota_write(ota_handle, buf, recv_len);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            write_ok = false;
            break;
        }
        remaining -= recv_len;
    }

    if (!write_ok)
    {
        esp_ota_abort(ota_handle);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"OTA write failed\"}", HTTPD_RESP_USE_STRLEN);
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"OTA validation failed\"}", HTTPD_RESP_USE_STRLEN);
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"OTA set boot failed\"}", HTTPD_RESP_USE_STRLEN);
    }

    ESP_LOGI(TAG, "OTA update complete, rebooting");
    httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

static esp_err_t version_get_handler(httpd_req_t *req)
{
    const esp_app_desc_t *desc = esp_app_get_description();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    char out[128];
    int n = snprintf(out, sizeof(out), "{\"version\":\"%s\",\"project\":\"%s\",\"date\":\"%s\"}",
                     desc->version, desc->project_name, desc->date);
    return httpd_resp_send(req, out, n);
}

static void extract_json_str(const char *buf, const char *key, char *out, size_t out_len)
{
    out[0] = '\0';
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);
    const char *pos = strstr(buf, search);
    if (!pos)
        return;
    pos = strchr(pos + strlen(search), ':');
    if (!pos)
        return;
    while (*pos == ':' || *pos == ' ')
        pos++;
    if (*pos != '"')
        return;
    pos++;
    size_t i = 0;
    while (*pos && *pos != '"' && i < out_len - 1)
        out[i++] = *pos++;
    out[i] = '\0';
}

static esp_err_t wifi_config_post_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    size_t to_read = req->content_len;
    if (to_read == 0 || to_read > 256)
    {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"Invalid body\"}", HTTPD_RESP_USE_STRLEN);
    }

    char buf[257] = {0};
    size_t received = 0;
    while (received < to_read)
    {
        int r = httpd_req_recv(req, buf + received, to_read - received);
        if (r <= 0)
        {
            if (r == HTTPD_SOCK_ERR_TIMEOUT)
                continue;
            httpd_resp_set_status(req, "400 Bad Request");
            return httpd_resp_send(req, "{\"error\":\"Read failed\"}", HTTPD_RESP_USE_STRLEN);
        }
        received += r;
    }
    buf[received] = '\0';

    char ssid[64] = {0};
    char password[64] = {0};
    extract_json_str(buf, "ssid", ssid, sizeof(ssid));
    extract_json_str(buf, "password", password, sizeof(password));

    if (ssid[0] == '\0')
    {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"Missing ssid\"}", HTTPD_RESP_USE_STRLEN);
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open("wifi_cfg", NVS_READWRITE, &h);
    if (err != ESP_OK)
    {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"NVS open failed\"}", HTTPD_RESP_USE_STRLEN);
    }
    nvs_set_str(h, "sta_ssid", ssid);
    nvs_set_str(h, "sta_pass", password);
    nvs_commit(h);
    nvs_close(h);

    ESP_LOGI(TAG, "WiFi STA credentials saved, SSID: %s", ssid);

    wifi_config_t sta_cfg = {0};
    strlcpy((char *)sta_cfg.sta.ssid, ssid, sizeof(sta_cfg.sta.ssid));
    strlcpy((char *)sta_cfg.sta.password, password, sizeof(sta_cfg.sta.password));
    esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
    esp_wifi_connect();

    char out[96];
    int n = snprintf(out, sizeof(out), "{\"ok\":true,\"ssid\":\"%s\"}", ssid);
    return httpd_resp_send(req, out, n);
}

esp_err_t server_init(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 12288;
    config.lru_purge_enable = true;
    config.max_open_sockets = 6;
    config.max_uri_handlers = 20;
    config.recv_wait_timeout = 30;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_start failed");
        return ESP_FAIL;
    }

    for (size_t i = 0; i < STATIC_ROUTE_COUNT; i++)
    {
        httpd_uri_t route = {
            .uri = s_static_routes[i].uri,
            .method = HTTP_GET,
            .handler = static_get_handler,
            .user_ctx = (void *)&s_static_routes[i],
        };
        httpd_register_uri_handler(server, &route);
    }

    httpd_uri_t guide = {.uri = "/guide", .method = HTTP_GET, .handler = guide_get_handler};
    httpd_uri_t data = {.uri = "/data", .method = HTTP_GET, .handler = data_get_handler};
    httpd_uri_t start_meas = {.uri = "/start-measurement", .method = HTTP_POST, .handler = start_measurement_handler};
    httpd_uri_t ota_post = {.uri = "/ota", .method = HTTP_POST, .handler = ota_post_handler};
    httpd_uri_t version = {.uri = "/version", .method = HTTP_GET, .handler = version_get_handler};
    httpd_uri_t wifi_cfg = {.uri = "/wifi-config", .method = HTTP_POST, .handler = wifi_config_post_handler};

    httpd_register_uri_handler(server, &guide);
    httpd_register_uri_handler(server, &data);
    httpd_register_uri_handler(server, &start_meas);
    httpd_register_uri_handler(server, &ota_post);
    httpd_register_uri_handler(server, &version);
    httpd_register_uri_handler(server, &wifi_cfg);
    ESP_LOGI(TAG, "HTTP server started");
    return ESP_OK;
}
