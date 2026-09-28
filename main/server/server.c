#include "server.h"
#include <stdlib.h>
#include <esp_app_desc.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <nvs.h>
#include <esp_wifi.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "measurement.h"
#include "../utils/init.h"
#include "../app/app_state.h"

static const char *TAG = "server";

// First byte of a valid ESP app image (see ESP_IMAGE_HEADER_MAGIC in
// esp_app_format.h); checked before esp_ota_begin so a bad upload is
// rejected before any flash erase happens.
#define ESP_APP_IMAGE_MAGIC_BYTE 0xE9

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
extern const uint8_t i18n_js_gz_start[] asm("_binary_i18n_js_gz_start");
extern const uint8_t i18n_js_gz_end[] asm("_binary_i18n_js_gz_end");
extern const uint8_t guide_html_gz_start[] asm("_binary_guide_html_gz_start");
extern const uint8_t guide_html_gz_end[] asm("_binary_guide_html_gz_end");

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
    {"/i18n.js", i18n_js_gz_start, i18n_js_gz_end, "application/javascript"},
    {"/guide", guide_html_gz_start, guide_html_gz_end, "text/html; charset=utf-8"},
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

static esp_err_t data_get_handler(httpd_req_t *req)
{
    // Ensure data responses are not cached by clients/proxies
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    // GET query: ?have=<number>
    long client_have = 0;
    if (httpd_req_get_url_query_len(req) > 0)
    {
        char qbuf[64];
        if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK)
        {
            char val[16];
            if (httpd_query_key_value(qbuf, "have", val, sizeof(val)) == ESP_OK)
            {
                char *endptr = NULL;
                long parsed = strtol(val, &endptr, 10);
                if (endptr != val)
                    client_have = parsed;
            }
        }
    }
    if (client_have < 0)
        client_have = 0;

    // Snapshot state of the system
    float x[DB_MAX_SAMPLES];
    float y[DB_MAX_SAMPLES];
    size_t count;
    bool success = db_snapshot(x, y, &count, DB_MAX_SAMPLES);
    if (!success)
    {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_send(req, "{\"error\":\"Server error. Please try again later.\"}", HTTPD_RESP_USE_STRLEN);
    }

    // If client already has all points, return small JSON with just the count
    // JSON: {count:...}
    // This allows client to stop polling if there is no new data.
    //
    // Additionally, if count is zero, which means no data yet, it also returns JSON with the count=0.
    // The client will then know there is no data yet, and keeps fetching periodically.
    if ((size_t)client_have >= count)
    {
        char small[64];
        int len = snprintf(small, sizeof(small), "{\"count\":%zu}", count);
        return httpd_resp_send(req, small, len);
    }

    // Otherwise return JSON array of points: [{x:...,y:...}, ...]
    char buf[1024];
    ssize_t len = build_x_y_samples_json(buf, sizeof(buf), x, y, DB_MAX_SAMPLES, count);

    return httpd_resp_send(req, buf, len);
}

static esp_err_t measurement_start_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    ESP_LOGI(TAG, "Measurement start requested");

    measurement_refuse_reason_t reason = MEASUREMENT_REFUSE_NONE;
    if (!measurement_request_ex(true, &reason))
    {
        char out[80];
        int n = snprintf(out, sizeof(out), "{\"error\":\"%s\"}", measurement_refuse_reason_str(reason));
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_send(req, out, n);
    }

    return httpd_resp_send(req, "{\"running\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t measurement_stop_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    ESP_LOGI(TAG, "Measurement stop requested");

    measurement_request(false);
    return httpd_resp_send(req, "{\"running\":false}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    float x[DB_MAX_SAMPLES];
    float y[DB_MAX_SAMPLES];
    size_t count = 0;
    db_snapshot(x, y, &count, DB_MAX_SAMPLES); // best-effort; count stays 0 on failure

    const esp_app_desc_t *desc = esp_app_get_description();

    char out[224];
    int n = snprintf(out, sizeof(out),
                      "{\"running\":%s,\"mode\":\"%s\",\"ina_ready\":%s,\"points\":%zu,\"version\":\"%s\",\"fault\":\"%s\"}",
                      measurement_is_running() ? "true" : "false",
                      measurement_get_producer_mode_label(),
                      g_app.ina_ready ? "true" : "false",
                      count,
                      desc->version,
                      measurement_fault_str(measurement_last_fault()));
    return httpd_resp_send(req, out, n);
}

// Consecutive HTTPD_SOCK_ERR_TIMEOUT retries tolerated in a single recv
// before giving up on the request (the client is presumably gone).
#define OTA_MAX_TIMEOUT_RETRIES 3

// Reads exactly `len` bytes into `buf`, retrying on timeout up to
// OTA_MAX_TIMEOUT_RETRIES times. Returns true on success; a 0 return from
// httpd_req_recv (peer closed) or a hard error is always treated as
// failure, never retried.
static bool httpd_recv_exact(httpd_req_t *req, char *buf, size_t len)
{
    size_t received = 0;
    int timeout_retries = 0;
    while (received < len)
    {
        int recv_len = httpd_req_recv(req, buf + received, len - received);
        if (recv_len > 0)
        {
            received += (size_t)recv_len;
            timeout_retries = 0;
            continue;
        }
        if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
        {
            timeout_retries++;
            if (timeout_retries <= OTA_MAX_TIMEOUT_RETRIES)
                continue;
            ESP_LOGE(TAG, "httpd_recv_exact: too many consecutive timeouts");
            return false;
        }
        // recv_len == 0 (peer closed) or any other error (< 0).
        ESP_LOGE(TAG, "httpd_recv_exact: recv failed (%d)", recv_len);
        return false;
    }
    return true;
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

    if (req->content_len > update_partition->size)
    {
        ESP_LOGE(TAG, "OTA upload too large: %zu > partition size %lu",
                 (size_t)req->content_len, (unsigned long)update_partition->size);
        httpd_resp_set_status(req, "413 Content Too Large");
        return httpd_resp_send(req, "{\"error\":\"Image larger than update partition\"}", HTTPD_RESP_USE_STRLEN);
    }

    char buf[1024];
    size_t remaining = req->content_len;

    // Read the first chunk before esp_ota_begin so a bad upload (wrong
    // magic byte) is rejected before any flash erase happens.
    size_t first_chunk = (remaining < sizeof(buf)) ? remaining : sizeof(buf);
    if (!httpd_recv_exact(req, buf, first_chunk))
    {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"Read failed\"}", HTTPD_RESP_USE_STRLEN);
    }

    if ((uint8_t)buf[0] != ESP_APP_IMAGE_MAGIC_BYTE)
    {
        ESP_LOGE(TAG, "OTA upload rejected: bad image magic byte 0x%02X", (uint8_t)buf[0]);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"Not a valid firmware image\"}", HTTPD_RESP_USE_STRLEN);
    }

    esp_ota_handle_t ota_handle = 0;
    // OTA_WITH_SEQUENTIAL_WRITES erases the partition incrementally as
    // writes come in, instead of erasing the whole ~1 MB slot up front.
    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"OTA begin failed\"}", HTTPD_RESP_USE_STRLEN);
    }

    err = esp_ota_write(ota_handle, buf, first_chunk);
    remaining -= first_chunk;
    bool write_ok = (err == ESP_OK);
    if (!write_ok)
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));

    while (write_ok && remaining > 0)
    {
        size_t to_recv = (remaining < sizeof(buf)) ? remaining : sizeof(buf);
        if (!httpd_recv_exact(req, buf, to_recv))
        {
            write_ok = false;
            break;
        }
        err = esp_ota_write(ota_handle, buf, to_recv);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            write_ok = false;
            break;
        }
        remaining -= to_recv;
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

static esp_err_t wifi_config_post_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    size_t to_read = req->content_len;
    if (to_read == 0 || to_read > 512)
    {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"Invalid body\"}", HTTPD_RESP_USE_STRLEN);
    }

    char buf[513] = {0};
    if (!httpd_recv_exact(req, buf, to_read))
    {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"Read failed\"}", HTTPD_RESP_USE_STRLEN);
    }
    buf[to_read] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root)
    {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"Invalid JSON\"}", HTTPD_RESP_USE_STRLEN);
    }

    char ssid_buf[64] = {0};
    char pass_buf[64] = {0};
    const cJSON *ssid_item = cJSON_GetObjectItemCaseSensitive(root, "ssid");
    if (cJSON_IsString(ssid_item) && ssid_item->valuestring)
        strncpy(ssid_buf, ssid_item->valuestring, sizeof(ssid_buf) - 1);
    const cJSON *pass_item = cJSON_GetObjectItemCaseSensitive(root, "password");
    if (cJSON_IsString(pass_item) && pass_item->valuestring)
        strncpy(pass_buf, pass_item->valuestring, sizeof(pass_buf) - 1);

    const char *ssid = ssid_buf;
    const char *password = pass_buf;
    size_t ssid_len = strlen(ssid);
    size_t pass_len = strlen(password);

    // ssid: 1..32, or empty (clears saved credentials).
    // password: 8..63, or empty (open network).
    if (ssid_len > 32 || pass_len > 63 || (pass_len > 0 && pass_len < 8))
    {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"Invalid ssid/password length\"}", HTTPD_RESP_USE_STRLEN);
    }

    bool clearing = (ssid_len == 0);

    nvs_handle_t h;
    esp_err_t err = nvs_open("wifi_cfg", NVS_READWRITE, &h);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        cJSON_Delete(root);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"NVS open failed\"}", HTTPD_RESP_USE_STRLEN);
    }

    if (clearing)
    {
        esp_err_t e1 = nvs_erase_key(h, "sta_ssid");
        esp_err_t e2 = nvs_erase_key(h, "sta_pass");
        if ((e1 != ESP_OK && e1 != ESP_ERR_NVS_NOT_FOUND) ||
            (e2 != ESP_OK && e2 != ESP_ERR_NVS_NOT_FOUND))
        {
            ESP_LOGE(TAG, "nvs_erase_key failed: ssid=%s pass=%s", esp_err_to_name(e1), esp_err_to_name(e2));
            nvs_close(h);
            cJSON_Delete(root);
            httpd_resp_set_status(req, "500 Internal Server Error");
            return httpd_resp_send(req, "{\"error\":\"NVS erase failed\"}", HTTPD_RESP_USE_STRLEN);
        }
    }
    else
    {
        esp_err_t e1 = nvs_set_str(h, "sta_ssid", ssid);
        esp_err_t e2 = nvs_set_str(h, "sta_pass", password);
        if (e1 != ESP_OK || e2 != ESP_OK)
        {
            ESP_LOGE(TAG, "nvs_set_str failed: ssid=%s pass=%s", esp_err_to_name(e1), esp_err_to_name(e2));
            nvs_close(h);
            cJSON_Delete(root);
            httpd_resp_set_status(req, "500 Internal Server Error");
            return httpd_resp_send(req, "{\"error\":\"NVS write failed\"}", HTTPD_RESP_USE_STRLEN);
        }
    }

    err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
        cJSON_Delete(root);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"NVS commit failed\"}", HTTPD_RESP_USE_STRLEN);
    }

    esp_err_t wifi_err = clearing ? wifi_clear_sta_credentials() : wifi_apply_sta_credentials(ssid, password);
    if (wifi_err != ESP_OK)
    {
        ESP_LOGE(TAG, "wifi %s failed: %s", clearing ? "clear" : "apply", esp_err_to_name(wifi_err));
        cJSON_Delete(root);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"WiFi apply failed\"}", HTTPD_RESP_USE_STRLEN);
    }

    ESP_LOGI(TAG, "WiFi STA credentials %s", clearing ? "cleared" : "saved");

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "ok", true);
    cJSON_AddStringToObject(resp, "ssid", ssid);
    char *resp_str = cJSON_PrintUnformatted(resp);
    esp_err_t send_err = httpd_resp_sendstr(req, resp_str ? resp_str : "{\"ok\":true}");
    cJSON_free(resp_str);
    cJSON_Delete(resp);
    cJSON_Delete(root);
    return send_err;
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

    httpd_uri_t data = {.uri = "/data", .method = HTTP_GET, .handler = data_get_handler};
    httpd_uri_t meas_start = {.uri = "/measurement/start", .method = HTTP_POST, .handler = measurement_start_handler};
    httpd_uri_t meas_stop = {.uri = "/measurement/stop", .method = HTTP_POST, .handler = measurement_stop_handler};
    httpd_uri_t status = {.uri = "/status", .method = HTTP_GET, .handler = status_get_handler};
    httpd_uri_t ota_post = {.uri = "/ota", .method = HTTP_POST, .handler = ota_post_handler};
    httpd_uri_t version = {.uri = "/version", .method = HTTP_GET, .handler = version_get_handler};
    httpd_uri_t wifi_cfg = {.uri = "/wifi-config", .method = HTTP_POST, .handler = wifi_config_post_handler};

    httpd_register_uri_handler(server, &data);
    httpd_register_uri_handler(server, &meas_start);
    httpd_register_uri_handler(server, &meas_stop);
    httpd_register_uri_handler(server, &status);
    httpd_register_uri_handler(server, &ota_post);
    httpd_register_uri_handler(server, &version);
    httpd_register_uri_handler(server, &wifi_cfg);
    ESP_LOGI(TAG, "HTTP server started");
    return ESP_OK;
}
