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

static esp_err_t send_file(httpd_req_t *req, const char *path, const char *type)
{
    FILE *f = fopen(path, "rb");
    if (!f)
    {
        httpd_resp_set_status(req, "404 Not Found");
        return httpd_resp_send(req, "Not found", HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_set_type(req, type);
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=86400");
    char buf[1024];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
    {
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK)
        {
            fclose(f);
            return ESP_FAIL;
        }
    }
    fclose(f);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t root_get_handler(httpd_req_t *req) { return send_file(req, "/spiffs/index.html", "text/html; charset=utf-8"); }
static esp_err_t chart_get_handler(httpd_req_t *req) { return send_file(req, "/spiffs/chart.umd.min.js", "application/javascript"); }
static esp_err_t script_get_handler(httpd_req_t *req) { return send_file(req, "/spiffs/script.js", "application/javascript"); }
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

static esp_err_t set_current_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    // Read request body
    size_t to_read = req->content_len;
    if (to_read == 0 || to_read > 256)
    {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "Invalid body", HTTPD_RESP_USE_STRLEN);
    }

    char buf[257];
    size_t received = 0;
    while (received < to_read)
    {
        int r = httpd_req_recv(req, buf + received, to_read - received);
        if (r <= 0)
        {
            if (r == HTTPD_SOCK_ERR_TIMEOUT)
                continue;
            httpd_resp_set_status(req, "400 Bad Request");
            return httpd_resp_send(req, "Failed to read body", HTTPD_RESP_USE_STRLEN);
        }
        received += r;
    }
    buf[received] = '\0';

    // Determine content-type
    char ctype[32] = {0};
    size_t ctlen = httpd_req_get_hdr_value_len(req, "Content-Type");
    if (ctlen > 0 && ctlen < sizeof(ctype))
        httpd_req_get_hdr_value_str(req, "Content-Type", ctype, sizeof(ctype));

    // Parse number (milliamps)
    double value_mA = 0.0;
    bool ok = false;

    if ((ctype[0] && strstr(ctype, "application/json")) || (buf[0] == '{'))
    {
        // Very small JSON parsing without cJSON: look for "current_mA"
        const char *key = strstr(buf, "current_mA");
        if (key)
        {
            const char *colon = strchr(key, ':');
            if (colon)
            {
                char *endptr = NULL;
                value_mA = strtod(colon + 1, &endptr);
                if (endptr && endptr != colon + 1)
                    ok = true;
            }
        }
    }

    if (!ok)
    {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "Invalid number", HTTPD_RESP_USE_STRLEN);
    }

    float current_setpoint_mA = db_get_current_setpoint_mA();
    if (current_setpoint_mA > 0.0 && fabs(value_mA - current_setpoint_mA) < 0.01)
    {
        // No change
        ESP_LOGI(TAG, "Set current setpoint: %.3f mA (no change)", current_setpoint_mA);

        httpd_resp_set_status(req, "200 OK");
        char out[96];
        int n = snprintf(out, sizeof(out), "{\"change\":false,\"current_mA\":%.3f}", current_setpoint_mA);
        return httpd_resp_send(req, out, n);
    }

    // Clamp to a safe range (0 .. MAX_CURRENT_MA)
    if (value_mA < 0.0)
        value_mA = 0.0;
    if (value_mA > MAX_CURRENT_MA)
        value_mA = MAX_CURRENT_MA;

    current_setpoint_mA = (float)value_mA;
    db_set_current_setpoint_mA(current_setpoint_mA);
    ESP_LOGI(TAG, "Set current setpoint: %.3f mA", current_setpoint_mA);

    // TODO: call your control layer here, e.g. current_control_set_mA(g_current_setpoint_mA);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    char out[96];
    int n = snprintf(out, sizeof(out), "{\"change\":true,\"current_mA\":%.3f}", current_setpoint_mA);
    return httpd_resp_send(req, out, n);
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

static esp_err_t get_current_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    float current_setpoint_mA = db_get_current_setpoint_mA();

    char out[64];

    int n = snprintf(out, sizeof(out), "{\"current_mA\":%.3f}", current_setpoint_mA);
    return httpd_resp_send(req, out, n);
}

static esp_err_t ota_get_handler(httpd_req_t *req)
{
    return send_file(req, "/spiffs/ota.html", "text/html; charset=utf-8");
}

static esp_err_t spiffs_ota_post_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    if (req->content_len == 0)
    {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"Empty body\"}", HTTPD_RESP_USE_STRLEN);
    }

    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "storage");
    if (!partition)
    {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"Storage partition not found\"}", HTTPD_RESP_USE_STRLEN);
    }

    esp_err_t err = esp_partition_erase_range(partition, 0, partition->size);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Partition erase failed: %s", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"Erase failed\"}", HTTPD_RESP_USE_STRLEN);
    }

    char buf[1024];
    int remaining = (int)req->content_len;
    uint32_t offset = 0;
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
        err = esp_partition_write(partition, offset, buf, recv_len);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Partition write failed at offset %lu: %s", (unsigned long)offset, esp_err_to_name(err));
            write_ok = false;
            break;
        }
        offset += recv_len;
        remaining -= recv_len;
    }

    if (!write_ok)
    {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"Write failed\"}", HTTPD_RESP_USE_STRLEN);
    }

    ESP_LOGI(TAG, "SPIFFS OTA complete (%lu bytes), rebooting", (unsigned long)offset);
    httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
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

esp_err_t spiffs_init(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 8,
        .format_if_mount_failed = true};
    return esp_vfs_spiffs_register(&conf);
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

    httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_get_handler};
    httpd_uri_t chart = {.uri = "/chart.js", .method = HTTP_GET, .handler = chart_get_handler};
    httpd_uri_t script = {.uri = "/script.js", .method = HTTP_GET, .handler = script_get_handler};
    httpd_uri_t data = {.uri = "/data", .method = HTTP_GET, .handler = data_get_handler};
    httpd_uri_t set_current = {.uri = "/set-current", .method = HTTP_POST, .handler = set_current_handler};
    httpd_uri_t get_current = {.uri = "/current", .method = HTTP_GET, .handler = get_current_handler};
    httpd_uri_t start_meas = {.uri = "/start-measurement", .method = HTTP_POST, .handler = start_measurement_handler};
    httpd_uri_t ota_get = {.uri = "/ota", .method = HTTP_GET, .handler = ota_get_handler};
    httpd_uri_t ota_post = {.uri = "/ota", .method = HTTP_POST, .handler = ota_post_handler};
    httpd_uri_t spiffs_ota_post = {.uri = "/ota/spiffs", .method = HTTP_POST, .handler = spiffs_ota_post_handler};
    httpd_uri_t version = {.uri = "/version", .method = HTTP_GET, .handler = version_get_handler};
    httpd_uri_t wifi_cfg = {.uri = "/wifi-config", .method = HTTP_POST, .handler = wifi_config_post_handler};

    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &chart);
    httpd_register_uri_handler(server, &script);
    httpd_register_uri_handler(server, &data);
    httpd_register_uri_handler(server, &set_current);
    httpd_register_uri_handler(server, &get_current);
    httpd_register_uri_handler(server, &start_meas);
    httpd_register_uri_handler(server, &ota_get);
    httpd_register_uri_handler(server, &ota_post);
    httpd_register_uri_handler(server, &spiffs_ota_post);
    httpd_register_uri_handler(server, &version);
    httpd_register_uri_handler(server, &wifi_cfg);
    ESP_LOGI(TAG, "HTTP server started");
    return ESP_OK;
}
