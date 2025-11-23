#include "server.h"
#include <math.h>

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
static esp_err_t get_current_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    float current_setpoint_mA = db_get_current_setpoint_mA();

    char out[64];

    int n = snprintf(out, sizeof(out), "{\"current_mA\":%.3f}", current_setpoint_mA);
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
    config.stack_size = 8192;
    config.lru_purge_enable = true;
    config.max_open_sockets = 6;
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

    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &chart);
    httpd_register_uri_handler(server, &script);
    httpd_register_uri_handler(server, &data);
    httpd_register_uri_handler(server, &set_current);
    httpd_register_uri_handler(server, &get_current);
    ESP_LOGI(TAG, "HTTP server started");
    return ESP_OK;
}
