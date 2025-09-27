#include "server.h"

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
    httpd_uri_t data = {.uri = "/data", .method = HTTP_GET, .handler = data_get_handler};
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &chart);
    httpd_register_uri_handler(server, &data);
    ESP_LOGI(TAG, "HTTP server started");
    return ESP_OK;
}
