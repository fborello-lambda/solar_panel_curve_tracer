#ifndef SERVER_SERVER_H
#define SERVER_SERVER_H

#include <stdio.h>
#include <esp_log.h>
#include <esp_spiffs.h>
#include <esp_vfs.h>
#include <esp_netif.h>
#include <esp_http_server.h>

#include "../utils/json_builder.h"
#include "../db/db.h"

/**
 * @brief Initialize SPIFFS filesystem.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t spiffs_init(void);

/**
 * @brief Initialize the HTTP server and register URI handlers.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t server_init(void);

#endif // SERVER_SERVER_H
