#ifndef INIT_H
#define INIT_H

#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <lwip/ip_addr.h>

#include "../server/server.h"
#include "../db/db.h"

/**
 * @brief Initialize WiFi in SoftAP mode.
 */
void wifi_init_softap(void);

/**
 * @brief Get configured SoftAP SSID used during startup.
 */
const char *wifi_softap_ssid(void);

/**
 * @brief Get configured SoftAP password used during startup.
 */
const char *wifi_softap_password(void);

/**
 * @brief Return true when SoftAP auth mode is open/no password.
 */
bool wifi_softap_is_open(void);

/**
 * @brief Apply new STA credentials at runtime: switches AP-only mode to
 * APSTA if needed, sets the STA config, resets the disconnect-retry
 * counter, and kicks off a connection attempt.
 *
 * Does not touch NVS; the caller is responsible for persisting (or
 * clearing) the credentials.
 *
 * @return ESP_OK on success, or the first esp_wifi_* error encountered.
 */
esp_err_t wifi_apply_sta_credentials(const char *ssid, const char *password);

/**
 * @brief Clear runtime STA state: disconnects (if connected/connecting),
 * resets the retry counter, and drops back to AP-only mode if currently
 * APSTA.
 *
 * Does not touch NVS; the caller is responsible for erasing saved
 * credentials.
 *
 * @return ESP_OK on success, or the first esp_wifi_* error encountered.
 */
esp_err_t wifi_clear_sta_credentials(void);

/**
 * @brief Initialize all system components:
 * - NVS
 * - WiFi
 * - HTTP server
 * - Database/State abstraction
 */
void system_init_all(void);

#endif // INIT_H
