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
 * @brief Initialize all system components:
 * - NVS
 * - SPIFFS
 * - WiFi
 * - HTTP server
 * - Database/State abstraction
 */
void system_init_all(void);

#endif // INIT_H
