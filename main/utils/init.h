#ifndef INIT_H
#define INIT_H

#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>

#include "../server/server.h"
#include "../db/db.h"

/**
 * @brief Initialize WiFi in SoftAP mode.
 */
void wifi_init_softap(void);

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
