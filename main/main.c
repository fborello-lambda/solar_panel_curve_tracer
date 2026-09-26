#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_system.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "app/app_state.h"
#include "app/app_tasks.h"
#include "measurement.h"
#include "ui.h"

#include "init.h"
#include "led_controller.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    // Waking from deep sleep is already a full chip reset that re-enters
    // app_main from scratch, so no extra esp_restart() is needed here; this
    // just logs why we're booting.
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    if (wakeup_cause == ESP_SLEEP_WAKEUP_GPIO)
    {
        ESP_LOGI(TAG, "Wakeup cause: GPIO (encoder switch)");
    }

    g_app.state_mtx = xSemaphoreCreateMutex();
    if (g_app.state_mtx == NULL)
    {
        ESP_LOGE(TAG, "Failed to create state mutex");
        return;
    }

    // Networking/HTTP/OTA must always come up, even if the load control
    // hardware (PWM/INA219) is missing or faulty; do that init first and
    // treat it as non-fatal.
    system_init_all();

    if (!measurement_init_load_control_hw())
    {
        ESP_LOGW(TAG, "Load control hardware not fully ready; REAL measurement mode will be refused");
    }

    if (led_init(WS2812_GPIO) == ESP_OK)
    {
        led_clear(WS2812_GPIO);
    }
    else
    {
        ESP_LOGW(TAG, "WS2812 init failed on GPIO %d", WS2812_GPIO);
    }

    measurement_set_producer_mode(CURVE_PRODUCER_REAL);
    ui_init_state();
    app_tasks_start();
}
