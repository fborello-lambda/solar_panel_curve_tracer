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
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    if (wakeup_cause == ESP_SLEEP_WAKEUP_GPIO)
    {
        ESP_LOGI(TAG, "Wakeup cause: GPIO (encoder switch)");
        // Force a clean boot path so UI/tasks/peripherals always start from scratch.
        esp_restart();
    }

    g_app.state_mtx = xSemaphoreCreateMutex();
    if (g_app.state_mtx == NULL)
    {
        ESP_LOGE(TAG, "Failed to create state mutex");
        return;
    }

    if (!measurement_init_load_control_hw(true))
    {
        return;
    }

    system_init_all();

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
