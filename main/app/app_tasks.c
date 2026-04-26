#include "app_tasks.h"

#include <stdio.h>

#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_hw.h"
#include "app_state.h"
#include "measurement.h"
#include "ui.h"

#include "driver_encoder.h"
#include "driver_sh1106.h"
#include "init.h"

static const char *TAG = "APP_TASKS";

void app_display_mark_dirty(void)
{
    g_app.display_dirty = true;
    if (g_app.display_task != NULL)
    {
        xTaskNotifyGive(g_app.display_task);
    }
}

void app_tasks_stop_display(void)
{
    g_app.display_enabled = false;
    if (g_app.display_task != NULL)
    {
        xTaskNotifyGive(g_app.display_task);
    }
}

static void build_wifi_qr_payload(char *out, size_t out_size)
{
    const char *ssid = wifi_softap_ssid();
    const char *password = wifi_softap_password();

    if (wifi_softap_is_open())
    {
        snprintf(out, out_size, "WIFI:T:nopass;S:%s;;", ssid);
    }
    else
    {
        snprintf(out, out_size, "WIFI:T:WPA;S:%s;P:%s;;", ssid, password);
    }
}

static void display_task(void *arg)
{
    (void)arg;

    while (g_app.display_enabled)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));

        if (g_app.ui_screen == UI_SCREEN_ACTION_MEASURE)
        {
            // Keep START/STOP state fresh even when producer changes state asynchronously.
            g_app.display_dirty = true;
        }

        if (g_app.ui_screen == UI_SCREEN_ACTION_DYNAMIC_LOAD)
        {
            dynamic_load_update_measured();
            g_app.display_dirty = true;
        }

        if (!g_app.display_dirty)
        {
            continue;
        }

        ui_render_display_frame(g_app.qr_frame);
        g_app.qr_frame_valid = true;

        esp_err_t ret = sh1106_flush(&g_app.display, g_app.qr_frame, sizeof(g_app.qr_frame));
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "display_task: flush failed: %s", esp_err_to_name(ret));
            continue;
        }

        g_app.display_dirty = false;
    }

    vTaskDelete(NULL);
}

static void display_init_and_start(void)
{
    build_wifi_qr_payload(g_app.wifi_qr_payload, sizeof(g_app.wifi_qr_payload));

    if (app_hw_ensure_i2c_bus_ready() != ESP_OK)
    {
        ESP_LOGW(TAG, "display_init_and_start: I2C bus unavailable, skipping OLED");
        return;
    }

    const uint8_t candidate_addrs[] = {
        SH1106_DEFAULT_I2C_ADDR,
        SH1106_ALT_I2C_ADDR,
    };

    esp_err_t ret = ESP_FAIL;
    uint8_t selected_addr = 0;
    for (size_t i = 0; i < sizeof(candidate_addrs) / sizeof(candidate_addrs[0]); i++)
    {
        ret = sh1106_init_on_bus(&g_app.display, g_app.i2c_bus, candidate_addrs[i], 400000);
        if (ret == ESP_OK)
        {
            selected_addr = candidate_addrs[i];
            break;
        }
    }

    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "display_init_and_start: SH1106 init failed at 0x%02X/0x%02X: %s",
                 SH1106_DEFAULT_I2C_ADDR, SH1106_ALT_I2C_ADDR, esp_err_to_name(ret));
        return;
    }

    ret = sh1106_set_rotation(&g_app.display, OLED_ROTATE_180);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "display_init_and_start: failed to set OLED rotation: %s", esp_err_to_name(ret));
    }

    sh1106_set_column_offset(&g_app.display, OLED_COLUMN_OFFSET);

    g_app.display_enabled = true;
    if (xTaskCreate(display_task, "display", 4096, NULL, 4, &g_app.display_task) != pdPASS)
    {
        ESP_LOGE(TAG, "display_init_and_start: failed to create display task");
        g_app.display_enabled = false;
        return;
    }

    app_display_mark_dirty();

    ESP_LOGI(TAG, "display_init_and_start: OLED menu UI started at 0x%02X (offset=%u), AP payload='%s'",
             selected_addr, OLED_COLUMN_OFFSET, g_app.wifi_qr_payload);
}

static void encoder_ui_task(void *arg)
{
    (void)arg;

    encoder_event_t ev;
    for (;;)
    {
        if (!encoder_get_event(&ev, portMAX_DELAY))
        {
            continue;
        }

        if (ev.type == ENCODER_EVENT_CW)
        {
            ESP_LOGI(TAG, "ENCODER: CW pos=%ld t=%lu", (long)ev.position, (unsigned long)ev.timestamp_ms);
            ui_on_rotate(+1);
        }
        else if (ev.type == ENCODER_EVENT_CCW)
        {
            ESP_LOGI(TAG, "ENCODER: CCW pos=%ld t=%lu", (long)ev.position, (unsigned long)ev.timestamp_ms);
            ui_on_rotate(-1);
        }
        else
        {
            ESP_LOGI(TAG, "ENCODER: BUTTON pos=%ld t=%lu", (long)ev.position, (unsigned long)ev.timestamp_ms);
            ui_on_button();
        }
    }
}

void app_tasks_start(void)
{
    display_init_and_start();

    encoder_config_t enc_cfg = {
        .dt_pin = ENC_DT_GPIO,
        .clk_pin = ENC_CLK_GPIO,
        .sw_pin = ENC_SW_GPIO,
        .use_internal_pullups = true,
        .sw_debounce_ms = 180,
        .event_queue_len = 32,
    };

    esp_err_t enc_ret = encoder_init(&enc_cfg);
    if (enc_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "encoder_init failed: %s", esp_err_to_name(enc_ret));
    }
    else if (xTaskCreate(encoder_ui_task, "encoder_ui", 3072, NULL, 4, &g_app.encoder_task) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create encoder UI task");
        g_app.encoder_task = NULL;
    }
}
