#include "ui.h"

#include <stdio.h>
#include <string.h>

#include <driver/gpio.h>
#include <esp_app_desc.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_system.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app/app_state.h"
#include "app/app_tasks.h"
#include "measurement.h"

#include "driver_sh1106.h"
#include "led_controller.h"
#include "pwm_controller.h"
#include "qrcode.h"

static const char *TAG = "UI";

static int wrap_index(int value, int count)
{
    if (count <= 0)
    {
        return 0;
    }

    while (value < 0)
    {
        value += count;
    }
    while (value >= count)
    {
        value -= count;
    }
    return value;
}

static void ui_set_screen(ui_screen_t screen)
{
    g_app.ui_screen = screen;
    app_display_mark_dirty();
}

static const char *ui_home_title(int index)
{
    static const char *titles[HOME_SECTION_COUNT] = {
        "NETWORK",
        "MEASURE",
        "SYSTEM",
    };

    int i = wrap_index(index, HOME_SECTION_COUNT);
    return titles[i];
}

static const char *ui_menu_item_label(int home_index, int menu_index)
{
    if (home_index == HOME_SECTION_NETWORK)
    {
        if (menu_index == 0)
        {
            return "SHOW WIFI QR";
        }
        if (menu_index == 1)
        {
            return "SHOW AP IP QR";
        }
        return "BACK";
    }

    if (home_index == HOME_SECTION_POWER)
    {
        if (menu_index == 0)
        {
            return "OTA";
        }
        if (menu_index == 1)
        {
            return "RESET";
        }
        if (menu_index == 2)
        {
            return "DEEP SLEEP";
        }
        return "BACK";
    }


    if (home_index == HOME_SECTION_MEASURE)
    {
        if (menu_index == 0)
        {
            return "CURVE TRACER";
        }
        if (menu_index == 1)
        {
            return "DYNAMIC LOAD";
        }
        return "BACK";
    }

    return "BACK";
}

static int ui_menu_item_count(int home_index)
{
    if (home_index == HOME_SECTION_NETWORK)
    {
        return 3;
    }
    if (home_index == HOME_SECTION_MEASURE)
    {
        return 3;
    }
    if (home_index == HOME_SECTION_POWER)
    {
        return 4;
    }
    return 2;
}

static void enter_deep_sleep_mode(void)
{
    measurement_request(false);
    pwm_controller_set_duty(0);
    led_clear(WS2812_GPIO);

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    esp_err_t ret = gpio_deep_sleep_wakeup_enable(ENC_SW_GPIO, GPIO_INTR_LOW_LEVEL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "deep_sleep: gpio_deep_sleep_wakeup_enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_deep_sleep_enable_gpio_wakeup((1ULL << ENC_SW_GPIO), ESP_GPIO_WAKEUP_GPIO_LOW);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "deep_sleep: esp_deep_sleep_enable_gpio_wakeup failed: %s", esp_err_to_name(ret));
        return;
    }

    app_tasks_stop_display();

    if (g_app.display.dev != NULL)
    {
        esp_err_t clear_ret = sh1106_clear(&g_app.display);
        if (clear_ret != ESP_OK)
        {
            ESP_LOGW(TAG, "deep_sleep: failed to clear OLED: %s", esp_err_to_name(clear_ret));
        }
    }

    ESP_LOGI(TAG, "Entering deep sleep. Wake source: encoder switch GPIO %d (active low)", ENC_SW_GPIO);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_deep_sleep_start();
}

void ui_init_state(void)
{
    g_app.ui_screen = UI_SCREEN_HOME;
    g_app.ui_qr_kind = UI_QR_WIFI;
    g_app.ui_home_index = HOME_SECTION_NETWORK;
    g_app.ui_menu_index = 0;
    g_app.ui_measure_index = 0;
    g_app.ui_system_index = 0;

    g_app.dynamic_load_active = false;
    g_app.dynamic_duty_steps = 0;
    g_app.dynamic_measured_mA = 0.0f;
    g_app.dynamic_power_mW = 0.0f;
    g_app.dynamic_bus_mv = 0;
    g_app.dynamic_shunt_uv = 0;
    g_app.dynamic_measured_valid = false;
    g_app.dynamic_power_limited = false;
    g_app.dynamic_last_adjust_tick = 0;
    g_app.dynamic_last_sample_tick = 0;

    app_display_mark_dirty();
}

void ui_on_rotate(int dir)
{
    if (g_app.ui_screen == UI_SCREEN_HOME)
    {
        g_app.ui_home_index = wrap_index(g_app.ui_home_index + dir, HOME_SECTION_COUNT);
        app_display_mark_dirty();
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_MENU)
    {
        g_app.ui_menu_index = wrap_index(g_app.ui_menu_index + dir, ui_menu_item_count(g_app.ui_home_index));
        app_display_mark_dirty();
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_ACTION_MEASURE)
    {
        g_app.ui_measure_index = wrap_index(g_app.ui_measure_index + dir, 3);
        app_display_mark_dirty();
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_ACTION_SYSTEM)
    {
        g_app.ui_system_index = wrap_index(g_app.ui_system_index + dir, 4);
        app_display_mark_dirty();
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_ACTION_DYNAMIC_LOAD)
    {
        dynamic_load_adjust(dir);
        app_display_mark_dirty();
    }
}

void ui_on_button(void)
{
    if (g_app.ui_screen == UI_SCREEN_HOME)
    {
        g_app.ui_menu_index = 0;
        ui_set_screen(UI_SCREEN_MENU);
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_MENU)
    {
        int item_count = ui_menu_item_count(g_app.ui_home_index);
        int back_index = item_count - 1;

        if (g_app.ui_menu_index == back_index)
        {
            ui_set_screen(UI_SCREEN_HOME);
            return;
        }

        if (g_app.ui_home_index == HOME_SECTION_NETWORK)
        {
            g_app.ui_qr_kind = (g_app.ui_menu_index == 0) ? UI_QR_WIFI : UI_QR_AP_IP;
            ui_set_screen(UI_SCREEN_ACTION_QR);
            return;
        }

        if (g_app.ui_home_index == HOME_SECTION_POWER)
        {
            if (g_app.ui_menu_index == 0)
            {
                g_app.ui_qr_kind = UI_QR_OTA;
                ui_set_screen(UI_SCREEN_ACTION_QR);
                return;
            }

            if (g_app.ui_menu_index == 1)
            {
                ESP_LOGI(TAG, "SYSTEM: reset requested");
                esp_restart();
                return;
            }

            if (g_app.ui_menu_index == 2)
            {
                enter_deep_sleep_mode();
                return;
            }

            ui_set_screen(UI_SCREEN_HOME);
            return;
        }

        if (g_app.ui_home_index == HOME_SECTION_MEASURE && g_app.ui_menu_index == 1)
        {
            measurement_request(false);
            dynamic_load_enter();
            ui_set_screen(UI_SCREEN_ACTION_DYNAMIC_LOAD);
            return;
        }

        if (g_app.ui_home_index == HOME_SECTION_MEASURE && g_app.ui_menu_index == 0)
        {
            g_app.ui_measure_index = 0;
            ui_set_screen(UI_SCREEN_ACTION_MEASURE);
            return;
        }

        ui_set_screen(UI_SCREEN_HOME);
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_ACTION_QR)
    {
        ui_set_screen(UI_SCREEN_MENU);
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_ACTION_MEASURE)
    {
        if (g_app.ui_measure_index == 0)
        {
            bool start = !measurement_is_running();
            if (start)
            {
                dynamic_load_exit();
            }

            if (!measurement_request(start))
            {
                ESP_LOGW(TAG, "UI: measurement %s request ignored", start ? "start" : "stop");
            }
            app_display_mark_dirty();
            return;
        }

        if (g_app.ui_measure_index == 1)
        {
            curve_producer_mode_t mode = measurement_get_producer_mode();
            curve_producer_mode_t next_mode = (mode == CURVE_PRODUCER_DUMMY) ? CURVE_PRODUCER_REAL : CURVE_PRODUCER_DUMMY;
            measurement_set_producer_mode(next_mode);
            ESP_LOGI(TAG, "Curve tracer mode set to %s", measurement_get_producer_mode_label());
            app_display_mark_dirty();
            return;
        }

        ui_set_screen(UI_SCREEN_MENU);
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_ACTION_SYSTEM)
    {
        if (g_app.ui_system_index == 0)
        {
            enter_deep_sleep_mode();
            return;
        }

        if (g_app.ui_system_index == 1)
        {
            ESP_LOGI(TAG, "SYSTEM: restart requested");
            esp_restart();
            return;
        }

        if (g_app.ui_system_index == 2)
        {
            g_app.ui_qr_kind = UI_QR_OTA;
            ui_set_screen(UI_SCREEN_ACTION_QR);
            return;
        }

        ui_set_screen(UI_SCREEN_MENU);
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_ACTION_DYNAMIC_LOAD)
    {
        dynamic_load_exit();
        ui_set_screen(UI_SCREEN_MENU);
    }
}

typedef struct
{
    uint8_t *fb;
    int cell_px;
    int quiet_zone;
} qr_render_ctx_t;

static void qr_draw_to_oled_cb(esp_qrcode_handle_t qrcode, void *user_data)
{
    qr_render_ctx_t *ctx = (qr_render_ctx_t *)user_data;
    if (!ctx || !ctx->fb || !qrcode)
    {
        return;
    }

    int size = esp_qrcode_get_size(qrcode);
    int quiet = ctx->quiet_zone;
    int cell_px = ctx->cell_px;
    int modules = size + (quiet * 2);

    while ((modules * cell_px) > (SH1106_HEIGHT - 2) && cell_px > 1)
    {
        cell_px--;
    }
    while (((size + (quiet * 2)) * cell_px) > (SH1106_HEIGHT - 2) && quiet > 0)
    {
        quiet--;
    }

    modules = size + (quiet * 2);
    int qr_px = modules * cell_px;
    int x0 = (SH1106_WIDTH - qr_px) / 2;
    int y0 = ((SH1106_HEIGHT - qr_px) / 2) + OLED_QR_Y_OFFSET;

    if (y0 < 1)
    {
        y0 = 1;
    }
    if (y0 + qr_px > SH1106_HEIGHT - 1)
    {
        y0 = SH1106_HEIGHT - qr_px - 1;
    }

    sh1106_fb_draw_rect(ctx->fb, x0 - 1, y0 - 1, qr_px + 2, qr_px + 2, false, true);

    for (int y = 0; y < modules; y++)
    {
        for (int x = 0; x < modules; x++)
        {
            int mx = x - quiet;
            int my = y - quiet;
            bool on = false;

            if (mx >= 0 && my >= 0 && mx < size && my < size)
            {
                on = esp_qrcode_get_module(qrcode, mx, my);
            }

            if (on)
            {
                sh1106_fb_draw_rect(ctx->fb,
                                    x0 + x * cell_px,
                                    y0 + y * cell_px,
                                    cell_px,
                                    cell_px,
                                    true,
                                    true);
            }
        }
    }
}

static void draw_real_qr_to_fb(uint8_t *fb, const char *payload)
{
    if (!fb || !payload || payload[0] == '\0')
    {
        return;
    }

    qr_render_ctx_t ctx = {
        .fb = fb,
        .cell_px = 2,
        .quiet_zone = 2,
    };

    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    cfg.display_func_with_cb = qr_draw_to_oled_cb;
    cfg.user_data = &ctx;
    cfg.max_qrcode_version = 2;
    cfg.qrcode_ecc_level = ESP_QRCODE_ECC_LOW;

    if (esp_qrcode_generate(&cfg, payload) != ESP_OK)
    {
        sh1106_fb_draw_text(fb, 0, 0, "QR GEN ERROR");
        sh1106_fb_draw_text(fb, 0, 12, "AP SSID ERROR");
    }
}

void ui_render_display_frame(uint8_t *fb)
{
    sh1106_fb_clear(fb, false);

    if (g_app.ui_screen == UI_SCREEN_HOME)
    {
        char home0[24] = {0};
        char home1[24] = {0};
        char home2[24] = {0};

        snprintf(home0, sizeof(home0), "%s %s",
                 g_app.ui_home_index == HOME_SECTION_NETWORK ? ">>" : "  ",
                 ui_home_title(HOME_SECTION_NETWORK));
        snprintf(home1, sizeof(home1), "%s %s",
                 g_app.ui_home_index == HOME_SECTION_MEASURE ? ">>" : "  ",
                 ui_home_title(HOME_SECTION_MEASURE));
        snprintf(home2, sizeof(home2), "%s %s",
                 g_app.ui_home_index == HOME_SECTION_POWER ? ">>" : "  ",
                 ui_home_title(HOME_SECTION_POWER));

        sh1106_fb_draw_text(fb, 0, 8, "HOME");
        sh1106_fb_draw_text(fb, 0, 22, home0);
        sh1106_fb_draw_text(fb, 0, 36, home1);
        sh1106_fb_draw_text(fb, 0, 50, home2);
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_MENU)
    {
        char line0[24] = {0};
        char line1[24] = {0};
        char line2[24] = {0};
        char line3[24] = {0};
        int item_count = ui_menu_item_count(g_app.ui_home_index);

        snprintf(line0, sizeof(line0), "%s %s",
                 g_app.ui_menu_index == 0 ? ">>" : "  ",
                 ui_menu_item_label(g_app.ui_home_index, 0));
        snprintf(line1, sizeof(line1), "%s %s",
                 g_app.ui_menu_index == 1 ? ">>" : "  ",
                 ui_menu_item_label(g_app.ui_home_index, 1));
        if (item_count > 2)
        {
            snprintf(line2, sizeof(line2), "%s %s",
                     g_app.ui_menu_index == 2 ? ">>" : "  ",
                     ui_menu_item_label(g_app.ui_home_index, 2));
        }
        if (item_count > 3)
        {
            snprintf(line3, sizeof(line3), "%s %s",
                     g_app.ui_menu_index == 3 ? ">>" : "  ",
                     ui_menu_item_label(g_app.ui_home_index, 3));
        }

        int section_y = (item_count > 3) ? 16 : 18;
        int line0_y = (item_count > 3) ? 24 : 30;
        int line1_y = (item_count > 3) ? 34 : 42;
        int line2_y = (item_count > 3) ? 44 : 54;
        int line3_y = 56;

        sh1106_fb_draw_text(fb, 0, 8, "MENU");
        sh1106_fb_draw_text(fb, 0, section_y, ui_home_title(g_app.ui_home_index));
        sh1106_fb_draw_text(fb, 0, line0_y, line0);
        sh1106_fb_draw_text(fb, 0, line1_y, line1);
        if (item_count > 2)
        {
            sh1106_fb_draw_text(fb, 0, line2_y, line2);
        }
        if (item_count > 3)
        {
            sh1106_fb_draw_text(fb, 0, line3_y, line3);
        }
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_ACTION_QR)
    {
        const char *payload;
        if (g_app.ui_qr_kind == UI_QR_AP_IP)
            payload = "http://192.168.4.1";
        else if (g_app.ui_qr_kind == UI_QR_OTA)
            payload = "http://192.168.4.1/ota";
        else
            payload = g_app.wifi_qr_payload;
        draw_real_qr_to_fb(fb, payload);
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_ACTION_DYNAMIC_LOAD)
    {
        char set_line[24] = {0};
        char current_line[24] = {0};
        char power_line[24] = {0};
        char vbus_line[24] = {0};
        char status_line[24] = {0};

        uint32_t pwm_res_disp = 0;
        pwm_controller_get_resolution(&pwm_res_disp);
        float duty_pct = (pwm_res_disp > 0) ? ((float)g_app.dynamic_duty_steps * 100.0f / (float)pwm_res_disp) : 0.0f;
        snprintf(set_line, sizeof(set_line), "PWM : %.2f %%", duty_pct);

        if (g_app.dynamic_measured_valid)
        {
            snprintf(current_line, sizeof(current_line), "I   : %.1f MA", g_app.dynamic_measured_mA);
            snprintf(power_line, sizeof(power_line), "PWR : %.0f MW", g_app.dynamic_power_mW);
            snprintf(vbus_line, sizeof(vbus_line), "VBUS: %ld MV", (long)g_app.dynamic_bus_mv);
        }
        else
        {
            snprintf(current_line, sizeof(current_line), "I   : N/A");
            snprintf(power_line, sizeof(power_line), "PWR : N/A");
            snprintf(vbus_line, sizeof(vbus_line), "VBUS: N/A");
        }

        if (g_app.dynamic_power_limited)
        {
            snprintf(status_line, sizeof(status_line), "LIMIT: HOLD <=2W");
        }
        else
        {
            snprintf(status_line, sizeof(status_line), "PWM : %lu", (unsigned long)g_app.dynamic_duty_steps);
        }

        sh1106_fb_draw_text(fb, 0, 8, "DYNAMIC LOAD");
        sh1106_fb_draw_text(fb, 0, 18, set_line);
        sh1106_fb_draw_text(fb, 0, 28, current_line);
        sh1106_fb_draw_text(fb, 0, 38, power_line);
        sh1106_fb_draw_text(fb, 0, 48, vbus_line);
        sh1106_fb_draw_text(fb, 0, 56, status_line);
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_ACTION_MEASURE)
    {
        char action_line[24] = {0};
        char mode_line[24] = {0};
        char back_line[24] = {0};

        snprintf(action_line, sizeof(action_line), "%s %s",
                 g_app.ui_measure_index == 0 ? ">>" : "  ",
                 measurement_is_running() ? "STOP TRACE" : "START TRACE");
        snprintf(mode_line, sizeof(mode_line), "%s MODE: %s",
                 g_app.ui_measure_index == 1 ? ">>" : "  ",
                 measurement_get_producer_mode_label());
        snprintf(back_line, sizeof(back_line), "%s BACK",
                 g_app.ui_measure_index == 2 ? ">>" : "  ");

        sh1106_fb_draw_text(fb, 0, 8, "CURVE TRACER");
        sh1106_fb_draw_text(fb, 0, 20, measurement_is_running() ? "STATE: RUNNING" : "STATE: STOPPED");
        sh1106_fb_draw_text(fb, 0, 32, action_line);
        sh1106_fb_draw_text(fb, 0, 44, mode_line);
        sh1106_fb_draw_text(fb, 0, 56, back_line);
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_ACTION_SYSTEM)
    {
        char ver_line[48] = {0};
        char sleep_line[24] = {0};
        char restart_line[24] = {0};
        char ota_line[24] = {0};
        char back_line[24] = {0};

        const esp_app_desc_t *desc = esp_app_get_description();
        snprintf(ver_line, sizeof(ver_line), "SYS v%s", desc->version);
        snprintf(sleep_line, sizeof(sleep_line), "%s DEEP SLEEP",
                 g_app.ui_system_index == 0 ? ">>" : "  ");
        snprintf(restart_line, sizeof(restart_line), "%s RESTART",
                 g_app.ui_system_index == 1 ? ">>" : "  ");
        snprintf(ota_line, sizeof(ota_line), "%s OTA UPDATE",
                 g_app.ui_system_index == 2 ? ">>" : "  ");
        snprintf(back_line, sizeof(back_line), "%s BACK",
                 g_app.ui_system_index == 3 ? ">>" : "  ");

        sh1106_fb_draw_text(fb, 0, 0, ver_line);
        sh1106_fb_draw_text(fb, 0, 14, sleep_line);
        sh1106_fb_draw_text(fb, 0, 28, restart_line);
        sh1106_fb_draw_text(fb, 0, 42, ota_line);
        sh1106_fb_draw_text(fb, 0, 56, back_line);
        return;
    }

    sh1106_fb_draw_text(fb, 0, 8, "MEASURE");
    sh1106_fb_draw_text(fb, 0, 24, measurement_is_running() ? "STATE: RUNNING" : "STATE: STOPPED");
    sh1106_fb_draw_text(fb, 0, 40, "BTN BACK");
    sh1106_fb_draw_text(fb, 0, 52, "USE MENU START/STOP");
}
