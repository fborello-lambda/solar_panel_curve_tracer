#include "ui.h"

#include <math.h>
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
#include "dynamic_load.h"
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

// The encoder has no detents, so overshooting past the first/last item is
// easy to do by feel. Clamp list navigation instead of wrapping: spinning
// fully one way reliably lands on (and stays on) the top or bottom item.
static int clamp_index(int value, int count)
{
    if (count <= 0)
    {
        return 0;
    }
    if (value < 0)
    {
        return 0;
    }
    if (value >= count)
    {
        return count - 1;
    }
    return value;
}

static void ui_mark_activity(void)
{
    g_app.last_activity_tick = xTaskGetTickCount();
}

// Returns true if the screen was off and this event should only wake it
// (the event itself is swallowed, not applied to navigation/selection).
static bool ui_wake_if_off(void)
{
    if (!g_app.display_off)
    {
        return false;
    }

    g_app.display_off = false;
    if (g_app.display.dev != NULL)
    {
        sh1106_set_display_on(&g_app.display, true);
    }
    app_display_mark_dirty();
    return true;
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

#define UI_MENU_MAX_ITEMS 4

static const char *const s_menu_items[HOME_SECTION_COUNT][UI_MENU_MAX_ITEMS] = {
    [HOME_SECTION_NETWORK] = {"SHOW WIFI QR", "SHOW AP IP QR", "SHOW REPO QR", "BACK"},
    [HOME_SECTION_MEASURE] = {"CURVE TRACER", "DYNAMIC LOAD", "BACK", NULL},
    [HOME_SECTION_SYSTEM] = {"OTA", "RESET", "DEEP SLEEP", "BACK"},
};

static const char *ui_menu_item_label(int home_index, int menu_index)
{
    if (home_index < 0 || home_index >= HOME_SECTION_COUNT ||
        menu_index < 0 || menu_index >= UI_MENU_MAX_ITEMS ||
        s_menu_items[home_index][menu_index] == NULL)
    {
        return "BACK";
    }
    return s_menu_items[home_index][menu_index];
}

static int ui_menu_item_count(int home_index)
{
    if (home_index < 0 || home_index >= HOME_SECTION_COUNT)
    {
        return 0;
    }
    int count = 0;
    while (count < UI_MENU_MAX_ITEMS && s_menu_items[home_index][count] != NULL)
    {
        count++;
    }
    return count;
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

    g_app.last_activity_tick = xTaskGetTickCount();
    g_app.display_off = false;

    app_display_mark_dirty();
}

void ui_on_rotate(int dir)
{
    ui_mark_activity();
    if (ui_wake_if_off())
    {
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_HOME)
    {
        g_app.ui_home_index = clamp_index(g_app.ui_home_index + dir, HOME_SECTION_COUNT);
        app_display_mark_dirty();
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_MENU)
    {
        g_app.ui_menu_index = clamp_index(g_app.ui_menu_index + dir, ui_menu_item_count(g_app.ui_home_index));
        app_display_mark_dirty();
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_ACTION_MEASURE)
    {
        g_app.ui_measure_index = clamp_index(g_app.ui_measure_index + dir, 3);
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
    ui_mark_activity();
    if (ui_wake_if_off())
    {
        return;
    }

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
            if (g_app.ui_menu_index == 0)
            {
                g_app.ui_qr_kind = UI_QR_WIFI;
            }
            else if (g_app.ui_menu_index == 1)
            {
                g_app.ui_qr_kind = UI_QR_AP_IP;
            }
            else
            {
                g_app.ui_qr_kind = UI_QR_REPO;
            }
            ui_set_screen(UI_SCREEN_ACTION_QR);
            return;
        }

        if (g_app.ui_home_index == HOME_SECTION_SYSTEM)
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
            if (dynamic_load_enter())
            {
                ui_set_screen(UI_SCREEN_ACTION_DYNAMIC_LOAD);
            }
            else
            {
                ESP_LOGW(TAG, "UI: dynamic load entry refused, staying on menu");
            }
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
    bool drawn;
} qr_render_ctx_t;

// Largest QR (in modules) that still leaves a lit margin on the 64 px tall
// panel at the requested module size.
#define QR_MAX_PX (SH1106_HEIGHT - 4)

static void qr_draw_to_oled_cb(esp_qrcode_handle_t qrcode, void *user_data)
{
    qr_render_ctx_t *ctx = (qr_render_ctx_t *)user_data;
    if (!ctx || !ctx->fb || !qrcode)
    {
        return;
    }

    int size = esp_qrcode_get_size(qrcode);
    int cell_px = ctx->cell_px;
    int qr_px = size * cell_px;
    if (qr_px > QR_MAX_PX)
    {
        // Too big at this module size: leave drawn=false so the caller
        // retries with lower error correction or smaller modules.
        return;
    }

    // Phone scanners expect dark modules on a light background, so light
    // the whole panel (it doubles as the quiet zone) and draw the dark
    // modules as unlit pixels.
    sh1106_fb_clear(ctx->fb, true);

    int x0 = (SH1106_WIDTH - qr_px) / 2;
    int y0 = ((SH1106_HEIGHT - qr_px) / 2) + OLED_QR_Y_OFFSET;
    if (y0 + qr_px > SH1106_HEIGHT)
    {
        y0 = SH1106_HEIGHT - qr_px;
    }

    for (int y = 0; y < size; y++)
    {
        for (int x = 0; x < size; x++)
        {
            if (esp_qrcode_get_module(qrcode, x, y))
            {
                sh1106_fb_draw_rect(ctx->fb,
                                    x0 + x * cell_px,
                                    y0 + y * cell_px,
                                    cell_px,
                                    cell_px,
                                    true,
                                    false);
            }
        }
    }
    ctx->drawn = true;
}

static void draw_real_qr_to_fb(uint8_t *fb, const char *payload)
{
    if (!fb || !payload || payload[0] == '\0')
    {
        return;
    }

    // Prefer 2 px modules with the strongest error correction that fits,
    // so a flickering OLED or a slightly blurry camera frame still decodes.
    // Long payloads (the GitHub guide URL) fall back to 1 px modules.
    static const int ecc_levels[] = {
        ESP_QRCODE_ECC_HIGH,
        ESP_QRCODE_ECC_QUART,
        ESP_QRCODE_ECC_MED,
        ESP_QRCODE_ECC_LOW,
    };
    static const int cell_sizes[] = {2, 1};

    qr_render_ctx_t ctx = {.fb = fb};
    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    cfg.display_func_with_cb = qr_draw_to_oled_cb;
    cfg.user_data = &ctx;
    cfg.max_qrcode_version = 10;

    for (size_t c = 0; c < sizeof(cell_sizes) / sizeof(cell_sizes[0]) && !ctx.drawn; c++)
    {
        ctx.cell_px = cell_sizes[c];
        for (size_t e = 0; e < sizeof(ecc_levels) / sizeof(ecc_levels[0]) && !ctx.drawn; e++)
        {
            cfg.qrcode_ecc_level = ecc_levels[e];
            esp_qrcode_generate(&cfg, payload);
        }
    }

    if (!ctx.drawn)
    {
        sh1106_fb_clear(fb, false);
        sh1106_fb_draw_text(fb, 0, 24, "QR ERROR");
    }
}

// Draws a "N/total" position hint at the top-right, on the same row as a
// header drawn with sh1106_fb_draw_text at y=8.
static void draw_position_hint(uint8_t *fb, int index, int count)
{
    char hint[16] = {0};
    snprintf(hint, sizeof(hint), "%d/%d", index + 1, count);
    int x = SH1106_WIDTH - (int)strlen(hint) * 6;
    if (x < 0)
    {
        x = 0;
    }
    sh1106_fb_draw_text(fb, x, 8, hint);
}

static void draw_list_row(uint8_t *fb, int y, bool selected, const char *label)
{
    if (selected)
    {
        sh1106_fb_draw_text_inverted(fb, 0, y, label);
    }
    else
    {
        sh1106_fb_draw_text(fb, 0, y, label);
    }
}

void ui_render_display_frame(uint8_t *fb)
{
    sh1106_fb_clear(fb, false);

    if (g_app.ui_screen == UI_SCREEN_HOME)
    {
        sh1106_fb_draw_text(fb, 0, 8, "HOME");
        draw_position_hint(fb, g_app.ui_home_index, HOME_SECTION_COUNT);
        draw_list_row(fb, 22, g_app.ui_home_index == HOME_SECTION_NETWORK, ui_home_title(HOME_SECTION_NETWORK));
        draw_list_row(fb, 36, g_app.ui_home_index == HOME_SECTION_MEASURE, ui_home_title(HOME_SECTION_MEASURE));
        draw_list_row(fb, 50, g_app.ui_home_index == HOME_SECTION_SYSTEM, ui_home_title(HOME_SECTION_SYSTEM));
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_MENU)
    {
        int item_count = ui_menu_item_count(g_app.ui_home_index);

        int section_y = (item_count > 3) ? 16 : 18;
        int line0_y = (item_count > 3) ? 24 : 30;
        int line1_y = (item_count > 3) ? 34 : 42;
        int line2_y = (item_count > 3) ? 44 : 54;
        int line3_y = 56;

        sh1106_fb_draw_text(fb, 0, 8, "MENU");
        draw_position_hint(fb, g_app.ui_menu_index, item_count);
        sh1106_fb_draw_text(fb, 0, section_y, ui_home_title(g_app.ui_home_index));
        draw_list_row(fb, line0_y, g_app.ui_menu_index == 0, ui_menu_item_label(g_app.ui_home_index, 0));
        draw_list_row(fb, line1_y, g_app.ui_menu_index == 1, ui_menu_item_label(g_app.ui_home_index, 1));
        if (item_count > 2)
        {
            draw_list_row(fb, line2_y, g_app.ui_menu_index == 2, ui_menu_item_label(g_app.ui_home_index, 2));
        }
        if (item_count > 3)
        {
            draw_list_row(fb, line3_y, g_app.ui_menu_index == 3, ui_menu_item_label(g_app.ui_home_index, 3));
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
        else if (g_app.ui_qr_kind == UI_QR_REPO)
            payload = "https://github.com/fborello-lambda/solar_panel_curve_tracer/blob/main/docs/quick_guide.md";
        else
            payload = g_app.wifi_qr_payload;
        draw_real_qr_to_fb(fb, payload);
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_ACTION_DYNAMIC_LOAD)
    {
        char load_bar[24] = {0};
        char current_line[24] = {0};
        char power_line[24] = {0};
        char vbus_line[24] = {0};
        char status_line[24] = {0};

        uint32_t pwm_res_disp = 0;
        pwm_controller_get_resolution(&pwm_res_disp);
        float duty_pct = (pwm_res_disp > 0) ? ((float)g_app.dynamic_duty_steps * 100.0f / (float)pwm_res_disp) : 0.0f;

        // Build load bar scaled to DYNAMIC_LOAD_DUTY_MAX_PERCENT
        float duty_rel = (DYNAMIC_LOAD_DUTY_MAX_PERCENT > 0) ? (duty_pct / DYNAMIC_LOAD_DUTY_MAX_PERCENT) : 0.0f;
        int filled = (int)roundf(duty_rel * 8.0f);
        if (filled > 8) filled = 8;
        char bar[9] = {0};
        for (int i = 0; i < 8; i++)
            bar[i] = (i < filled) ? '>' : '.';
        int duty_int = (int)roundf(duty_pct);
        snprintf(load_bar, sizeof(load_bar), "|%s| %d/%d%%", bar, duty_int, DYNAMIC_LOAD_DUTY_MAX_PERCENT);

        if (g_app.dynamic_measured_valid)
        {
            snprintf(current_line, sizeof(current_line), "I   : %.1f mA", g_app.dynamic_measured_mA);
            snprintf(power_line, sizeof(power_line), "PWR : %.0f mW", g_app.dynamic_power_mW);
            snprintf(vbus_line, sizeof(vbus_line), "VBUS: %ld mV", (long)g_app.dynamic_bus_mv);
        }
        else
        {
            snprintf(current_line, sizeof(current_line), "I   : N/A");
            snprintf(power_line, sizeof(power_line), "PWR : N/A");
            snprintf(vbus_line, sizeof(vbus_line), "VBUS: N/A");
        }

        if (g_app.dynamic_power_limited)
            snprintf(status_line, sizeof(status_line), "LIMIT: MAX %.0fW", LOAD_POWER_LIMIT_MW / 1000.0f);
        else
            snprintf(status_line, sizeof(status_line), "STATUS: OK");

        sh1106_fb_draw_text(fb, 0, 8, "DYNAMIC LOAD");
        sh1106_fb_draw_text(fb, 0, 18, load_bar);
        sh1106_fb_draw_text(fb, 0, 28, current_line);
        sh1106_fb_draw_text(fb, 0, 38, power_line);
        sh1106_fb_draw_text(fb, 0, 48, vbus_line);
        sh1106_fb_draw_text(fb, 0, 56, status_line);
        return;
    }

    if (g_app.ui_screen == UI_SCREEN_ACTION_MEASURE)
    {
        char mode_line[24] = {0};
        snprintf(mode_line, sizeof(mode_line), "MODE: %s", measurement_get_producer_mode_label());

        sh1106_fb_draw_text(fb, 0, 8, "CURVE TRACER");
        draw_position_hint(fb, g_app.ui_measure_index, 3);
        sh1106_fb_draw_text(fb, 0, 20, measurement_is_running() ? "STATE: RUNNING" : "STATE: STOPPED");
        draw_list_row(fb, 32, g_app.ui_measure_index == 0, measurement_is_running() ? "STOP TRACE" : "START TRACE");
        draw_list_row(fb, 44, g_app.ui_measure_index == 1, mode_line);
        draw_list_row(fb, 56, g_app.ui_measure_index == 2, "BACK");
        return;
    }

}
