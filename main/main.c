#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include <driver/gpio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "init.h"
#include "json_builder.h"
#include "db.h"

#include "pwm_controller.h"
#include "driver_ina219.h"
#include "driver_ssd1306.h"
#include "driver_encoder.h"
#include "led_controller.h"

static const char *TAG = "MAIN";

typedef struct
{
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t ina_dev;
    ssd1306_t display;
    ina219_cal_t ina_cal;

    TaskHandle_t producer_task;
    TaskHandle_t display_task;
    TaskHandle_t encoder_task;
    SemaphoreHandle_t state_mtx;

    bool display_enabled;
    bool measurement_running;
    char wifi_qr_payload[128];
} app_state_t;

static app_state_t g_app = {0};

#define MAX_MEASUREMENTS_PER_CYCLE 10
uint32_t g_step_size_per_measurement = 0;

// --------------------------------------------------------------------------------

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_6
#define I2C_SCL_GPIO GPIO_NUM_7
#define I2C_FREQ_HZ 100000
#define WS2812_GPIO GPIO_NUM_10
#define OLED_ROTATE_180 false
#define OLED_COLUMN_OFFSET 2
#define ENC_DT_GPIO GPIO_NUM_2
#define ENC_CLK_GPIO GPIO_NUM_3
#define ENC_SW_GPIO GPIO_NUM_4

static void measurement_apply_state_locked(bool running);
static bool measurement_start_locked(void);
static bool measurement_stop_locked(void);

bool measurement_is_running(void)
{
    return g_app.measurement_running;
}

bool measurement_request(bool start)
{
    if (g_app.state_mtx == NULL)
    {
        ESP_LOGW(TAG, "measurement_request: state mutex not ready");
        return false;
    }

    bool changed = false;
    if (xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        changed = start ? measurement_start_locked() : measurement_stop_locked();
        xSemaphoreGive(g_app.state_mtx);
    }
    else
    {
        ESP_LOGW(TAG, "measurement_request: failed to take mutex");
    }
    return changed;
}

static void dummy_producer_task(void *arg);
static void producer_task(void *arg);
static void display_task(void *arg);
static void encoder_log_task(void *arg);

static void encoder_log_task(void *arg)
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
        }
        else if (ev.type == ENCODER_EVENT_CCW)
        {
            ESP_LOGI(TAG, "ENCODER: CCW pos=%ld t=%lu", (long)ev.position, (unsigned long)ev.timestamp_ms);
        }
        else
        {
            ESP_LOGI(TAG, "ENCODER: BUTTON pos=%ld t=%lu", (long)ev.position, (unsigned long)ev.timestamp_ms);

            bool start = !measurement_is_running();
            if (!measurement_request(start))
            {
                ESP_LOGW(TAG, "ENCODER: measurement %s request ignored", start ? "start" : "stop");
            }
        }
    }
}

static esp_err_t ensure_i2c_bus_ready(void)
{
    if (g_app.i2c_bus != NULL)
    {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false, // using external pullups on PCB
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &g_app.i2c_bus);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ensure_i2c_bus_ready: failed to init bus: %s", esp_err_to_name(ret));
    }

    return ret;
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

static uint32_t fnv1a32(const char *txt)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; txt && txt[i] != '\0'; i++)
    {
        h ^= (uint8_t)txt[i];
        h *= 16777619u;
    }
    return h;
}

static bool qr_finder_at(int x, int y, int ox, int oy)
{
    if (x < ox || x >= ox + 7 || y < oy || y >= oy + 7)
    {
        return false;
    }
    int dx = x - ox;
    int dy = y - oy;
    bool border = (dx == 0 || dx == 6 || dy == 0 || dy == 6);
    bool center = (dx >= 2 && dx <= 4 && dy >= 2 && dy <= 4);
    return border || center;
}

static void draw_qr_placeholder(uint8_t *fb, const char *payload)
{
    const int modules = 21;
    const int cell_px = 2;
    const int qr_px = modules * cell_px;
    const int x0 = (SSD1306_WIDTH - qr_px) / 2;
    const int y0 = 12;

    const uint32_t h = fnv1a32(payload);

    ssd1306_fb_draw_rect(fb, x0 - 2, y0 - 2, qr_px + 4, qr_px + 4, true, false);
    ssd1306_fb_draw_rect(fb, x0 - 2, y0 - 2, qr_px + 4, qr_px + 4, false, true);

    for (int y = 0; y < modules; y++)
    {
        for (int x = 0; x < modules; x++)
        {
            bool on = false;
            if (qr_finder_at(x, y, 0, 0) || qr_finder_at(x, y, modules - 7, 0) || qr_finder_at(x, y, 0, modules - 7))
            {
                on = true;
            }
            else
            {
                uint32_t mix = h ^ (uint32_t)(x * 0x45d9f3bu) ^ (uint32_t)(y * 0x27d4eb2du) ^ (uint32_t)((x * y + 17) * 2654435761u);
                on = (mix & 0x1u) != 0;
            }

            if (on)
            {
                ssd1306_fb_draw_rect(fb, x0 + (x * cell_px), y0 + (y * cell_px), cell_px, cell_px, true, true);
            }
        }
    }
}

static void render_display_frame(uint8_t *fb, uint32_t frame_idx)
{
    ssd1306_fb_clear(fb, false);

    if ((frame_idx % 2) == 0)
    {
        ssd1306_fb_draw_text(fb, 0, 0, "AP QR");
        draw_qr_placeholder(fb, g_app.wifi_qr_payload);
        ssd1306_fb_draw_text(fb, 0, 56, "SCAN TO JOIN");
    }
    else
    {
        ssd1306_fb_draw_text(fb, 0, 0, "SOLAR TRACER");
        ssd1306_fb_draw_text(fb, 0, 12, measurement_is_running() ? "MEAS: RUNNING" : "MEAS: IDLE");
        ssd1306_fb_draw_text(fb, 0, 24, "AP SSID:");
        ssd1306_fb_draw_text(fb, 0, 34, wifi_softap_ssid());
        ssd1306_fb_draw_text(fb, 0, 46, "WEB: 192.168.4.1");
    }
}

static void display_task(void *arg)
{
    (void)arg;

    uint8_t fb[SSD1306_FB_SIZE];
    uint32_t frame_idx = 0;

    while (g_app.display_enabled)
    {
        render_display_frame(fb, frame_idx++);
        esp_err_t ret = ssd1306_flush(&g_app.display, fb, sizeof(fb));
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "display_task: flush failed: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    vTaskDelete(NULL);
}

static void display_init_and_start(void)
{
    build_wifi_qr_payload(g_app.wifi_qr_payload, sizeof(g_app.wifi_qr_payload));

    if (ensure_i2c_bus_ready() != ESP_OK)
    {
        ESP_LOGW(TAG, "display_init_and_start: I2C bus unavailable, skipping OLED");
        return;
    }

    const uint8_t candidate_addrs[] = {
        SSD1306_DEFAULT_I2C_ADDR, // standard 7-bit 0x3C
        SSD1306_ALT_I2C_ADDR,     // alternate 7-bit 0x3D
    };

    esp_err_t ret = ESP_FAIL;
    uint8_t selected_addr = 0;
    for (size_t i = 0; i < sizeof(candidate_addrs) / sizeof(candidate_addrs[0]); i++)
    {
        ret = ssd1306_init_on_bus(&g_app.display, g_app.i2c_bus, candidate_addrs[i], 400000);
        if (ret == ESP_OK)
        {
            selected_addr = candidate_addrs[i];
            break;
        }
    }
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "display_init_and_start: SSD1306 init failed at 0x%02X/0x%02X: %s",
                 SSD1306_DEFAULT_I2C_ADDR, SSD1306_ALT_I2C_ADDR, esp_err_to_name(ret));
        return;
    }

    ret = ssd1306_set_rotation(&g_app.display, OLED_ROTATE_180);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "display_init_and_start: failed to set OLED rotation: %s", esp_err_to_name(ret));
    }

    ssd1306_set_column_offset(&g_app.display, OLED_COLUMN_OFFSET);

    g_app.display_enabled = true;
    if (xTaskCreate(display_task, "display", 4096, NULL, 4, &g_app.display_task) != pdPASS)
    {
        ESP_LOGE(TAG, "display_init_and_start: failed to create display task");
        g_app.display_enabled = false;
        return;
    }

    ESP_LOGI(TAG, "display_init_and_start: OLED started at 0x%02X (offset=%u), AP payload='%s'",
             selected_addr, OLED_COLUMN_OFFSET, g_app.wifi_qr_payload);
}

#define DUMMY_PRODUCER
#ifdef DUMMY_PRODUCER
#define PRODUCER dummy_producer_task
#else
#define PRODUCER producer_task
#endif

static void measurement_apply_state_locked(bool running)
{
    g_app.measurement_running = running;
    if (running)
    {
        led_set_color(WS2812_GPIO, (led_color_t){.r = 0, .g = 20, .b = 0});
    }
    else
    {
        led_clear(WS2812_GPIO);
    }
}

static bool measurement_start_locked(void)
{
    if (g_app.measurement_running)
        return false;

    db_reset();

    if (g_app.producer_task == NULL)
    {
        if (xTaskCreate(PRODUCER, "producer", 4096, NULL, 5, &g_app.producer_task) != pdPASS)
        {
            ESP_LOGE(TAG, "measurement_start_locked: failed to create producer task");
            g_app.producer_task = NULL;
            return false;
        }
    }

    measurement_apply_state_locked(true);
    ESP_LOGI(TAG, "Measurement cycle started");
    return true;
}

static bool measurement_stop_locked(void)
{
    if (!g_app.measurement_running)
        return false;

    if (g_app.producer_task)
    {
        db_reset();
        pwm_controller_set_duty(0);
        vTaskDelete(g_app.producer_task);
        g_app.producer_task = NULL;
    }

    measurement_apply_state_locked(false);
    ESP_LOGI(TAG, "Measurement cycle stopped");
    return true;
}

// --------------------------------------------------------------------------------
// The max_scale_current_mA is the maximum current the system can provide.
// It's determined in the notebook analysis and is a constant. It depends on various factors
// like the shunt resistor, the INA219 calibration, the power supply, etc.
// The desired_range_mA is the current setpoint the user wants to reach. It can be changed by the user.
static uint32_t calculate_step_size(float max_scale_current_mA, float desired_range_mA, uint32_t number_of_measurements, uint32_t pwm_resolution)
{
    if (max_scale_current_mA == 0)
        return 1; // at least 1 mA step

    // Calculate step size to reach max_current_mA in given number of measurements
    // Use integer math to avoid floating point in this simple calculation
    float step_f = ((float)pwm_resolution * desired_range_mA) / (max_scale_current_mA * (float)number_of_measurements);

    ESP_LOGI(TAG, "calculate_step_size: max_scale_current_mA=%.3f desired_range_mA=%.3f number_of_measurements=%d pwm_resolution=%d => step_f=%.3f",
             max_scale_current_mA, desired_range_mA, number_of_measurements, pwm_resolution, step_f);
    // Should be smaller than pwm_resolution
    if (step_f > (float)pwm_resolution)
        step_f = (float)pwm_resolution;
    // Round to nearest integer
    uint32_t step = (uint32_t)(step_f + 0.5f);

    return step;
}

// --------------------------------------------------------------------------------

static void dummy_producer_task(void *arg)
{
    int8_t duty = 0;
    pwm_controller_set_duty(duty);
    ESP_LOGI(TAG, "dummy_producer_task: Starting data production");

    float x_array[] = {22.464, 22.215, 21.942, 21.661, 21.365, 21.059, 20.731, 20.391, 20.021, 19.612, 19.164, 18.624, 17.823, 16.489, 14.571, 6.140, 0.060, 0.060, 0.059, 0.060};
    float y_array[] = {36.000, 72.000, 109.000, 147.000, 182.000, 219.000, 255.000, 292.000, 327.000, 364.000, 400.000, 436.000, 473.000, 510.000, 545.000, 579.000, 581.000, 582.000, 582.000, 583.000};

    int len = sizeof(x_array) / sizeof(x_array[0]);
    for (int i = 0; i < len; ++i)
    {

        ESP_LOGI(TAG, "dummy_producer_task: Set duty to %d%%", duty);

        const int max_retries = 0;
        int attempts = 0;
        bool success = false;

        while (!(success = db_add(x_array[i], y_array[i])) && attempts++ < max_retries)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (!success)
        {
            ESP_LOGW(TAG, "dummy_producer_task: db_add failed after %d attempts, dropping sample (x=%.3f,y=%.3f)", attempts, x_array[i], y_array[i]);
        }

        // simulate time taken for multiple measurements
        for (int j = 0; j < MAX_MEASUREMENTS_PER_CYCLE; j++)
        {
            vTaskDelay(pdMS_TO_TICKS(10 * MAX_MEASUREMENTS_PER_CYCLE));
        }

        ESP_LOGI(TAG, "dummy_producer_task: db_add succeeded (x=%.3f,y=%.3f) || attempts=%d", x_array[i], y_array[i], attempts);

        if (duty == 100)
        {
            duty += 10;
            duty %= 100;
        }
        else
        {
            duty += 10;
            if (duty > 100)
                duty = 100;
        }
        pwm_controller_set_duty(duty);
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    ESP_LOGI(TAG, "dummy_producer_task: Finished data production");

    // clear producer handle and turn LED off (under mutex if available)
    if (g_app.state_mtx && xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        g_app.producer_task = NULL;
        measurement_apply_state_locked(false);
        pwm_controller_set_duty(0);
        xSemaphoreGive(g_app.state_mtx);
    }
    else
    {
        // best-effort fallback
        g_app.producer_task = NULL;
        g_app.measurement_running = false;
        pwm_controller_set_duty(0);
    }

    ESP_LOGI(TAG, "dummy_producer_task: Deleting self");

    vTaskDelete(NULL);
}

static void producer_task(void *arg)
{
    // Calculate step size based on max current range, desired range and number of measurements
    float desired_range_mA = db_get_current_setpoint_mA();
    uint32_t pwm_res;
    // Don't check return value
    pwm_controller_get_resolution(&pwm_res);

    uint32_t stepsize = calculate_step_size(MAX_CURRENT_MA, desired_range_mA, DB_MAX_SAMPLES, pwm_res);
    ESP_LOGI(TAG, "producer_task: Calculated step size: %d || number of steps %d (desired_range=%.3f mA)", stepsize, DB_MAX_SAMPLES, desired_range_mA);

    uint32_t duty = 0;

    pwm_controller_set_duty_in_res_steps(duty);
    vTaskDelay(pdMS_TO_TICKS(250));

    ESP_LOGI(TAG, "producer_task: Starting data production");

    for (int i = 0; i < DB_MAX_SAMPLES; i++)
    {

        const int max_retries = 5;
        int attempts = 0;
        bool success = false;

        int32_t shunt_uV = 0;
        int32_t bus_mV = 0;
        int32_t current_mA = 0;
        int32_t power_mW = 0;

        int32_t shunt_uV_array[MAX_MEASUREMENTS_PER_CYCLE] = {0};
        int32_t bus_mV_array[MAX_MEASUREMENTS_PER_CYCLE] = {0};
        int32_t current_mA_array[MAX_MEASUREMENTS_PER_CYCLE] = {0};
        int32_t power_mW_array[MAX_MEASUREMENTS_PER_CYCLE] = {0};

        for (int j = 0; j < MAX_MEASUREMENTS_PER_CYCLE; j++)
        {

            if (ina219_get_shunt_voltage_uv(g_app.ina_dev, &shunt_uV) != ESP_OK)
            {
                ESP_LOGW(TAG, "driver_ina219: shunt read failed");
            }
            if (ina219_get_bus_voltage_mv(g_app.ina_dev, &bus_mV) != ESP_OK)
            {
                ESP_LOGW(TAG, "driver_ina219: bus read failed");
            }
            if (ina219_get_current_ma(g_app.ina_dev, &g_app.ina_cal, &current_mA) != ESP_OK)
            {
                ESP_LOGW(TAG, "driver_ina219: current read failed");
            }
            if (ina219_get_power_mw(g_app.ina_dev, &g_app.ina_cal, &power_mW) != ESP_OK)
            {
                ESP_LOGW(TAG, "driver_ina219: power read failed");
            }
            shunt_uV_array[j] = shunt_uV;
            bus_mV_array[j] = bus_mV;
            current_mA_array[j] = current_mA;
            power_mW_array[j] = power_mW;
            vTaskDelay(pdMS_TO_TICKS(1000 / MAX_MEASUREMENTS_PER_CYCLE));
        }

        // Simple averaging
        for (int k = 0; k < MAX_MEASUREMENTS_PER_CYCLE; k++)
        {
            shunt_uV += shunt_uV_array[k];
            bus_mV += bus_mV_array[k];
            current_mA += current_mA_array[k];
            power_mW += power_mW_array[k];
        }
        shunt_uV /= MAX_MEASUREMENTS_PER_CYCLE;
        bus_mV /= MAX_MEASUREMENTS_PER_CYCLE;
        current_mA /= MAX_MEASUREMENTS_PER_CYCLE;
        power_mW /= MAX_MEASUREMENTS_PER_CYCLE;

        ESP_LOGI(TAG, "VBUS=%d mV  VSHUNT=%d uV  I=%d mA  P=%d mW",
                 bus_mV, shunt_uV, current_mA, power_mW);

        /*
        Compute voltage across the load
        Assume shunt voltage is small compared to bus voltage
        Vload = Vbus - Vshunt
        And assume there isn't much voltage drop in wiring and connections
        */
        float v = (bus_mV - (shunt_uV / 1000.0f)) / 1000.0f; // convert to V
        float i = (float)current_mA;                         // in mA

        while (!(success = db_add(v, i)) && attempts++ < max_retries)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (!success)
        {
            ESP_LOGW(TAG, "producer_task: db_add failed after %d attempts, dropping sample (v=%.3f,i=%.3f)", attempts, v, i);
        }

        ESP_LOGI(TAG, "producer_task: db_add succeeded (v=%.3f,i=%.3f) || attempts=%d", v, i, attempts);

        // Should not happen, but wrap around just in case.
        duty = (duty + stepsize) % pwm_res;
        pwm_controller_set_duty_in_res_steps(duty);
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    ESP_LOGI(TAG, "producer_task: Finished data production");

    // clear producer handle and turn LED off (under mutex if available)
    if (g_app.state_mtx && xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        g_app.producer_task = NULL;
        measurement_apply_state_locked(false);
        pwm_controller_set_duty(0);
        xSemaphoreGive(g_app.state_mtx);
    }
    else
    {
        // best-effort fallback
        g_app.producer_task = NULL;
        g_app.measurement_running = false;
        pwm_controller_set_duty(0);
    }

    ESP_LOGI(TAG, "producer_task: Deleting self");

    vTaskDelete(NULL);
}

void app_main(void)
{

    g_app.state_mtx = xSemaphoreCreateMutex();
    if (g_app.state_mtx == NULL)
    {
        ESP_LOGE(TAG, "Failed to create state mutex");
        return;
    }

    if (PRODUCER == producer_task)
    {
        int res = pwm_controller_init(NULL);
        if (res != 0)
        {
            ESP_LOGE(TAG, "pwm_controller_init failed: %d", res);
            return;
        }

        res = ensure_i2c_bus_ready();
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(res));
            return;
        }

        res = ina219_init_on_bus(g_app.i2c_bus, &g_app.ina_dev, I2C_FREQ_HZ, INA219_ADDRESS_DEFAULT);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "ina219_init_on_bus failed: %s", esp_err_to_name(res));
            return;
        }

        res = ina219_calibrate_for_32V_10A(g_app.ina_dev, &g_app.ina_cal);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "ina219_calibrate_for_32V_10A failed");
            return;
        }

        ESP_LOGI(TAG, "driver_ina219: Calibration Done -- Current_Divider_mA=%d  Power_Multiplier_mW=%d  Current_LSB=%.6f A/bit CAL=0x%04X",
                 g_app.ina_cal.current_divider_mA, g_app.ina_cal.power_multiplier_mW, g_app.ina_cal.current_lsb, g_app.ina_cal.cal_value);
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
    else if (xTaskCreate(encoder_log_task, "encoder_log", 3072, NULL, 4, &g_app.encoder_task) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create encoder log task");
        g_app.encoder_task = NULL;
    }
}
