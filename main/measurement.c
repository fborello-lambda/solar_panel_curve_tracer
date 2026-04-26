#include "measurement.h"

#include <math.h>
#include <stdio.h>

#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "app/app_hw.h"
#include "app/app_state.h"
#include "app/app_tasks.h"

#include "db.h"
#include "driver_ina219.h"
#include "led_controller.h"
#include "pwm_controller.h"

static const char *TAG = "MEASURE";

static curve_producer_mode_t s_producer_mode = CURVE_PRODUCER_REAL;

static void measurement_apply_state_locked(bool running);
static bool measurement_start_locked(void);
static bool measurement_stop_locked(void);

static void dynamic_load_set_duty(uint32_t duty_steps);

static uint32_t calculate_step_size(float max_scale_current_mA, float desired_range_mA, uint32_t number_of_measurements, uint32_t pwm_resolution);

static void dummy_producer_task(void *arg);
static void producer_task(void *arg);

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

void measurement_set_producer_mode(curve_producer_mode_t mode)
{
    s_producer_mode = mode;
}

curve_producer_mode_t measurement_get_producer_mode(void)
{
    return s_producer_mode;
}

const char *measurement_get_producer_mode_label(void)
{
    return (s_producer_mode == CURVE_PRODUCER_DUMMY) ? "DEMO" : "REAL";
}

void dynamic_load_adjust(int dir)
{
    if (!g_app.pwm_ready)
        return;

    if (dir > 0 && g_app.dynamic_measured_valid)
    {
        float near_limit_mW = LOAD_POWER_LIMIT_MW - LOAD_POWER_NEAR_MARGIN_MW;
        if (g_app.dynamic_power_mW >= near_limit_mW)
        {
            g_app.dynamic_power_limited = true;
            ESP_LOGW(TAG, "dynamic_load: power near limit (%.0f mW), blocking duty increase", g_app.dynamic_power_mW);
            return;
        }
    }
    if (dir < 0)
    {
        g_app.dynamic_power_limited = false;
    }

    int32_t new_duty = (int32_t)g_app.dynamic_duty_steps + (dir * DYNAMIC_LOAD_DUTY_STEP);
    if (new_duty < 0)
        new_duty = 0;

    dynamic_load_set_duty((uint32_t)new_duty);
}

void dynamic_load_update_measured(void)
{
    if (!g_app.ina_ready)
    {
        g_app.dynamic_measured_valid = false;
        return;
    }

    TickType_t now = xTaskGetTickCount();
    if ((now - g_app.dynamic_last_sample_tick) < pdMS_TO_TICKS(DYNAMIC_LOAD_UPDATE_MS))
    {
        return;
    }

    if ((now - g_app.dynamic_last_adjust_tick) < pdMS_TO_TICKS(DYNAMIC_LOAD_SETTLE_MS))
    {
        return;
    }

    g_app.dynamic_last_sample_tick = now;

    int32_t sum_mA = 0;
    int32_t sum_bus_mv = 0;
    int32_t sum_shunt_uv = 0;
    int valid = 0;
    for (int n = 0; n < DYNAMIC_LOAD_SAMPLE_COUNT; n++)
    {
        int32_t raw_mA = 0, bus_mv = 0, shunt_uv = 0;
        bool ok = ina219_get_current_ma(g_app.ina_dev, &g_app.ina_cal, &raw_mA) == ESP_OK;
        ok &= ina219_get_bus_voltage_mv(g_app.ina_dev, &bus_mv) == ESP_OK;
        ok &= ina219_get_shunt_voltage_uv(g_app.ina_dev, &shunt_uv) == ESP_OK;
        if (ok)
        {
            sum_mA += raw_mA;
            sum_bus_mv += bus_mv;
            sum_shunt_uv += shunt_uv;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (valid > 0)
    {
        float avg_signed_mA = (float)sum_mA / (float)valid;
        float avg_mA = fabsf(avg_signed_mA);
        float avg_bus_mV = (float)sum_bus_mv / (float)valid;

        g_app.dynamic_measured_mA = (avg_mA < 3.0f) ? 0.0f : avg_mA;
        g_app.dynamic_bus_mv = sum_bus_mv / valid;
        g_app.dynamic_shunt_uv = sum_shunt_uv / valid;
        g_app.dynamic_power_mW = (g_app.dynamic_measured_mA * avg_bus_mV) / 1000.0f;
        g_app.dynamic_measured_valid = true;

        if (g_app.dynamic_power_mW >= LOAD_POWER_LIMIT_MW)
        {
            g_app.dynamic_power_limited = true;
            if (g_app.dynamic_duty_steps > 0)
            {
                uint32_t reduced_duty = (g_app.dynamic_duty_steps > DYNAMIC_LOAD_DUTY_STEP)
                                            ? (g_app.dynamic_duty_steps - DYNAMIC_LOAD_DUTY_STEP)
                                            : 0;
                ESP_LOGW(TAG, "dynamic_load: power limit reached (%.0f mW), backing off duty %lu -> %lu",
                         g_app.dynamic_power_mW,
                         (unsigned long)g_app.dynamic_duty_steps,
                         (unsigned long)reduced_duty);
                dynamic_load_set_duty(reduced_duty);
            }
        }
        else if (g_app.dynamic_power_mW < (LOAD_POWER_LIMIT_MW - LOAD_POWER_NEAR_MARGIN_MW))
        {
            g_app.dynamic_power_limited = false;
        }
    }
    else
    {
        g_app.dynamic_measured_valid = false;
        g_app.dynamic_power_mW = 0.0f;
    }
}

void dynamic_load_enter(void)
{
    g_app.dynamic_load_active = true;
    g_app.dynamic_power_limited = false;
    g_app.dynamic_power_mW = 0.0f;
    g_app.dynamic_last_adjust_tick = xTaskGetTickCount();
    g_app.dynamic_last_sample_tick = 0;

    dynamic_load_set_duty(0);
    dynamic_load_update_measured();
    app_display_mark_dirty();
}

void dynamic_load_exit(void)
{
    g_app.dynamic_load_active = false;
    g_app.dynamic_measured_valid = false;
    g_app.dynamic_measured_mA = 0.0f;
    g_app.dynamic_power_mW = 0.0f;
    g_app.dynamic_bus_mv = 0;
    g_app.dynamic_shunt_uv = 0;
    g_app.dynamic_power_limited = false;
    g_app.dynamic_duty_steps = 0;
    if (g_app.pwm_ready)
    {
        pwm_controller_set_duty(0);
    }
    app_display_mark_dirty();
}

bool measurement_init_load_control_hw(bool strict_mode)
{
    g_app.pwm_ready = false;
    g_app.ina_ready = false;

    int pwm_ret = pwm_controller_init(NULL);
    if (pwm_ret != 0)
    {
        ESP_LOGE(TAG, "pwm_controller_init failed: %d", pwm_ret);
        return !strict_mode;
    }
    g_app.pwm_ready = true;

    esp_err_t ret = app_hw_ensure_i2c_bus_ready();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return !strict_mode;
    }

    ret = ina219_init_on_bus(g_app.i2c_bus, &g_app.ina_dev, I2C_FREQ_HZ, INA219_ADDRESS_DEFAULT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ina219_init_on_bus failed: %s", esp_err_to_name(ret));
        return !strict_mode;
    }

    ret = ina219_calibrate_for_32V_10A(g_app.ina_dev, &g_app.ina_cal);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ina219_calibrate_for_32V_10A failed");
        return !strict_mode;
    }

    ESP_LOGI(TAG, "driver_ina219: Calibration Done -- Current_Divider_mA=%d  Power_Multiplier_mW=%d  Current_LSB=%.6f A/bit CAL=0x%04X",
             g_app.ina_cal.current_divider_mA, g_app.ina_cal.power_multiplier_mW, g_app.ina_cal.current_lsb, g_app.ina_cal.cal_value);
    g_app.ina_ready = true;

    return true;
}

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

    app_display_mark_dirty();
}

static bool measurement_start_locked(void)
{
    if (g_app.measurement_running)
        return false;

    db_reset();

    TaskFunction_t producer_fn = (s_producer_mode == CURVE_PRODUCER_DUMMY) ? dummy_producer_task : producer_task;
    const char *producer_name = (s_producer_mode == CURVE_PRODUCER_DUMMY) ? "producer_demo" : "producer";

    if (g_app.producer_task == NULL)
    {
        if (xTaskCreate(producer_fn, producer_name, 4096, NULL, 5, &g_app.producer_task) != pdPASS)
        {
            ESP_LOGE(TAG, "measurement_start_locked: failed to create %s task", producer_name);
            g_app.producer_task = NULL;
            return false;
        }
    }

    measurement_apply_state_locked(true);
    ESP_LOGI(TAG, "Measurement cycle started (mode=%s)", measurement_get_producer_mode_label());
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

static void dynamic_load_set_duty(uint32_t duty_steps)
{
    if (!g_app.pwm_ready)
    {
        g_app.dynamic_duty_steps = 0;
        g_app.dynamic_last_adjust_tick = xTaskGetTickCount();
        return;
    }

    uint32_t pwm_res = 0;
    if (pwm_controller_get_resolution(&pwm_res) != 0 || pwm_res == 0)
    {
        ESP_LOGW(TAG, "dynamic_load: failed to read PWM resolution");
        return;
    }

    uint32_t max_duty = (uint32_t)((DYNAMIC_LOAD_DUTY_MAX_PERCENT / 100.0f) * (float)pwm_res);
    if (duty_steps > max_duty)
        duty_steps = max_duty;

    if (pwm_controller_set_duty_in_res_steps(duty_steps) != 0)
    {
        ESP_LOGW(TAG, "dynamic_load: failed to apply PWM duty=%lu", (unsigned long)duty_steps);
        return;
    }

    g_app.dynamic_duty_steps = duty_steps;
    g_app.dynamic_last_adjust_tick = xTaskGetTickCount();
}

// The max_scale_current_mA is the maximum current the system can provide.
// It's determined in the notebook analysis and is a constant. It depends on various factors
// like the shunt resistor, the INA219 calibration, the power supply, etc.
// The desired_range_mA is the current setpoint the user wants to reach. It can be changed by the user.
static uint32_t calculate_step_size(float max_scale_current_mA, float desired_range_mA, uint32_t number_of_measurements, uint32_t pwm_resolution)
{
    if (max_scale_current_mA == 0)
        return 1; // at least 1 mA step

    // Calculate step size to reach max_current_mA in given number of measurements
    float step_f = ((float)pwm_resolution * desired_range_mA) / (max_scale_current_mA * (float)(number_of_measurements - 1));

    ESP_LOGI(TAG, "calculate_step_size: max_scale_current_mA=%.3f desired_range_mA=%.3f number_of_measurements=%d pwm_resolution=%d => step_f=%.3f",
             max_scale_current_mA, desired_range_mA, number_of_measurements, pwm_resolution, step_f);
    if (step_f > (float)pwm_resolution)
        step_f = (float)pwm_resolution;

    return (uint32_t)(step_f + 0.5f);
}

static void dummy_producer_task(void *arg)
{
    (void)arg;

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

    if (g_app.state_mtx && xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        g_app.producer_task = NULL;
        measurement_apply_state_locked(false);
        pwm_controller_set_duty(0);
        xSemaphoreGive(g_app.state_mtx);
    }
    else
    {
        g_app.producer_task = NULL;
        g_app.measurement_running = false;
        pwm_controller_set_duty(0);
    }

    ESP_LOGI(TAG, "dummy_producer_task: Deleting self");

    vTaskDelete(NULL);
}

static void producer_task(void *arg)
{
    (void)arg;

    float desired_range_mA = db_get_current_setpoint_mA();
    uint32_t pwm_res;
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

        shunt_uV = 0;
        bus_mV = 0;
        current_mA = 0;
        power_mW = 0;
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

        int32_t abs_power_mW = (power_mW < 0) ? -power_mW : power_mW;
        float near_limit_mW = LOAD_POWER_LIMIT_MW - LOAD_POWER_NEAR_MARGIN_MW;
        if ((float)abs_power_mW >= near_limit_mW)
        {
            ESP_LOGW(TAG, "producer_task: power near limit (%ld mW), stopping sweep to protect load",
                     (long)abs_power_mW);
            break;
        }

        float v = (bus_mV - (shunt_uV / 1000.0f)) / 1000.0f;
        float i = (float)current_mA;

        while (!(success = db_add(v, i)) && attempts++ < max_retries)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (!success)
        {
            ESP_LOGW(TAG, "producer_task: db_add failed after %d attempts, dropping sample (v=%.3f,i=%.3f)", attempts, v, i);
        }

        ESP_LOGI(TAG, "producer_task: db_add succeeded (v=%.3f,i=%.3f) || attempts=%d", v, i, attempts);

        duty += stepsize;
        if (duty > pwm_res)
            duty = pwm_res;

        pwm_controller_set_duty_in_res_steps(duty);
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    ESP_LOGI(TAG, "producer_task: Finished data production");

    if (g_app.state_mtx && xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        g_app.producer_task = NULL;
        measurement_apply_state_locked(false);
        pwm_controller_set_duty(0);
        xSemaphoreGive(g_app.state_mtx);
    }
    else
    {
        g_app.producer_task = NULL;
        g_app.measurement_running = false;
        pwm_controller_set_duty(0);
    }

    ESP_LOGI(TAG, "producer_task: Deleting self");

    vTaskDelete(NULL);
}
