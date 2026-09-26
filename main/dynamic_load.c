#include "dynamic_load.h"

#include <math.h>

#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "app/app_hw.h"
#include "app/app_state.h"
#include "app/app_tasks.h"

#include "driver_ina219.h"
#include "pwm_controller.h"

static const char *TAG = "DYNLOAD";

// Short timeout: this is called from the encoder and display tasks, and must
// never block either of them for long. It must never be held while calling
// measurement_request(), which takes the same g_app.state_mtx.
#define DYNLOAD_LOCK_TIMEOUT_MS 20

static void dynamic_load_set_duty(uint32_t duty_steps);

void dynamic_load_adjust(int dir)
{
    if (!g_app.pwm_ready)
        return;

    if (g_app.measurement_running)
        return;

    if (g_app.state_mtx == NULL || xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(DYNLOAD_LOCK_TIMEOUT_MS)) != pdTRUE)
    {
        ESP_LOGW(TAG, "dynamic_load_adjust: failed to take state mutex, skipping");
        return;
    }

    if (dir > 0 && g_app.dynamic_measured_valid)
    {
        float near_limit_mW = LOAD_POWER_LIMIT_MW - LOAD_POWER_NEAR_MARGIN_MW;
        if (g_app.dynamic_power_mW >= near_limit_mW)
        {
            g_app.dynamic_power_limited = true;
            ESP_LOGW(TAG, "dynamic_load: power near limit (%.0f mW), blocking duty increase", g_app.dynamic_power_mW);
            xSemaphoreGive(g_app.state_mtx);
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
    xSemaphoreGive(g_app.state_mtx);
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
                if (g_app.state_mtx != NULL && xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(DYNLOAD_LOCK_TIMEOUT_MS)) == pdTRUE)
                {
                    uint32_t reduced_duty = (g_app.dynamic_duty_steps > DYNAMIC_LOAD_DUTY_STEP)
                                                ? (g_app.dynamic_duty_steps - DYNAMIC_LOAD_DUTY_STEP)
                                                : 0;
                    ESP_LOGW(TAG, "dynamic_load: power limit reached (%.0f mW), backing off duty %lu -> %lu",
                             g_app.dynamic_power_mW,
                             (unsigned long)g_app.dynamic_duty_steps,
                             (unsigned long)reduced_duty);
                    dynamic_load_set_duty(reduced_duty);
                    xSemaphoreGive(g_app.state_mtx);
                }
                else
                {
                    ESP_LOGW(TAG, "dynamic_load: failed to take state mutex, skipping power-limit backoff");
                }
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

bool dynamic_load_enter(void)
{
    if (g_app.measurement_running)
        return false;

    if (g_app.state_mtx == NULL || xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(DYNLOAD_LOCK_TIMEOUT_MS)) != pdTRUE)
    {
        ESP_LOGW(TAG, "dynamic_load_enter: failed to take state mutex, refusing");
        return false;
    }

    // Re-check under the lock: measurement_start_locked reads
    // dynamic_load_active under this same mutex, so this is the single
    // point of truth for both sides of that race.
    if (g_app.measurement_running)
    {
        xSemaphoreGive(g_app.state_mtx);
        return false;
    }

    g_app.dynamic_load_active = true;
    g_app.dynamic_power_limited = false;
    g_app.dynamic_power_mW = 0.0f;
    g_app.dynamic_last_adjust_tick = xTaskGetTickCount();
    g_app.dynamic_last_sample_tick = 0;

    dynamic_load_set_duty(0);
    xSemaphoreGive(g_app.state_mtx);

    // Sampling over I2C, done outside the lock so it never blocks
    // measurement_start_locked.
    dynamic_load_update_measured();
    app_display_mark_dirty();
    return true;
}

void dynamic_load_exit(void)
{
    if (g_app.state_mtx != NULL && xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(DYNLOAD_LOCK_TIMEOUT_MS)) == pdTRUE)
    {
        g_app.dynamic_load_active = false;
        g_app.dynamic_measured_valid = false;
        g_app.dynamic_measured_mA = 0.0f;
        g_app.dynamic_power_mW = 0.0f;
        g_app.dynamic_bus_mv = 0;
        g_app.dynamic_shunt_uv = 0;
        g_app.dynamic_power_limited = false;
        g_app.dynamic_duty_steps = 0;
        xSemaphoreGive(g_app.state_mtx);
    }
    else
    {
        ESP_LOGW(TAG, "dynamic_load_exit: failed to take state mutex, clearing state unlocked");
        g_app.dynamic_load_active = false;
        g_app.dynamic_measured_valid = false;
        g_app.dynamic_measured_mA = 0.0f;
        g_app.dynamic_power_mW = 0.0f;
        g_app.dynamic_bus_mv = 0;
        g_app.dynamic_shunt_uv = 0;
        g_app.dynamic_power_limited = false;
        g_app.dynamic_duty_steps = 0;
    }

    if (g_app.pwm_ready)
    {
        pwm_controller_set_duty(0);
    }
    app_display_mark_dirty();
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
