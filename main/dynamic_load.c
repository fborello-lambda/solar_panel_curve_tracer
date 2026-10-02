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
#include "measurement.h"
#include "pwm_controller.h"
#include "sweep_plan.h"

static const char *TAG = "DYNLOAD";

// Short timeout: this is called from the encoder and display tasks, and must
// never block either of them for long. It must never be held while calling
// measurement_request(), which takes the same g_app.state_mtx.
#define DYNLOAD_LOCK_TIMEOUT_MS 20

// Range probe timing, same as the sweep's auto-range: a long drain at duty 0
// so the first reading is the true open-circuit voltage, then a settle per
// (ascending) probe step.
#define DYNAMIC_RANGE_DRAIN_MS 1000
#define DYNAMIC_RANGE_SETTLE_MS 250

static void dynamic_load_set_duty(uint32_t duty_steps);

static uint32_t dynamic_load_max_duty(uint32_t pwm_res)
{
    return (uint32_t)((DYNAMIC_LOAD_DUTY_MAX_PERCENT / 100.0f) * (float)pwm_res);
}

static uint32_t dynamic_pos_duty(int pos)
{
    return (uint32_t)(((uint64_t)g_app.dynamic_full_duty * (uint64_t)pos) / DYNAMIC_LOAD_POSITIONS);
}

// Same correction as the sweep: raw - offset - k * V (see measurement.c).
static float dynamic_corrected_mA(float bus_mV, float raw_mA)
{
    return raw_mA - measurement_ina_offset_mA() - measurement_ina_k_mA_per_V() * (bus_mV / 1000.0f);
}

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
        float near_limit_mW = DYNAMIC_LOAD_POWER_LIMIT_MW - LOAD_POWER_NEAR_MARGIN_MW;
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

    if (g_app.dynamic_range_state != DYNAMIC_RANGE_IDLE)
    {
        xSemaphoreGive(g_app.state_mtx);
        return;
    }

    int pos = g_app.dynamic_pos + ((dir > 0) ? 1 : (dir < 0) ? -1 : 0);
    if (pos < 0)
        pos = 0;
    if (pos > DYNAMIC_LOAD_POSITIONS)
        pos = DYNAMIC_LOAD_POSITIONS;
    g_app.dynamic_pos = pos;

    dynamic_load_set_duty(dynamic_pos_duty(pos));
    xSemaphoreGive(g_app.state_mtx);
}

void dynamic_load_request_range(void)
{
    if (g_app.state_mtx == NULL || xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(DYNLOAD_LOCK_TIMEOUT_MS)) != pdTRUE)
        return;
    if (g_app.dynamic_load_active && g_app.dynamic_range_state == DYNAMIC_RANGE_IDLE)
        g_app.dynamic_range_state = DYNAMIC_RANGE_REQUESTED;
    xSemaphoreGive(g_app.state_mtx);
    app_display_mark_dirty();
}

// Commands one probe duty. Returns false if the operator left the screen
// (or left and re-entered it) in the meantime: dynamic_load_exit() already
// turned the load off and the probe must not turn it back on.
static bool range_set_duty(uint32_t session, uint32_t duty)
{
    if (g_app.state_mtx == NULL || xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(DYNLOAD_LOCK_TIMEOUT_MS)) != pdTRUE)
        return false;
    bool active = g_app.dynamic_load_active && g_app.dynamic_session == session;
    if (active)
        dynamic_load_set_duty(duty);
    xSemaphoreGive(g_app.state_mtx);
    return active;
}

// Range probe for whatever is connected (a panel in any light, a bench
// supply, ...): the sweep's auto-range doubling search (sweep_range_* in
// components/sweep_plan) finds the duty where the source collapses, i.e.
// reaches its short-circuit current. That duty becomes the top knob
// position. A stiff source that never collapses tops out at the duty cap,
// and the probe also stops short of DYNAMIC_LOAD_POWER_LIMIT_MW.
static void dynamic_load_run_range(void)
{
    uint32_t pwm_res = 0;
    if (!g_app.ina_ready || pwm_controller_get_resolution(&pwm_res) != 0 || pwm_res == 0)
        return;
    uint32_t cap = dynamic_load_max_duty(pwm_res);

    uint32_t session = g_app.dynamic_session;
    sweep_range_t sr;
    sweep_range_begin(&sr, pwm_res, DYNAMIC_LOAD_POSITIONS);

    if (!range_set_duty(session, 0))
        return;
    vTaskDelay(pdMS_TO_TICKS(DYNAMIC_RANGE_DRAIN_MS));

    float bus_mV = 0.0f, raw_mA = 0.0f;
    if (!measurement_sample(&bus_mV, &raw_mA))
        return;
    if (!sweep_range_on_voc(&sr, bus_mV))
    {
        ESP_LOGW(TAG, "range: %.0f mV with no load, nothing connected (or too little light), scale unchanged",
                 (double)bus_mV);
        return;
    }

    uint32_t full = cap;
    float full_mA = 0.0f; // unknown unless the source collapsed below the cap
    uint32_t prev_duty = 0;
    float prev_mA = dynamic_corrected_mA(bus_mV, raw_mA);
    for (;;)
    {
        uint32_t duty = sr.duty;
        if (duty > cap)
        {
            ESP_LOGI(TAG, "range: no collapse up to the %d%% duty cap", DYNAMIC_LOAD_DUTY_MAX_PERCENT);
            break;
        }
        if (!range_set_duty(session, duty))
            return;
        vTaskDelay(pdMS_TO_TICKS(DYNAMIC_RANGE_SETTLE_MS));
        if (!measurement_sample(&bus_mV, &raw_mA))
            break;

        float i_mA = dynamic_corrected_mA(bus_mV, raw_mA);
        float p_mW = (i_mA > 0.0f ? i_mA : 0.0f) * bus_mV / 1000.0f;
        ESP_LOGI(TAG, "range: probe duty=%lu -> V=%.0f mV I=%.1f mA", (unsigned long)duty, (double)bus_mV,
                 (double)i_mA);

        if (p_mW >= DYNAMIC_LOAD_POWER_LIMIT_MW - LOAD_POWER_NEAR_MARGIN_MW)
        {
            ESP_LOGI(TAG, "range: %.0f mW at duty %lu, top limited by power", (double)p_mW, (unsigned long)duty);
            full = prev_duty;
            full_mA = prev_mA;
            break;
        }
        if (sweep_range_on_probe(&sr, bus_mV, i_mA))
        {
            // sr.top is pushed out to the collapsing probe's duty (up to 2x
            // the knee with a doubling search) so a sweep always ends at the
            // collapse; the knob wants the knee estimate itself instead, so
            // its top positions aren't all the same Isc.
            uint32_t top = sr.knee * SWEEP_KNEE_HEADROOM_PERCENT / 100;
            if (sr.collapsed && top <= cap)
            {
                full = top;
                full_mA = sr.isc_mA;
            }
            break;
        }
        prev_duty = duty;
        prev_mA = i_mA;
    }

    if (full < DYNAMIC_LOAD_POSITIONS)
        full = DYNAMIC_LOAD_POSITIONS;

    if (g_app.state_mtx == NULL || xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(DYNLOAD_LOCK_TIMEOUT_MS)) != pdTRUE)
        return;
    if (g_app.dynamic_load_active && g_app.dynamic_session == session)
    {
        g_app.dynamic_full_duty = full;
        g_app.dynamic_full_mA = full_mA;
        g_app.dynamic_pos = 0;
        g_app.dynamic_power_limited = false;
        dynamic_load_set_duty(0);
        ESP_LOGI(TAG, "range: top position = duty %lu (%.1f mA)", (unsigned long)full, (double)full_mA);
    }
    xSemaphoreGive(g_app.state_mtx);
}

void dynamic_load_update_measured(void)
{
    if (!g_app.ina_ready)
    {
        g_app.dynamic_measured_valid = false;
        return;
    }

    // A range request is first only acknowledged, so this display_task pass
    // renders "RANGING"; the (seconds-long) probe runs on the next pass.
    if (g_app.dynamic_range_state == DYNAMIC_RANGE_REQUESTED)
    {
        g_app.dynamic_range_state = DYNAMIC_RANGE_RUNNING;
        return;
    }
    if (g_app.dynamic_range_state == DYNAMIC_RANGE_RUNNING)
    {
        dynamic_load_run_range();
        g_app.dynamic_range_state = DYNAMIC_RANGE_IDLE;
        g_app.dynamic_last_sample_tick = 0;
        app_display_mark_dirty();
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

    float avg_bus_mV = 0.0f, raw_mA = 0.0f;
    if (measurement_sample(&avg_bus_mV, &raw_mA))
    {
        g_app.ina_err_mA_per_V = measurement_ina_k_mA_per_V();
        float i_mA = dynamic_corrected_mA(avg_bus_mV, raw_mA);

        g_app.dynamic_measured_mA = (i_mA > 0.0f) ? i_mA : 0.0f;
        g_app.dynamic_bus_mv = (int32_t)avg_bus_mV;
        g_app.dynamic_power_mW = (g_app.dynamic_measured_mA * avg_bus_mV) / 1000.0f;
        g_app.dynamic_measured_valid = true;
        // Machine-readable dump for debugging over USB serial:
        // DYN,duty,bus_mV,raw_mA,corrected_mA
        ESP_LOGI(TAG, "DYN,%lu,%.1f,%.3f,%.3f", (unsigned long)g_app.dynamic_duty_steps,
                 (double)avg_bus_mV, (double)raw_mA, (double)g_app.dynamic_measured_mA);

        if (g_app.dynamic_power_mW >= DYNAMIC_LOAD_POWER_LIMIT_MW)
        {
            g_app.dynamic_power_limited = true;
            if (g_app.dynamic_duty_steps > 0)
            {
                if (g_app.state_mtx != NULL && xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(DYNLOAD_LOCK_TIMEOUT_MS)) == pdTRUE)
                {
                    if (g_app.dynamic_pos > 0)
                        g_app.dynamic_pos--;
                    uint32_t reduced_duty = dynamic_pos_duty(g_app.dynamic_pos);
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
        else if (g_app.dynamic_power_mW < (DYNAMIC_LOAD_POWER_LIMIT_MW - LOAD_POWER_NEAR_MARGIN_MW))
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
    g_app.dynamic_session++;
    g_app.dynamic_power_limited = false;
    g_app.dynamic_range_state = DYNAMIC_RANGE_IDLE;
    g_app.dynamic_pos = 0;

    // Knob scale from the last REAL sweep, if any: its auto-range top is the
    // duty that reached that panel's Isc. Light may have changed since, so a
    // double press re-probes (dynamic_load_request_range).
    uint32_t pwm_res = 0;
    uint32_t cap = (pwm_controller_get_resolution(&pwm_res) == 0) ? dynamic_load_max_duty(pwm_res) : 0;
    uint32_t top = g_app.last_sweep_top_duty;
    if (top > 0 && top <= cap)
    {
        g_app.dynamic_full_duty = top;
        g_app.dynamic_full_mA = g_app.last_sweep_isc_mA;
    }
    else
    {
        g_app.dynamic_full_duty = cap;
        g_app.dynamic_full_mA = 0.0f;
    }
    if (g_app.dynamic_full_duty < DYNAMIC_LOAD_POSITIONS)
        g_app.dynamic_full_duty = DYNAMIC_LOAD_POSITIONS;
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
        g_app.dynamic_pos = 0;
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
        g_app.dynamic_pos = 0;
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
        g_app.dynamic_pos = 0;
        g_app.dynamic_last_adjust_tick = xTaskGetTickCount();
        return;
    }

    uint32_t pwm_res = 0;
    if (pwm_controller_get_resolution(&pwm_res) != 0 || pwm_res == 0)
    {
        ESP_LOGW(TAG, "dynamic_load: failed to read PWM resolution");
        return;
    }

    uint32_t max_duty = dynamic_load_max_duty(pwm_res);
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
