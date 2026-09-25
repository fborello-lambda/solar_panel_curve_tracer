#include "measurement.h"

#include <math.h>
#include <stdio.h>

#include <esp_log.h>
#include <esp_timer.h>

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

// ── auto-range / sweep tunables ─────────────────────────────────────────
// See AGENTS.md "Key design patterns" for the shape of the algorithm this
// backs: a Voc probe, a doubling search for the knee, then 20 points placed
// mostly across that knee. Ported from a sibling Rust implementation that
// runs the same PWM -> RC -> VCCS -> MOSFET hardware design.

#define SWEEP_PROBE_START_DUTY 8         // first duty the doubling search tries
#define SWEEP_COLLAPSE_PERCENT_OF_VOC 15 // panel counts as collapsed below this % of Voc
#define SWEEP_DUTY_MAX_PERCENT 20        // hard ceiling on commanded duty, % of pwm_res
#define SWEEP_KNEE_HEADROOM_PERCENT 115  // sweep top = knee * this / 100
#define SWEEP_SETTLE_MS 250              // settle time after a duty step before sampling
#define SWEEP_FIRST_POINT_SETTLE_MS 1000 // longer settle for the true open-circuit point
#define SWEEP_SAMPLE_WINDOW_MS 100       // averaging window, an integer number of 50/60 Hz half-cycles
#define SWEEP_VOC_MIN_MV 500             // below this, no panel worth sweeping
#define SWEEP_FINE_LEG_POINTS 12         // points spent across the knee band
#define SWEEP_TAIL_LEG_POINTS 2          // points spent from the knee band to top
#define SWEEP_FINE_BAND_START_PERCENT 85 // fine band start, % of knee duty
#define SWEEP_FINE_BAND_END_PERCENT 102  // fine band end, % of knee duty
#define SWEEP_STOP_POLL_MS 25            // chunk size for waits, so stop is honored quickly

static curve_producer_mode_t s_producer_mode = CURVE_PRODUCER_REAL;

static void measurement_apply_state_locked(bool running);
static bool measurement_start_locked(void);
static bool measurement_stop_locked(void);

static void dummy_producer_task(void *arg);
static void producer_task(void *arg);
static void producer_finish(const char *task_name);

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

    // For a stop, wait (outside the mutex) for the producer to actually exit,
    // up to a bound. The producer clears g_app.producer_task on self-delete.
    if (!start && changed)
    {
        const int max_wait_ms = 2000;
        int waited = 0;
        while (g_app.producer_task != NULL && waited < max_wait_ms)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            waited += 20;
        }
        if (g_app.producer_task != NULL)
        {
            ESP_LOGW(TAG, "measurement_request: producer did not stop within %d ms", max_wait_ms);
        }
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

    if (g_app.dynamic_load_active)
    {
        ESP_LOGW(TAG, "measurement_start_locked: refused, dynamic load screen is active");
        return false;
    }

    db_reset();
    g_app.measurement_stop_requested = false;

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

    // Cooperative stop: signal the producer, which will finish its current
    // step, release all locks, zero the PWM, and delete itself. Never
    // vTaskDelete() it here — it may hold the I2C bus or db mutex.
    g_app.measurement_stop_requested = true;
    ESP_LOGI(TAG, "Measurement stop requested");
    return true;
}

static void producer_finish(const char *task_name)
{
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

    ESP_LOGI(TAG, "%s: Deleting self", task_name);
}

// ── sweep point placement ───────────────────────────────────────────────

// Commanded duty for sweep step `step`, given the auto-ranged `top` and the
// `knee` duty it was derived from. Three legs, not a linear ramp: a coarse
// leg over the flat below-knee region (a panel there is a current source, so
// coarse steps lose little), most of the budget across the knee band where
// the curve actually bends, and a short tail up to `top` to pin Isc. Step 0
// is always duty 0, the last step is always exactly `top`.
static uint32_t sweep_duty_for_step(int step, uint32_t top, uint32_t knee)
{
    const int coarse_points = DB_MAX_SAMPLES - SWEEP_FINE_LEG_POINTS - SWEEP_TAIL_LEG_POINTS;
    uint32_t band_start = knee * SWEEP_FINE_BAND_START_PERCENT / 100;
    uint32_t band_end = knee * SWEEP_FINE_BAND_END_PERCENT / 100;
    if (band_end > top)
        band_end = top;

    if (step < coarse_points)
    {
        return (uint32_t)((uint64_t)step * band_start / coarse_points);
    }
    else if (step < coarse_points + SWEEP_FINE_LEG_POINTS)
    {
        uint32_t fine_step = (uint32_t)(step - coarse_points);
        uint32_t span = (band_end > band_start) ? (band_end - band_start) : 0;
        return band_start + (uint32_t)((uint64_t)fine_step * span / (SWEEP_FINE_LEG_POINTS - 1));
    }
    else
    {
        uint32_t tail_step = (uint32_t)(step - coarse_points - SWEEP_FINE_LEG_POINTS + 1);
        uint32_t span = (top > band_end) ? (top - band_end) : 0;
        return band_end + (uint32_t)((uint64_t)tail_step * span / SWEEP_TAIL_LEG_POINTS);
    }
}

// ── point measurement ────────────────────────────────────────────────────

typedef enum
{
    MEASURE_OK = 0,
    MEASURE_STOP_REQUESTED,
    MEASURE_NO_VALID_READS,
    MEASURE_POWER_LIMIT,
} measure_result_t;

// Sleeps `ms` in small chunks, checking the cooperative stop flag between
// each chunk so a stop request is honored within roughly SWEEP_STOP_POLL_MS.
// Returns true if a stop was requested during the wait.
static bool sleep_chunked_checking_stop(uint32_t ms)
{
    uint32_t waited = 0;
    while (waited < ms)
    {
        if (g_app.measurement_stop_requested)
            return true;
        uint32_t chunk = (ms - waited < SWEEP_STOP_POLL_MS) ? (ms - waited) : SWEEP_STOP_POLL_MS;
        vTaskDelay(pdMS_TO_TICKS(chunk));
        waited += chunk;
    }
    return g_app.measurement_stop_requested;
}

// Commands `duty_steps`, settles for `settle_ms`, then averages bus/shunt
// voltage readings over a fixed SWEEP_SAMPLE_WINDOW_MS window (sampled as
// fast as I2C allows) — 100 ms spans an integer number of half-cycles of
// both 50 Hz and 60 Hz mains flicker, so ripple from a lamp cancels out.
// Used by both auto-range probes and recorded sweep points.
static measure_result_t measure_point(uint32_t duty_steps, uint32_t settle_ms,
                                       float *out_v, float *out_i_mA, float *out_power_mW)
{
    pwm_controller_set_duty_in_res_steps(duty_steps);

    if (sleep_chunked_checking_stop(settle_ms))
        return MEASURE_STOP_REQUESTED;

    int64_t bus_mV_sum = 0;
    int64_t shunt_uV_sum = 0;
    int valid = 0;

    int64_t window_start = esp_timer_get_time();
    while ((esp_timer_get_time() - window_start) < (int64_t)SWEEP_SAMPLE_WINDOW_MS * 1000)
    {
        if (g_app.measurement_stop_requested)
            break;

        int32_t bus_mV = 0, shunt_uV = 0;
        bool ok = ina219_get_bus_voltage_mv(g_app.ina_dev, &bus_mV) == ESP_OK;
        ok &= ina219_get_shunt_voltage_uv(g_app.ina_dev, &shunt_uV) == ESP_OK;
        if (ok)
        {
            bus_mV_sum += bus_mV;
            shunt_uV_sum += shunt_uV;
            valid++;
        }
    }

    if (g_app.measurement_stop_requested)
        return MEASURE_STOP_REQUESTED;

    if (valid == 0)
        return MEASURE_NO_VALID_READS;

    float bus_mV = (float)bus_mV_sum / (float)valid;
    float shunt_uV = (float)shunt_uV_sum / (float)valid;
    int shunt_mOhm = g_app.ina_cal.shunt_resistor_mOhm;
    float current_mA = (shunt_mOhm > 0) ? (shunt_uV / (float)shunt_mOhm) : 0.0f;
    float voltage_mV = bus_mV - (shunt_uV / 1000.0f);

    if (voltage_mV < 0.0f)
        voltage_mV = 0.0f;
    if (current_mA < 0.0f)
        current_mA = 0.0f;

    *out_v = voltage_mV / 1000.0f; // volts
    *out_i_mA = current_mA;
    *out_power_mW = (voltage_mV * current_mA) / 1000.0f;

    if (*out_power_mW >= LOAD_POWER_LIMIT_MW)
        return MEASURE_POWER_LIMIT;

    return MEASURE_OK;
}

// ── auto-range ───────────────────────────────────────────────────────────

// Finds this sweep's top duty by probing for Voc, then doubling the
// commanded duty until the panel collapses (voltage drops below
// SWEEP_COLLAPSE_PERCENT_OF_VOC of Voc). Probe points are not recorded in
// db. Returns false if the sweep should abort (no panel, stop requested, or
// a safety breach).
static bool auto_range(uint32_t pwm_res, uint32_t *out_top, uint32_t *out_knee, float *out_voc_mv)
{
    uint32_t hard_max = pwm_res * SWEEP_DUTY_MAX_PERCENT / 100;
    if (hard_max < 1)
        hard_max = 1;

    float voc_v, dummy_i, dummy_p;
    measure_result_t r = measure_point(0, SWEEP_SETTLE_MS, &voc_v, &dummy_i, &dummy_p);
    if (r != MEASURE_OK)
        return false;

    float voc_mV = voc_v * 1000.0f;
    *out_voc_mv = voc_mV;
    if (voc_mV < SWEEP_VOC_MIN_MV)
    {
        ESP_LOGW(TAG, "auto_range: Voc %.0f mV below %d mV, no panel to sweep (dark, disconnected)",
                 voc_mV, SWEEP_VOC_MIN_MV);
        return false;
    }

    float collapse_mV = voc_mV * SWEEP_COLLAPSE_PERCENT_OF_VOC / 100.0f;

    // Ascending doubling search only: the RC filter that turns this PWM
    // duty into an analog setpoint is slow next to a probe's settle window,
    // so a probe walking duty DOWN would still read the previous, higher
    // duty's current draining out of the filter rather than its own.
    uint32_t lo = 0;
    float lo_i_mA = 0.0f; // stores lo's measured current, as float mA
    uint32_t knee = 0;
    uint32_t duty = (SWEEP_PROBE_START_DUTY < hard_max) ? SWEEP_PROBE_START_DUTY : hard_max;

    for (;;)
    {
        float v, i_mA, p_mW;
        r = measure_point(duty, SWEEP_SETTLE_MS, &v, &i_mA, &p_mW);
        if (r != MEASURE_OK)
            return false;

        float v_mV = v * 1000.0f;
        ESP_LOGI(TAG, "auto_range: probe duty=%lu -> V=%.0f mV I=%.1f mA", (unsigned long)duty, v_mV, i_mA);

        if (v_mV <= collapse_mV)
        {
            float isc_mA = i_mA;
            if (lo == 0 || lo_i_mA <= 0.5f)
            {
                // Collapsed on the very first loaded probe: no regulating
                // point to scale from, so fall back to the collapsing duty.
                knee = duty;
            }
            else
            {
                knee = (uint32_t)((isc_mA * (float)lo) / lo_i_mA);
                if (knee > hard_max)
                    knee = hard_max;
            }
            ESP_LOGI(TAG, "auto_range: collapsed at duty %lu, Isc=%.1f mA, %.1f mA per 1000 duty steps, knee at duty %lu",
                     (unsigned long)duty, isc_mA,
                     lo ? (double)((lo_i_mA * 1000.0f) / (float)lo) : 0.0,
                     (unsigned long)knee);
            break;
        }

        lo = duty;
        lo_i_mA = i_mA;
        if (duty >= hard_max)
            break;
        duty = (duty * 2 < hard_max) ? duty * 2 : hard_max;
    }

    if (knee == 0)
    {
        ESP_LOGW(TAG, "auto_range: never collapsed up to the %d%% duty cap, curve will stop short of Isc",
                 SWEEP_DUTY_MAX_PERCENT);
        *out_top = hard_max;
        *out_knee = hard_max * 100 / SWEEP_KNEE_HEADROOM_PERCENT;
        return true;
    }

    uint32_t top = (uint32_t)(((uint64_t)knee * SWEEP_KNEE_HEADROOM_PERCENT) / 100);
    if (top > hard_max)
        top = hard_max;
    if (top < DB_MAX_SAMPLES)
        top = DB_MAX_SAMPLES;

    ESP_LOGI(TAG, "auto_range: Voc=%.0f mV, knee at duty %lu, sweeping 0..%lu",
             voc_mV, (unsigned long)knee, (unsigned long)top);

    *out_top = top;
    *out_knee = knee;
    return true;
}

static void dummy_producer_task(void *arg)
{
    (void)arg;

    int8_t duty = 0;
    pwm_controller_set_duty(duty); // duty is a percentage (0..100)
    ESP_LOGI(TAG, "dummy_producer_task: Starting data production");

    float x_array[] = {22.464, 22.215, 21.942, 21.661, 21.365, 21.059, 20.731, 20.391, 20.021, 19.612, 19.164, 18.624, 17.823, 16.489, 14.571, 6.140, 0.060, 0.060, 0.059, 0.060};
    float y_array[] = {36.000, 72.000, 109.000, 147.000, 182.000, 219.000, 255.000, 292.000, 327.000, 364.000, 400.000, 436.000, 473.000, 510.000, 545.000, 579.000, 581.000, 582.000, 582.000, 583.000};

    int len = sizeof(x_array) / sizeof(x_array[0]);
    for (int i = 0; i < len; ++i)
    {
        if (g_app.measurement_stop_requested)
        {
            ESP_LOGI(TAG, "dummy_producer_task: stop requested, ending early");
            break;
        }

        ESP_LOGI(TAG, "dummy_producer_task: Set duty to %d%%", duty);

        if (!db_add(x_array[i], y_array[i]))
        {
            ESP_LOGW(TAG, "dummy_producer_task: db_add failed, dropping sample (x=%.3f,y=%.3f)", x_array[i], y_array[i]);
        }
        else
        {
            ESP_LOGI(TAG, "dummy_producer_task: db_add succeeded (x=%.3f,y=%.3f)", x_array[i], y_array[i]);
        }

        duty += 10;
        if (duty > 100)
            duty = 100;

        pwm_controller_set_duty(duty); // duty is a percentage (0..100)
        vTaskDelay(pdMS_TO_TICKS(250)); // pace like a real sweep, ~250 ms/point
    }

    ESP_LOGI(TAG, "dummy_producer_task: Finished data production");

    producer_finish("dummy_producer_task");
    vTaskDelete(NULL);
}

static void producer_task(void *arg)
{
    (void)arg;

    uint32_t pwm_res = 0;
    pwm_controller_get_resolution(&pwm_res);

    ESP_LOGI(TAG, "producer_task: starting auto-range");

    uint32_t top = 0, knee = 0;
    float voc_mV = 0.0f;
    if (!auto_range(pwm_res, &top, &knee, &voc_mV))
    {
        ESP_LOGW(TAG, "producer_task: auto-range aborted, recording nothing");
        ESP_LOGI(TAG, "producer_task stack high water mark: %u words", (unsigned)uxTaskGetStackHighWaterMark(NULL));
        producer_finish("producer_task");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "producer_task: Starting data production (top=%lu knee=%lu)",
             (unsigned long)top, (unsigned long)knee);

    uint32_t prev_duty = 0;
    for (int step = 0; step < DB_MAX_SAMPLES; step++)
    {
        if (g_app.measurement_stop_requested)
        {
            ESP_LOGI(TAG, "producer_task: stop requested, ending sweep early");
            break;
        }

        uint32_t duty = sweep_duty_for_step(step, top, knee);
        if (step > 0 && duty <= prev_duty)
            duty = prev_duty + 1;
        // Leave room for every remaining step to still strictly increase up
        // to `top`, so the bump above never crowds the final points.
        int steps_left = DB_MAX_SAMPLES - 1 - step;
        if (steps_left > 0 && duty > top - (uint32_t)steps_left)
            duty = top - (uint32_t)steps_left;
        if (step == DB_MAX_SAMPLES - 1)
            duty = top;
        prev_duty = duty;

        // Step 0 (duty 0) after auto-range's high-duty last probe needs a
        // longer settle so it reads a true open-circuit voltage instead of
        // the RC filter still draining down.
        uint32_t settle_ms = (step == 0) ? SWEEP_FIRST_POINT_SETTLE_MS : SWEEP_SETTLE_MS;

        float v, i_mA, p_mW;
        measure_result_t r = measure_point(duty, settle_ms, &v, &i_mA, &p_mW);

        if (r == MEASURE_STOP_REQUESTED)
        {
            ESP_LOGI(TAG, "producer_task: stop requested during step %d, ending sweep early", step);
            break;
        }
        if (r == MEASURE_NO_VALID_READS)
        {
            ESP_LOGW(TAG, "producer_task: no valid samples at step %d (duty=%lu), aborting sweep",
                     step, (unsigned long)duty);
            break;
        }
        if (r == MEASURE_POWER_LIMIT)
        {
            ESP_LOGW(TAG, "producer_task: power limit reached (%.0f mW >= %.0f mW) at step %d, aborting sweep",
                     p_mW, (float)LOAD_POWER_LIMIT_MW, step);
            break;
        }

        if (!db_add(v, i_mA))
        {
            ESP_LOGW(TAG, "producer_task: db_add failed, dropping sample (v=%.3f,i=%.3f)", v, i_mA);
        }
        else
        {
            ESP_LOGI(TAG, "producer_task: point %d duty=%lu V=%.3f I=%.3f mA P=%.1f mW",
                     step, (unsigned long)duty, v, i_mA, p_mW);
        }
    }

    ESP_LOGI(TAG, "producer_task: Finished data production");

    ESP_LOGI(TAG, "producer_task stack high water mark: %u words", (unsigned)uxTaskGetStackHighWaterMark(NULL));

    producer_finish("producer_task");
    vTaskDelete(NULL);
}
