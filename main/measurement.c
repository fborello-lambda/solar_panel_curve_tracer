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
#include "sweep_plan.h"

static const char *TAG = "MEASURE";

// ── sweep I/O tunables ───────────────────────────────────────────────────
// The algorithm tunables (duty placement, auto-range thresholds) live in
// components/sweep_plan/include/sweep_plan.h, where they're testable on the
// host. See AGENTS.md "Key design patterns" for the shape of the algorithm:
// a Voc probe, a doubling search for the knee, then 20 points placed mostly
// across that knee. Ported from a sibling Rust implementation that runs the
// same PWM -> RC -> VCCS -> MOSFET hardware design.

#define SWEEP_SETTLE_MS 250              // settle time after a duty step before sampling
#define SWEEP_FIRST_POINT_SETTLE_MS 1000 // longer settle for the true open-circuit point
#define SWEEP_SAMPLE_WINDOW_MS 100       // averaging window, an integer number of 50/60 Hz half-cycles
#define SWEEP_STOP_POLL_MS 25            // chunk size for waits, so stop is honored quickly

static curve_producer_mode_t s_producer_mode = CURVE_PRODUCER_REAL;

static void measurement_apply_state_locked(bool running);
static bool measurement_start_locked(measurement_refuse_reason_t *out_reason);
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
    return measurement_request_ex(start, NULL);
}

bool measurement_request_ex(bool start, measurement_refuse_reason_t *out_reason)
{
    if (out_reason)
        *out_reason = MEASUREMENT_REFUSE_NONE;

    if (g_app.state_mtx == NULL)
    {
        ESP_LOGW(TAG, "measurement_request: state mutex not ready");
        if (out_reason)
            *out_reason = MEASUREMENT_REFUSE_SENSOR_NOT_READY;
        return false;
    }

    bool changed = false;
    if (xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        changed = start ? measurement_start_locked(out_reason) : measurement_stop_locked();
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

const char *measurement_refuse_reason_str(measurement_refuse_reason_t reason)
{
    switch (reason)
    {
    case MEASUREMENT_REFUSE_ALREADY_RUNNING:
        return "already running";
    case MEASUREMENT_REFUSE_DYNAMIC_LOAD_ACTIVE:
        return "dynamic load active";
    case MEASUREMENT_REFUSE_SENSOR_NOT_READY:
        return "sensor not ready";
    case MEASUREMENT_REFUSE_NONE:
    default:
        return "unknown";
    }
}

bool measurement_init_load_control_hw(void)
{
    g_app.pwm_ready = false;
    g_app.ina_ready = false;

    int pwm_ret = pwm_controller_init(NULL);
    if (pwm_ret != 0)
    {
        ESP_LOGE(TAG, "pwm_controller_init failed: %d", pwm_ret);
        return false;
    }
    g_app.pwm_ready = true;

    esp_err_t ret = app_hw_ensure_i2c_bus_ready();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = ina219_init_on_bus(g_app.i2c_bus, &g_app.ina_dev, I2C_FREQ_HZ, INA219_ADDRESS_DEFAULT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ina219_init_on_bus failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = ina219_calibrate_for_32V_10A(g_app.ina_dev, &g_app.ina_cal);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ina219_calibrate_for_32V_10A failed");
        return false;
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

static bool measurement_start_locked(measurement_refuse_reason_t *out_reason)
{
    if (g_app.measurement_running)
    {
        if (out_reason)
            *out_reason = MEASUREMENT_REFUSE_ALREADY_RUNNING;
        return false;
    }

    if (g_app.dynamic_load_active)
    {
        ESP_LOGW(TAG, "measurement_start_locked: refused, dynamic load screen is active");
        if (out_reason)
            *out_reason = MEASUREMENT_REFUSE_DYNAMIC_LOAD_ACTIVE;
        return false;
    }

    if (s_producer_mode == CURVE_PRODUCER_REAL && (!g_app.ina_ready || !g_app.pwm_ready))
    {
        ESP_LOGW(TAG, "measurement_start_locked: refused REAL mode, hardware not ready (ina_ready=%d pwm_ready=%d)",
                 g_app.ina_ready, g_app.pwm_ready);
        if (out_reason)
            *out_reason = MEASUREMENT_REFUSE_SENSOR_NOT_READY;
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
    if (g_app.state_mtx && xSemaphoreTake(g_app.state_mtx, pdMS_TO_TICKS(200)) == pdTRUE)
    {
        g_app.producer_task = NULL;
        measurement_apply_state_locked(false);
        pwm_controller_set_duty(0);
        xSemaphoreGive(g_app.state_mtx);
    }
    else
    {
        ESP_LOGW(TAG, "%s: failed to take state mutex, finishing without it", task_name);
        g_app.producer_task = NULL;
        g_app.measurement_running = false;
        pwm_controller_set_duty(0);
        led_clear(WS2812_GPIO);
        app_display_mark_dirty();
    }

    ESP_LOGI(TAG, "%s: Deleting self", task_name);
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
    sweep_range_t sr;
    sweep_range_begin(&sr, pwm_res, DB_MAX_SAMPLES);

    float voc_v, dummy_i, dummy_p;
    measure_result_t r = measure_point(0, SWEEP_SETTLE_MS, &voc_v, &dummy_i, &dummy_p);
    if (r != MEASURE_OK)
        return false;

    float voc_mV = voc_v * 1000.0f;
    *out_voc_mv = voc_mV;

    if (!sweep_range_on_voc(&sr, voc_mV))
    {
        ESP_LOGW(TAG, "auto_range: Voc %.0f mV below %d mV, no panel to sweep (dark, disconnected)",
                 voc_mV, SWEEP_VOC_MIN_MV);
        return false;
    }

    // Ascending doubling search only: the RC filter that turns this PWM
    // duty into an analog setpoint is slow next to a probe's settle window,
    // so a probe walking duty DOWN would still read the previous, higher
    // duty's current draining out of the filter rather than its own.
    for (;;)
    {
        float v, i_mA, p_mW;
        r = measure_point(sr.duty, SWEEP_SETTLE_MS, &v, &i_mA, &p_mW);
        if (r != MEASURE_OK)
            return false;

        float v_mV = v * 1000.0f;
        ESP_LOGI(TAG, "auto_range: probe duty=%lu -> V=%.0f mV I=%.1f mA", (unsigned long)sr.duty, v_mV, i_mA);

        if (sweep_range_on_probe(&sr, v_mV, i_mA))
            break;
    }

    if (!sr.collapsed)
    {
        ESP_LOGW(TAG, "auto_range: never collapsed up to the %d%% duty cap, curve will stop short of Isc",
                 SWEEP_DUTY_MAX_PERCENT);
    }
    else
    {
        ESP_LOGI(TAG, "auto_range: collapsed, Isc=%.1f mA, knee at duty %lu",
                 (double)sr.isc_mA, (unsigned long)sr.knee);
    }

    ESP_LOGI(TAG, "auto_range: Voc=%.0f mV, knee at duty %lu, sweeping 0..%lu",
             voc_mV, (unsigned long)sr.knee, (unsigned long)sr.top);

    *out_top = sr.top;
    *out_knee = sr.knee;
    return true;
}

static void dummy_producer_task(void *arg)
{
    (void)arg;

    // DEMO mode only replays a canned curve; it must never drive the real
    // load, so no pwm_controller_set_duty calls happen in this task.
    int8_t duty = 0;
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

    uint32_t duties[DB_MAX_SAMPLES];
    sweep_plan_build(top, knee, duties, DB_MAX_SAMPLES);

    for (int step = 0; step < DB_MAX_SAMPLES; step++)
    {
        if (g_app.measurement_stop_requested)
        {
            ESP_LOGI(TAG, "producer_task: stop requested, ending sweep early");
            break;
        }

        uint32_t duty = duties[step];

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
