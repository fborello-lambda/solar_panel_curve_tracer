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

static const char *TAG = "MAIN";

// TODO: Create a state struct to hold all global state variables
// inside db.c.
// It will be easier to manage and avoid global variables.
// But for now, just use global variables.
i2c_master_bus_handle_t g_bus_handle;
i2c_master_dev_handle_t g_dev_handle;
ina219_cal_t g_cal;
#define MAX_MEASUREMENTS_PER_CYCLE 10
uint32_t g_step_size_per_measurement = 0;

// --------------------------------------------------------------------------------

#define BTN_GPIO 0
#define LED_GPIO 1
#define DEBOUNCE_MS 100

static TaskHandle_t meas_task_handle = NULL;
static TaskHandle_t producer_task_handle = NULL;
static SemaphoreHandle_t state_mtx = NULL;

/* new shared LED state (visible across tasks) */
static volatile bool g_led_on = false;

static void measurement_set_led_locked(bool on);
static bool measurement_start_locked(void);
static bool measurement_stop_locked(void);

bool measurement_is_running(void)
{
    return g_led_on;
}

bool measurement_request(bool start)
{
    if (state_mtx == NULL)
    {
        ESP_LOGW(TAG, "measurement_request_stop: state mutex not ready");
        return false;
    }

    bool stopped = false;
    if (xSemaphoreTake(state_mtx, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        stopped = start ? measurement_start_locked() : measurement_stop_locked();
        xSemaphoreGive(state_mtx);
    }
    else
    {
        ESP_LOGW(TAG, "measurement_request_stop: failed to take mutex");
    }
    return stopped;
}

static void dummy_producer_task(void *arg);
static void producer_task(void *arg);

// #define DUMMY_PRODUCER
#ifdef DUMMY_PRODUCER
#define PRODUCER dummy_producer_task
#else
#define PRODUCER producer_task
#endif

static void measurement_set_led_locked(bool on)
{
    g_led_on = on;
    gpio_set_level(LED_GPIO, on ? 1 : 0);
}

static bool measurement_start_locked(void)
{
    if (g_led_on)
        return false;

    db_reset();

    if (producer_task_handle == NULL)
    {
        if (xTaskCreate(PRODUCER, "producer", 4096, NULL, 5, &producer_task_handle) != pdPASS)
        {
            ESP_LOGE(TAG, "measurement_start_locked: failed to create producer task");
            producer_task_handle = NULL;
            return false;
        }
    }

    measurement_set_led_locked(true);
    ESP_LOGI(TAG, "Measurement cycle started");
    return true;
}

static bool measurement_stop_locked(void)
{
    if (!g_led_on)
        return false;

    if (producer_task_handle)
    {
        db_reset();
        pwm_controller_set_duty(0);
        vTaskDelete(producer_task_handle);
        producer_task_handle = NULL;
    }

    measurement_set_led_locked(false);
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

static void start_meas(void *arg)
{
    gpio_set_level(LED_GPIO, 0);
    const TickType_t half_debounce = pdMS_TO_TICKS(DEBOUNCE_MS / 2);

    for (;;)
    {
        // wait for ISR notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // short wait so bounce settles, then sample
        vTaskDelay(half_debounce);
        if (gpio_get_level(BTN_GPIO) != 0)
            continue;

        // confirm stable press
        vTaskDelay(half_debounce);
        if (gpio_get_level(BTN_GPIO) != 0)
            continue;

        bool was_running = false;
        bool changed = false;
        bool attempted = false;
        if (xSemaphoreTake(state_mtx, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            was_running = g_led_on;
            changed = was_running ? measurement_stop_locked() : measurement_start_locked();
            attempted = true;
            xSemaphoreGive(state_mtx);
        }
        else
        {
            ESP_LOGW(TAG, "start_meas: failed to take state mutex");
        }

        if (attempted && !changed)
        {
            ESP_LOGW(TAG, "start_meas: %s request ignored", was_running ? "stop" : "start");
        }

        // wait until button released to avoid multiple toggles for one press
        while (gpio_get_level(BTN_GPIO) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ISR: notify the led task (must use task handle)
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    BaseType_t woke1 = pdFALSE;

    if (meas_task_handle)
        vTaskNotifyGiveFromISR(meas_task_handle, &woke1);

    // if vTaskNotifyGiveFromISR woke a higher priority task, request a context switch now
    if (woke1)
        portYIELD_FROM_ISR();
}

// Call this during initialization (e.g. in app_main or after wifi init)
static void button_init_and_start_tasks(void)
{
    // create mutex for shared state
    state_mtx = xSemaphoreCreateMutex();
    if (state_mtx == NULL)
    {
        // handle error or log
    }

    // configure LED GPIO (output)
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    // start led task and keep handle
    if (xTaskCreate(start_meas, "start_meas", 2048, NULL, 5, &meas_task_handle) != pdPASS)
    {
        meas_task_handle = NULL;
        return;
    }

    // configure button GPIO (input, internal pull-up, falling edge)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BTN_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE};
    gpio_config(&io_conf);

    // install ISR service and add handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_GPIO, gpio_isr_handler, (void *)(uintptr_t)BTN_GPIO);
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
    if (state_mtx && xSemaphoreTake(state_mtx, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        producer_task_handle = NULL;
        measurement_set_led_locked(false);
        pwm_controller_set_duty(0);
        xSemaphoreGive(state_mtx);
    }
    else
    {
        // best-effort fallback
        producer_task_handle = NULL;
        g_led_on = false;
        pwm_controller_set_duty(0);
        gpio_set_level(LED_GPIO, 0);
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

            if (ina219_get_shunt_voltage_uv(g_dev_handle, &shunt_uV) != ESP_OK)
            {
                ESP_LOGW(TAG, "driver_ina219: shunt read failed");
            }
            if (ina219_get_bus_voltage_mv(g_dev_handle, &bus_mV) != ESP_OK)
            {
                ESP_LOGW(TAG, "driver_ina219: bus read failed");
            }
            if (ina219_get_current_ma(g_dev_handle, &g_cal, &current_mA) != ESP_OK)
            {
                ESP_LOGW(TAG, "driver_ina219: current read failed");
            }
            if (ina219_get_power_mw(g_dev_handle, &g_cal, &power_mW) != ESP_OK)
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
    if (state_mtx && xSemaphoreTake(state_mtx, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        producer_task_handle = NULL;
        measurement_set_led_locked(false);
        pwm_controller_set_duty(0);
        xSemaphoreGive(state_mtx);
    }
    else
    {
        // best-effort fallback
        producer_task_handle = NULL;
        g_led_on = false;
        pwm_controller_set_duty(0);
        gpio_set_level(LED_GPIO, 0);
    }

    ESP_LOGI(TAG, "producer_task: Deleting self");

    vTaskDelete(NULL);
}

void app_main(void)
{

    if (PRODUCER == producer_task)
    {
        int res = pwm_controller_init(NULL);
        if (res != 0)
        {
            ESP_LOGE(TAG, "pwm_controller_init failed: %d", res);
            return;
        }

        res = ina219_init(&g_bus_handle, &g_dev_handle, I2C_NUM_0, 6, 7, 100000, INA219_ADDRESS_DEFAULT);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "ina219_init failed: %s", esp_err_to_name(res));
            return;
        }

        res = ina219_calibrate_for_32V_10A(g_dev_handle, &g_cal);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "ina219_calibrate_for_32V_10A failed");
            return;
        }

        ESP_LOGI(TAG, "driver_ina219: Calibration Done -- Current_Divider_mA=%d  Power_Multiplier_mW=%d  Current_LSB=%.6f A/bit CAL=0x%04X",
                 g_cal.current_divider_mA, g_cal.power_multiplier_mW, g_cal.current_lsb, g_cal.cal_value);
    }

    system_init_all();

    button_init_and_start_tasks();
}
