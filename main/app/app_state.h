#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "driver_ina219.h"
#include "driver_sh1106.h"

typedef enum
{
    UI_SCREEN_HOME = 0,
    UI_SCREEN_MENU,
    UI_SCREEN_ACTION_QR,
    UI_SCREEN_ACTION_MEASURE,
    UI_SCREEN_ACTION_SYSTEM,
    UI_SCREEN_ACTION_DYNAMIC_LOAD,
} ui_screen_t;

typedef enum
{
    UI_QR_WIFI = 0,
    UI_QR_AP_IP,
    UI_QR_OTA,
} ui_qr_kind_t;

typedef enum
{
    HOME_SECTION_NETWORK = 0,
    HOME_SECTION_MEASURE,
    HOME_SECTION_POWER,
    HOME_SECTION_COUNT,
} home_section_t;

typedef struct
{
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t ina_dev;
    sh1106_t display;
    ina219_cal_t ina_cal;

    TaskHandle_t producer_task;
    TaskHandle_t display_task;
    TaskHandle_t encoder_task;
    SemaphoreHandle_t state_mtx;

    bool display_enabled;
    bool measurement_running;
    bool pwm_ready;
    bool ina_ready;
    char wifi_qr_payload[128];
    uint8_t qr_frame[SH1106_FB_SIZE];
    bool qr_frame_valid;
    ui_screen_t ui_screen;
    ui_qr_kind_t ui_qr_kind;
    int ui_home_index;
    int ui_menu_index;
    int ui_measure_index;
    int ui_system_index;
    bool dynamic_load_active;
    float dynamic_measured_mA;
    float dynamic_power_mW;
    int32_t dynamic_bus_mv;
    int32_t dynamic_shunt_uv;
    bool dynamic_measured_valid;
    bool dynamic_power_limited;
    uint32_t dynamic_duty_steps;
    TickType_t dynamic_last_adjust_tick;
    TickType_t dynamic_last_sample_tick;
    bool display_dirty;
} app_state_t;

extern app_state_t g_app;

#define MAX_MEASUREMENTS_PER_CYCLE 10

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_6
#define I2C_SCL_GPIO GPIO_NUM_7
#define I2C_FREQ_HZ 100000
#define WS2812_GPIO GPIO_NUM_10
#define OLED_ROTATE_180 false
#define OLED_COLUMN_OFFSET 2
#define OLED_QR_Y_OFFSET 3
#define ENC_DT_GPIO GPIO_NUM_2
#define ENC_CLK_GPIO GPIO_NUM_3
#define ENC_SW_GPIO GPIO_NUM_4
#define DYNAMIC_LOAD_DUTY_STEP 96
#define DYNAMIC_LOAD_DUTY_MAX_PERCENT 10
#define DYNAMIC_LOAD_SAMPLE_COUNT 8
#define DYNAMIC_LOAD_UPDATE_MS 120
#define DYNAMIC_LOAD_SETTLE_MS 80
#define LOAD_POWER_LIMIT_MW 2000.0f
#define LOAD_POWER_NEAR_MARGIN_MW 150.0f
