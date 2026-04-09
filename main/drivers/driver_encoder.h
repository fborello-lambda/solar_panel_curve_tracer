#ifndef DRIVER_ENCODER_H
#define DRIVER_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include <esp_err.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    ENCODER_EVENT_CW = 0,
    ENCODER_EVENT_CCW,
    ENCODER_EVENT_BUTTON,
} encoder_event_type_t;

typedef struct
{
    encoder_event_type_t type;
    int32_t position;
    uint32_t timestamp_ms;
} encoder_event_t;

typedef struct
{
    gpio_num_t dt_pin;
    gpio_num_t clk_pin;
    gpio_num_t sw_pin;
    bool use_internal_pullups;
    uint32_t sw_debounce_ms;
    uint32_t event_queue_len;
} encoder_config_t;

esp_err_t encoder_init(const encoder_config_t *cfg);
bool encoder_get_event(encoder_event_t *out_event, TickType_t wait_ticks);
int32_t encoder_get_position(void);

#ifdef __cplusplus
}
#endif

#endif
