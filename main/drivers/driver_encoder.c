#include "driver_encoder.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#ifndef CONFIG_FREERTOS_HZ
#define CONFIG_FREERTOS_HZ 100
#endif

static uint32_t ticks_to_ms(TickType_t ticks)
{
    return (uint32_t)(((uint64_t)ticks * 1000ULL) / (uint64_t)CONFIG_FREERTOS_HZ);
}

static TickType_t ms_to_ticks(uint32_t ms)
{
    return (TickType_t)(((uint64_t)ms * (uint64_t)CONFIG_FREERTOS_HZ + 999ULL) / 1000ULL);
}

typedef struct
{
    gpio_num_t dt_pin;
    gpio_num_t clk_pin;
    gpio_num_t sw_pin;
    uint32_t sw_debounce_ms;
    QueueHandle_t queue;
    volatile int32_t position;
    volatile int last_clk_level;
    volatile TickType_t last_sw_tick;
    bool initialized;
} encoder_state_t;

static encoder_state_t s_encoder = {0};

static void IRAM_ATTR encoder_isr(void *arg)
{
    uint32_t pin = (uint32_t)(uintptr_t)arg;

    if (pin == (uint32_t)s_encoder.clk_pin)
    {
        int clk = gpio_get_level(s_encoder.clk_pin);
        if (clk != s_encoder.last_clk_level)
        {
            s_encoder.last_clk_level = clk;
            if (clk == 1)
            {
                int dt = gpio_get_level(s_encoder.dt_pin);
                int dir = (dt != clk) ? 1 : -1;
                s_encoder.position += dir;

                BaseType_t woke = pdFALSE;
                if (s_encoder.queue)
                {
                    encoder_event_t ev = {
                        .type = (dir > 0) ? ENCODER_EVENT_CW : ENCODER_EVENT_CCW,
                        .position = s_encoder.position,
                        .timestamp_ms = ticks_to_ms(xTaskGetTickCountFromISR()),
                    };
                    xQueueSendFromISR(s_encoder.queue, &ev, &woke);
                }
                if (woke)
                {
                    portYIELD_FROM_ISR();
                }
            }
        }
    }
    else if (pin == (uint32_t)s_encoder.sw_pin)
    {
        TickType_t now = xTaskGetTickCountFromISR();
        TickType_t debounce_ticks = ms_to_ticks(s_encoder.sw_debounce_ms);

        if ((now - s_encoder.last_sw_tick) >= debounce_ticks)
        {
            s_encoder.last_sw_tick = now;

            BaseType_t woke = pdFALSE;
            if (s_encoder.queue)
            {
                encoder_event_t ev = {
                    .type = ENCODER_EVENT_BUTTON,
                    .position = s_encoder.position,
                    .timestamp_ms = ticks_to_ms(now),
                };
                xQueueSendFromISR(s_encoder.queue, &ev, &woke);
            }
            if (woke)
            {
                portYIELD_FROM_ISR();
            }
        }
    }
}

esp_err_t encoder_init(const encoder_config_t *cfg)
{
    if (!cfg)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_encoder.initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (cfg->dt_pin == cfg->clk_pin || cfg->dt_pin == cfg->sw_pin || cfg->clk_pin == cfg->sw_pin)
    {
        return ESP_ERR_INVALID_ARG;
    }

    s_encoder.dt_pin = cfg->dt_pin;
    s_encoder.clk_pin = cfg->clk_pin;
    s_encoder.sw_pin = cfg->sw_pin;
    s_encoder.sw_debounce_ms = (cfg->sw_debounce_ms == 0) ? 150 : cfg->sw_debounce_ms;
    s_encoder.position = 0;
    s_encoder.last_sw_tick = 0;

    uint32_t q_len = (cfg->event_queue_len == 0) ? 16 : cfg->event_queue_len;
    s_encoder.queue = xQueueCreate(q_len, sizeof(encoder_event_t));
    if (!s_encoder.queue)
    {
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t rotary_cfg = {
        .pin_bit_mask = (1ULL << s_encoder.dt_pin) | (1ULL << s_encoder.clk_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = cfg->use_internal_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&rotary_cfg));

    gpio_config_t sw_cfg = {
        .pin_bit_mask = (1ULL << s_encoder.sw_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = cfg->use_internal_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&sw_cfg));

    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        vQueueDelete(s_encoder.queue);
        s_encoder.queue = NULL;
        return ret;
    }

    s_encoder.last_clk_level = gpio_get_level(s_encoder.clk_pin);

    ESP_ERROR_CHECK(gpio_isr_handler_add(s_encoder.clk_pin, encoder_isr, (void *)(uintptr_t)s_encoder.clk_pin));
    ESP_ERROR_CHECK(gpio_isr_handler_add(s_encoder.sw_pin, encoder_isr, (void *)(uintptr_t)s_encoder.sw_pin));

    s_encoder.initialized = true;
    return ESP_OK;
}

bool encoder_get_event(encoder_event_t *out_event, TickType_t wait_ticks)
{
    if (!s_encoder.initialized || !s_encoder.queue || !out_event)
    {
        return false;
    }

    return xQueueReceive(s_encoder.queue, out_event, wait_ticks) == pdTRUE;
}

int32_t encoder_get_position(void)
{
    return s_encoder.position;
}
