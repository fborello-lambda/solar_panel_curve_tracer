#include "driver_encoder.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <esp_timer.h>

#include "quadrature.h"

#ifndef CONFIG_FREERTOS_HZ
#define CONFIG_FREERTOS_HZ 100
#endif

// Ignore a button press that lands within this long after the last emitted
// rotation step: it's almost always a thumb pushing down on the knob while
// still turning it, not a deliberate click.
#define ENCODER_POST_ROTATE_SUPPRESS_MS 150

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
    qdec_t qdec;
    uint32_t counts_per_step;
    volatile TickType_t last_rot_emit_tick;
    volatile bool sw_stable_high; // debounced button state: true = released (idle, pulled up)
    esp_timer_handle_t sw_timer;
    esp_timer_handle_t long_timer;
    bool initialized;
} encoder_state_t;

static encoder_state_t s_encoder = {0};

static void IRAM_ATTR encoder_rot_isr(void *arg)
{
    (void)arg;

    int clk = gpio_get_level(s_encoder.clk_pin);
    int dt = gpio_get_level(s_encoder.dt_pin);

    int step = qdec_update(&s_encoder.qdec, clk, dt, s_encoder.counts_per_step);
    if (step == 0)
    {
        return;
    }

    TickType_t now = xTaskGetTickCountFromISR();
    s_encoder.position += step;
    s_encoder.last_rot_emit_tick = now;

    if (s_encoder.queue)
    {
        BaseType_t woke = pdFALSE;
        encoder_event_t ev = {
            .type = (step > 0) ? ENCODER_EVENT_CW : ENCODER_EVENT_CCW,
            .position = s_encoder.position,
            .timestamp_ms = ticks_to_ms(now),
        };
        xQueueSendFromISR(s_encoder.queue, &ev, &woke);
        if (woke)
        {
            portYIELD_FROM_ISR();
        }
    }
}

static void IRAM_ATTR encoder_sw_isr(void *arg)
{
    (void)arg;

    // Any edge (re)starts the settle timer; the timer callback is the only
    // place that reads the level and decides whether a transition is real.
    // esp_timer_start_once() is safe from ISR context and restarts a
    // still-pending one-shot timer.
    esp_timer_stop(s_encoder.sw_timer);
    esp_timer_start_once(s_encoder.sw_timer, (uint64_t)s_encoder.sw_debounce_ms * 1000ULL);
}

// One-shot, started on a confirmed press and stopped on the confirmed release.
// Both callbacks run in the esp_timer task, so they never interleave.
static void sw_long_timer_cb(void *arg)
{
    (void)arg;

    if (s_encoder.sw_stable_high || !s_encoder.queue)
    {
        return; // released in the meantime
    }

    TickType_t now = xTaskGetTickCount();
    encoder_event_t ev = {
        .type = ENCODER_EVENT_LONG_PRESS,
        .position = s_encoder.position,
        .timestamp_ms = ticks_to_ms(now),
    };
    xQueueSend(s_encoder.queue, &ev, 0);
}

// Runs in the esp_timer task context (not ISR context), so it may use
// blocking-capable APIs such as xQueueSend.
static void sw_settle_timer_cb(void *arg)
{
    (void)arg;

    int level = gpio_get_level(s_encoder.sw_pin);
    bool now_high = (level != 0);

    if (now_high == s_encoder.sw_stable_high)
    {
        return; // no real transition since the last confirmed state
    }

    TickType_t now = xTaskGetTickCount();

    if (s_encoder.sw_stable_high && !now_high)
    {
        // Confirmed stable high -> low: a press.
        s_encoder.sw_stable_high = false;

        TickType_t suppress_ticks = ms_to_ticks(ENCODER_POST_ROTATE_SUPPRESS_MS);
        if ((now - s_encoder.last_rot_emit_tick) < suppress_ticks)
        {
            return; // thumb pressing the knob while still turning it
        }

        if (s_encoder.queue)
        {
            encoder_event_t ev = {
                .type = ENCODER_EVENT_BUTTON,
                .position = s_encoder.position,
                .timestamp_ms = ticks_to_ms(now),
            };
            xQueueSend(s_encoder.queue, &ev, 0);
            esp_timer_stop(s_encoder.long_timer);
            esp_timer_start_once(s_encoder.long_timer, (uint64_t)ENCODER_LONG_PRESS_MS * 1000ULL);
        }
    }
    else
    {
        // Confirmed stable low -> high: a release. Update state only, a new
        // press can only be emitted after this.
        s_encoder.sw_stable_high = true;
        esp_timer_stop(s_encoder.long_timer);
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
    s_encoder.sw_debounce_ms = (cfg->sw_debounce_ms == 0) ? 25 : cfg->sw_debounce_ms;
    s_encoder.position = 0;
    s_encoder.counts_per_step = (cfg->counts_per_step == 0) ? 4 : cfg->counts_per_step;
    s_encoder.last_rot_emit_tick = 0;
    s_encoder.sw_stable_high = true;

    uint32_t q_len = (cfg->event_queue_len == 0) ? 16 : cfg->event_queue_len;
    s_encoder.queue = xQueueCreate(q_len, sizeof(encoder_event_t));
    if (!s_encoder.queue)
    {
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t dt_cfg = {
        .pin_bit_mask = (1ULL << s_encoder.dt_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = cfg->use_internal_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&dt_cfg));

    gpio_config_t clk_cfg = {
        .pin_bit_mask = (1ULL << s_encoder.clk_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = cfg->use_internal_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&clk_cfg));

    gpio_config_t sw_cfg = {
        .pin_bit_mask = (1ULL << s_encoder.sw_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = cfg->use_internal_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&sw_cfg));

    const esp_timer_create_args_t timer_args = {
        .callback = sw_settle_timer_cb,
        .arg = NULL,
        .name = "enc_sw_settle",
    };
    const esp_timer_create_args_t long_args = {
        .callback = sw_long_timer_cb,
        .arg = NULL,
        .name = "enc_sw_long",
    };
    esp_err_t timer_ret = esp_timer_create(&timer_args, &s_encoder.sw_timer);
    if (timer_ret == ESP_OK)
    {
        timer_ret = esp_timer_create(&long_args, &s_encoder.long_timer);
        if (timer_ret != ESP_OK)
        {
            esp_timer_delete(s_encoder.sw_timer);
            s_encoder.sw_timer = NULL;
        }
    }
    if (timer_ret != ESP_OK)
    {
        vQueueDelete(s_encoder.queue);
        s_encoder.queue = NULL;
        return timer_ret;
    }

    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        esp_timer_delete(s_encoder.sw_timer);
        s_encoder.sw_timer = NULL;
        esp_timer_delete(s_encoder.long_timer);
        s_encoder.long_timer = NULL;
        vQueueDelete(s_encoder.queue);
        s_encoder.queue = NULL;
        return ret;
    }

    qdec_init(&s_encoder.qdec, gpio_get_level(s_encoder.clk_pin), gpio_get_level(s_encoder.dt_pin));
    s_encoder.sw_stable_high = gpio_get_level(s_encoder.sw_pin) != 0;

    ESP_ERROR_CHECK(gpio_isr_handler_add(s_encoder.clk_pin, encoder_rot_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(s_encoder.dt_pin, encoder_rot_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(s_encoder.sw_pin, encoder_sw_isr, NULL));

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
