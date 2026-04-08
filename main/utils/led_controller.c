#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "led_strip.h"

#include "led_controller.h"

static const char *TAG = "LED_CONTROLLER";

#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)
#define LED_STRIP_MAX_INSTANCES 4

static SemaphoreHandle_t s_led_mtx = NULL;

typedef struct
{
    bool used;
    gpio_num_t gpio;
    led_strip_handle_t strip;
} led_strip_instance_t;

static led_strip_instance_t s_instances[LED_STRIP_MAX_INSTANCES] = {0};

static esp_err_t ensure_mutex(void)
{
    if (s_led_mtx != NULL)
    {
        return ESP_OK;
    }

    s_led_mtx = xSemaphoreCreateMutex();
    return (s_led_mtx == NULL) ? ESP_ERR_NO_MEM : ESP_OK;
}

static int find_instance_index_locked(gpio_num_t gpio)
{
    for (int i = 0; i < LED_STRIP_MAX_INSTANCES; i++)
    {
        if (s_instances[i].used && s_instances[i].gpio == gpio)
        {
            return i;
        }
    }

    return -1;
}

static int alloc_instance_index_locked(void)
{
    for (int i = 0; i < LED_STRIP_MAX_INSTANCES; i++)
    {
        if (!s_instances[i].used)
        {
            return i;
        }
    }

    return -1;
}

esp_err_t led_init(gpio_num_t gpio)
{
    if (gpio < 0 || gpio >= GPIO_NUM_MAX)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ensure_mutex();
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (xSemaphoreTake(s_led_mtx, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    int idx = find_instance_index_locked(gpio);
    if (idx >= 0)
    {
        xSemaphoreGive(s_led_mtx);
        return ESP_OK;
    }

    idx = alloc_instance_index_locked();
    if (idx < 0)
    {
        xSemaphoreGive(s_led_mtx);
        ESP_LOGE(TAG, "No free led_strip instance slots");
        return ESP_ERR_NO_MEM;
    }

    led_strip_config_t strip_config = {
        .strip_gpio_num = gpio,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
        .mem_block_symbols = 0,
        .flags = {
            .with_dma = false,
        },
    };

    led_strip_handle_t strip = NULL;
    ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &strip);
    if (ret == ESP_OK)
    {
        ret = led_strip_clear(strip);
    }

    if (ret == ESP_OK)
    {
        s_instances[idx].used = true;
        s_instances[idx].gpio = gpio;
        s_instances[idx].strip = strip;
    }
    else if (strip != NULL)
    {
        led_strip_del(strip);
    }

    xSemaphoreGive(s_led_mtx);
    return ret;
}

esp_err_t led_set_color(gpio_num_t gpio, led_color_t color)
{
    esp_err_t ret = led_init(gpio);
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (xSemaphoreTake(s_led_mtx, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    int idx = find_instance_index_locked(gpio);
    if (idx < 0)
    {
        xSemaphoreGive(s_led_mtx);
        return ESP_ERR_NOT_FOUND;
    }

    ret = led_strip_set_pixel(s_instances[idx].strip, 0, color.r, color.g, color.b);
    if (ret == ESP_OK)
    {
        ret = led_strip_refresh(s_instances[idx].strip);
    }

    xSemaphoreGive(s_led_mtx);
    return ret;
}

esp_err_t led_clear(gpio_num_t gpio)
{
    esp_err_t ret = led_init(gpio);
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (xSemaphoreTake(s_led_mtx, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    int idx = find_instance_index_locked(gpio);
    if (idx < 0)
    {
        xSemaphoreGive(s_led_mtx);
        return ESP_ERR_NOT_FOUND;
    }

    ret = led_strip_clear(s_instances[idx].strip);
    xSemaphoreGive(s_led_mtx);
    return ret;
}
