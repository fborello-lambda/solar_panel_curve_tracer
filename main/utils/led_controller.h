#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    } led_color_t;

    typedef struct
    {
        gpio_num_t gpio;
        uint32_t delay_ms;
        led_color_t color;
    } led_blink_cfg_t;

    esp_err_t led_init(gpio_num_t gpio);
    esp_err_t led_set_color(gpio_num_t gpio, led_color_t color);
    esp_err_t led_clear(gpio_num_t gpio);

#ifdef __cplusplus
}
#endif

#endif /* LED_CONTROLLER_H */
