/*
 * pwm_controller.h
 * Simple Way to Control PWM. Wrapper around LEDC API.
 *
 * Adapted from ESP-IDF LEDC examples:
 * https://github.com/espressif/esp-idf/blob/master/examples/peripherals/ledc/ledc_fade/main/ledc_fade_example_main.c
 */

#ifndef PWM_CONTROLLER_H
#define PWM_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        int gpio;                         /* GPIO pin for LEDC output */
        uint32_t freq_hz;                 /* PWM frequency */
        uint32_t duty_percent;            /* initial duty 0..100 */
        ledc_channel_t channel;           /* LEDC channel to use */
        ledc_timer_t timer;               /* LEDC timer to use */
        ledc_mode_t speed_mode;           /* LEDC_LOW_SPEED_MODE or LEDC_HIGH_SPEED_MODE */
        ledc_timer_bit_t duty_resolution; /* e.g. LEDC_TIMER_13_BIT */
    } pwm_controller_config_t;

#define PWM_CONTROLLER_CONFIG_DEFAULT   \
    {.gpio = 8,                         \
     .freq_hz = 8000U,                  \
     .duty_percent = 0U,                \
     .channel = LEDC_CHANNEL_0,         \
     .timer = LEDC_TIMER_0,             \
     .speed_mode = LEDC_LOW_SPEED_MODE, \
     .duty_resolution = LEDC_TIMER_13_BIT}

    typedef struct pwm_controller pwm_controller_t; /* opaque */

    /** @brief Initialize the PWM controller with the given config (or default if NULL).
     *
     * @param cfg Pointer to configuration struct, or NULL for default config.
     * @return 0 on success, negative on error.
     */
    int pwm_controller_init(const pwm_controller_config_t *cfg);

    /**
     * @brief Set the duty cycle for the PWM controller.
     *
     * @param duty_percent Duty cycle percentage (0..100).
     * @return 0 on success, negative on error.
     */
    int pwm_controller_set_duty(uint32_t duty_percent);

    /**
     * @brief Set the duty cycle in resolution steps (0..max based on duty_resolution).
     *
     * @param duty_in_res_steps Duty cycle in resolution steps.
     * @return 0 on success, negative on error.
     */
    int pwm_controller_set_duty_in_res_steps(uint32_t duty_in_res_steps);

    /**
     * @brief Get the current duty cycle percentage for the PWM controller.
     *
     * @param duty_percent Pointer to uint32_t where the duty cycle percentage will be stored.
     * @return 0 on success, negative on error.
     */
    int pwm_controller_get_duty(uint32_t *duty_percent);

    /**
     * @brief Get the current PWM controller resolution.
     *
     * @param res Pointer to uint32_t where the resolution will be stored.
     * @return 0 on success, negative on error.
     */
    int pwm_controller_get_resolution(uint32_t *res);

#ifdef __cplusplus
}
#endif

#endif /* PWM_CONTROLLER_H */
