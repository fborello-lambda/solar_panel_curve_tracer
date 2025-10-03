#include "pwm_controller.h"

/* Instance-based controller implementation */
struct pwm_controller
{
    pwm_controller_config_t cfg;
    uint32_t duty_percent;
};

static pwm_controller_t s_pwm_controller;

int pwm_controller_init(const pwm_controller_config_t *cfg)
{
    pwm_controller_t *pc = &s_pwm_controller;
    if (!pc)
        return -1;

    if (cfg != NULL)
        pc->cfg = *cfg;
    else
    {
        pc->cfg = (pwm_controller_config_t)PWM_CONTROLLER_CONFIG_DEFAULT;
    }

    pc->duty_percent = pc->cfg.duty_percent;

    /* configure timer using provided config */
    ledc_timer_config_t tc = {
        .duty_resolution = pc->cfg.duty_resolution,
        .freq_hz = (int)pc->cfg.freq_hz,
        .speed_mode = pc->cfg.speed_mode,
        .timer_num = pc->cfg.timer,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&tc) != ESP_OK)
    {
        return -2;
    }

    ledc_channel_config_t ch = {
        .channel = pc->cfg.channel,
        .duty = 0,
        .gpio_num = pc->cfg.gpio,
        .speed_mode = pc->cfg.speed_mode,
        .hpoint = 0,
        .timer_sel = pc->cfg.timer,
    };
    if (ledc_channel_config(&ch) != ESP_OK)
    {
        return -3;
    }

    /* apply initial duty */
    pwm_controller_set_duty(pc->duty_percent);
    return 0;
}

int pwm_controller_get_resolution(uint32_t *res)
{
    pwm_controller_t *pc = &s_pwm_controller;
    if (!pc || !res)
        return -1;
    *res = (1ULL << pc->cfg.duty_resolution) - 1ULL;
    return 0;
}

int pwm_controller_set_duty(uint32_t duty_percent)
{
    pwm_controller_t *pc = &s_pwm_controller;

    if (duty_percent > 100U)
        duty_percent = 100U;
    pc->duty_percent = duty_percent;

    const uint32_t bits = pc->cfg.duty_resolution;
    if (bits == 0 || bits > 31) // guard against invalid/UB shift
        return -1;

    uint64_t max = ((1ULL << bits) - 1ULL);
    uint32_t duty = (uint32_t)((((uint64_t)pc->duty_percent * max) + 50ULL) / 100ULL); // rounded

    if (ledc_set_duty(pc->cfg.speed_mode, pc->cfg.channel, duty) != ESP_OK)
        return -1;
    if (ledc_update_duty(pc->cfg.speed_mode, pc->cfg.channel) != ESP_OK)
        return -2;
    return 0;
}

int pwm_controller_set_duty_in_res_steps(uint32_t duty_in_res_steps)
{
    pwm_controller_t *pc = &s_pwm_controller;

    if (!pc)
        return -1;

    const uint32_t bits = pc->cfg.duty_resolution;
    if (bits == 0 || bits > 31) // guard against invalid/UB shift
        return -2;

    uint32_t max = ((1U << bits) - 1U);
    if (duty_in_res_steps > max)
        duty_in_res_steps = max;

    pc->duty_percent = (duty_in_res_steps * 100U) / max;

    if (ledc_set_duty(pc->cfg.speed_mode, pc->cfg.channel, duty_in_res_steps) != ESP_OK)
        return -3;
    if (ledc_update_duty(pc->cfg.speed_mode, pc->cfg.channel) != ESP_OK)
        return -4;
    return 0;
}

int pwm_controller_get_duty(uint32_t *duty_percent)
{
    pwm_controller_t *pc = &s_pwm_controller;

    // ledc_get_duty(pc->cfg.speed_mode, pc->cfg.channel);

    if (!pc || !duty_percent)
        return -1;
    *duty_percent = pc->duty_percent;
    return 0;
}
