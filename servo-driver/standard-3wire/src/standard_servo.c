#include "standard_servo.h"

#include <stdlib.h>

struct standard_servo_driver {
    standard_servo_config_t config;
    uint32_t period_us;
    uint32_t current_pulse_us;
};

esp_err_t standard_servo_init(const standard_servo_config_t *config,
                              standard_servo_handle_t *ret_servo)
{
    if (config == NULL || ret_servo == NULL || config->signal_gpio < 0 ||
        config->frequency_hz == 0 || config->min_pulse_us > config->max_pulse_us ||
        config->max_pulse_us == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    standard_servo_handle_t servo = calloc(1, sizeof(*servo));
    if (servo == NULL) {
        return ESP_ERR_NO_MEM;
    }
    servo->config = *config;
    servo->period_us = 1000000U / config->frequency_hz;
    if (servo->period_us == 0 || config->max_pulse_us > servo->period_us) {
        free(servo);
        return ESP_ERR_INVALID_ARG;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = config->speed_mode,
        .timer_num = config->timer_num,
        .duty_resolution = config->duty_resolution,
        .freq_hz = config->frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t channel_config = {
        .gpio_num = config->signal_gpio,
        .speed_mode = config->speed_mode,
        .channel = config->channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = config->timer_num,
        .duty = 0,
        .hpoint = 0,
    };

    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) goto fail;
    err = ledc_channel_config(&channel_config);
    if (err != ESP_OK) goto fail;

    *ret_servo = servo;
    return standard_servo_set_pulse_us(servo, config->min_pulse_us);

fail:
    free(servo);
    return err;
}

esp_err_t standard_servo_set_pulse_us(standard_servo_handle_t servo,
                                      uint32_t pulse_us)
{
    if (servo == NULL || pulse_us < servo->config.min_pulse_us ||
        pulse_us > servo->config.max_pulse_us) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint64_t max_duty = (1ULL << servo->config.duty_resolution) - 1ULL;
    const uint32_t duty = (uint32_t)((max_duty * pulse_us) / servo->period_us);
    esp_err_t err = ledc_set_duty(servo->config.speed_mode, servo->config.channel, duty);
    if (err != ESP_OK) {
        return err;
    }
    err = ledc_update_duty(servo->config.speed_mode, servo->config.channel);
    if (err == ESP_OK) {
        servo->current_pulse_us = pulse_us;
    }
    return err;
}

esp_err_t standard_servo_set_angle_deg(standard_servo_handle_t servo,
                                       int angle_deg)
{
    if (servo == NULL || angle_deg < -90 || angle_deg > 90) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t span = servo->config.max_pulse_us - servo->config.min_pulse_us;
    uint32_t pulse_us = servo->config.min_pulse_us +
                        ((uint32_t)(angle_deg + 90) * span) / 180U;
    return standard_servo_set_pulse_us(servo, pulse_us);
}
