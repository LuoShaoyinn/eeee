#include "standard_servo.h"

#include <stdlib.h>

struct standard_servo_driver {
    standard_servo_config_t config;
    uint32_t period_us;
};

esp_err_t standard_servo_init(const standard_servo_config_t *config,
                              standard_servo_handle_t *ret_servo)
{
    if (config == NULL || ret_servo == NULL || config->signal_gpio < 0 ||
        config->frequency_hz == 0 || config->min_pulse_us > config->max_pulse_us ||
        config->max_pulse_us == 0) return ESP_ERR_INVALID_ARG;
    standard_servo_handle_t servo = calloc(1, sizeof(*servo));
    if (servo == NULL) return ESP_ERR_NO_MEM;
    servo->config = *config;
    servo->period_us = 1000000U / config->frequency_hz;
    if (servo->period_us == 0 || config->max_pulse_us > servo->period_us) {
        free(servo);
        return ESP_ERR_INVALID_ARG;
    }
    const ledc_timer_config_t timer_config = {
        .speed_mode = config->speed_mode, .timer_num = config->timer_num,
        .duty_resolution = config->duty_resolution, .freq_hz = config->frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t channel_config = {
        .gpio_num = config->signal_gpio, .speed_mode = config->speed_mode,
        .channel = config->channel, .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = config->timer_num, .duty = 0, .hpoint = 0,
    };
    esp_err_t err = ledc_timer_config(&timer_config);
    if (err == ESP_OK) err = ledc_channel_config(&channel_config);
    if (err == ESP_OK) { *ret_servo = servo; return ESP_OK; }
    free(servo);
    return err;
}

esp_err_t standard_servo_set_pulse_us(standard_servo_handle_t servo,
                                      uint32_t pulse_us)
{
    if (servo == NULL || pulse_us < servo->config.min_pulse_us ||
        pulse_us > servo->config.max_pulse_us) return ESP_ERR_INVALID_ARG;
    const uint64_t max_duty = (1ULL << servo->config.duty_resolution) - 1ULL;
    const uint32_t duty = (uint32_t)((max_duty * pulse_us) / servo->period_us);
    esp_err_t err = ledc_set_duty(servo->config.speed_mode, servo->config.channel, duty);
    if (err == ESP_OK) err = ledc_update_duty(servo->config.speed_mode, servo->config.channel);
    return err;
}

esp_err_t standard_servo_disable(standard_servo_handle_t servo)
{
    if (servo == NULL) return ESP_ERR_INVALID_ARG;
    return ledc_stop(servo->config.speed_mode, servo->config.channel, 0);
}
