#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

typedef struct standard_servo_driver *standard_servo_handle_t;

typedef struct {
    gpio_num_t signal_gpio;
    ledc_mode_t speed_mode;
    ledc_timer_t timer_num;
    ledc_channel_t channel;
    ledc_timer_bit_t duty_resolution;
    uint32_t frequency_hz;
    uint32_t min_pulse_us;
    uint32_t max_pulse_us;
} standard_servo_config_t;

esp_err_t standard_servo_init(const standard_servo_config_t *config,
                              standard_servo_handle_t *ret_servo);
esp_err_t standard_servo_set_pulse_us(standard_servo_handle_t servo,
                                      uint32_t pulse_us);
esp_err_t standard_servo_disable(standard_servo_handle_t servo);
