#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct jga25_2430_ce_driver *jga25_2430_ce_handle_t;

typedef struct {
    gpio_num_t pwm_gpio;
    gpio_num_t direction_gpio;
    uint32_t pwm_frequency_hz;
    uint32_t pwm_resolution_hz;
    int mcpwm_group_id;
} jga25_2430_ce_config_t;

esp_err_t jga25_2430_ce_init(const jga25_2430_ce_config_t *config,
                             jga25_2430_ce_handle_t *ret_driver);

// The motor controller is active-low: 0% stops and 100% is full drive.
esp_err_t jga25_2430_ce_set_duty_percent(jga25_2430_ce_handle_t driver,
                                          uint32_t duty_percent);

esp_err_t jga25_2430_ce_set_direction(jga25_2430_ce_handle_t driver,
                                      bool reverse);

esp_err_t jga25_2430_ce_stop(jga25_2430_ce_handle_t driver);

#ifdef __cplusplus
}
#endif
