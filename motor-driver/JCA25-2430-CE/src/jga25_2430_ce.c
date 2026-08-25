#include "jga25_2430_ce.h"

#include <stdlib.h>

#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"

#define DEFAULT_PWM_FREQUENCY_HZ 20000
#define DEFAULT_PWM_RESOLUTION_HZ 1000000

struct jga25_2430_ce_driver {
    gpio_num_t direction_gpio;
    uint32_t pwm_period_ticks;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
};

esp_err_t jga25_2430_ce_init(const jga25_2430_ce_config_t *config,
                             jga25_2430_ce_handle_t *ret_driver)
{
    if (config == NULL || ret_driver == NULL || config->pwm_gpio < 0 ||
        config->direction_gpio < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t frequency_hz = config->pwm_frequency_hz;
    uint32_t resolution_hz = config->pwm_resolution_hz;
    if (frequency_hz == 0) frequency_hz = DEFAULT_PWM_FREQUENCY_HZ;
    if (resolution_hz == 0 || resolution_hz < frequency_hz) {
        resolution_hz = DEFAULT_PWM_RESOLUTION_HZ;
    }

    jga25_2430_ce_handle_t driver = calloc(1, sizeof(*driver));
    if (driver == NULL) return ESP_ERR_NO_MEM;

    driver->direction_gpio = config->direction_gpio;
    driver->pwm_period_ticks = resolution_hz / frequency_hz;
    if (driver->pwm_period_ticks < 2) {
        free(driver);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = gpio_set_direction(config->direction_gpio, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) goto fail;
    err = gpio_set_level(config->direction_gpio, 0);
    if (err != ESP_OK) goto fail;

    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    const mcpwm_timer_config_t timer_config = {
        .group_id = config->mcpwm_group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = resolution_hz,
        .period_ticks = driver->pwm_period_ticks,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    const mcpwm_operator_config_t operator_config = {
        .group_id = config->mcpwm_group_id,
    };
    const mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    const mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = config->pwm_gpio,
    };

    err = mcpwm_new_timer(&timer_config, &timer);
    if (err != ESP_OK) goto fail;
    err = mcpwm_new_operator(&operator_config, &oper);
    if (err != ESP_OK) goto fail;
    err = mcpwm_new_comparator(oper, &comparator_config, &driver->comparator);
    if (err != ESP_OK) goto fail;
    err = mcpwm_new_generator(oper, &generator_config, &driver->generator);
    if (err != ESP_OK) goto fail;
    err = mcpwm_operator_connect_timer(oper, timer);
    if (err != ESP_OK) goto fail;
    err = mcpwm_comparator_set_compare_value(driver->comparator, 1);
    if (err != ESP_OK) goto fail;
    err = mcpwm_generator_set_action_on_timer_event(
        driver->generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_LOW));
    if (err != ESP_OK) goto fail;
    err = mcpwm_generator_set_action_on_compare_event(
        driver->generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       driver->comparator,
                                       MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK) goto fail;
    err = mcpwm_timer_enable(timer);
    if (err != ESP_OK) goto fail;
    err = mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
    if (err != ESP_OK) goto fail;

    *ret_driver = driver;
    return jga25_2430_ce_stop(driver);

fail:
    free(driver);
    return err;
}

esp_err_t jga25_2430_ce_set_duty_percent(jga25_2430_ce_handle_t driver,
                                          uint32_t duty_percent)
{
    if (driver == NULL || duty_percent > 100) return ESP_ERR_INVALID_ARG;
    if (duty_percent == 0) {
        return mcpwm_generator_set_force_level(driver->generator, 1, true);
    }

    uint32_t compare_ticks = (driver->pwm_period_ticks * duty_percent) / 100;
    if (compare_ticks == 0) compare_ticks = 1;
    if (compare_ticks >= driver->pwm_period_ticks) {
        compare_ticks = driver->pwm_period_ticks - 1;
    }

    esp_err_t err = mcpwm_generator_set_force_level(driver->generator, -1, true);
    if (err != ESP_OK) return err;
    return mcpwm_comparator_set_compare_value(driver->comparator, compare_ticks);
}

esp_err_t jga25_2430_ce_set_direction(jga25_2430_ce_handle_t driver,
                                      bool reverse)
{
    if (driver == NULL) return ESP_ERR_INVALID_ARG;
    return gpio_set_level(driver->direction_gpio, reverse ? 1 : 0);
}

esp_err_t jga25_2430_ce_stop(jga25_2430_ce_handle_t driver)
{
    return jga25_2430_ce_set_duty_percent(driver, 0);
}
