#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "jga25_2430_ce.h"

#define MOTOR_PWM_GPIO GPIO_NUM_25
#define MOTOR_DIRECTION_GPIO GPIO_NUM_26
#define MOTOR_INITIAL_DUTY_PERCENT 0

void app_main(void)
{
    const jga25_2430_ce_config_t config = {
        .pwm_gpio = MOTOR_PWM_GPIO,
        .direction_gpio = MOTOR_DIRECTION_GPIO,
        .pwm_frequency_hz = 20000,
        .pwm_resolution_hz = 1000000,
        .mcpwm_group_id = 0,
    };
    jga25_2430_ce_handle_t motor;

    ESP_ERROR_CHECK(jga25_2430_ce_init(&config, &motor));
    ESP_ERROR_CHECK(jga25_2430_ce_set_direction(motor, false));
    ESP_ERROR_CHECK(jga25_2430_ce_set_duty_percent(motor, MOTOR_INITIAL_DUTY_PERCENT));

    printf("JGA25-2430-CE motor test started\n");
    printf("PWM: GPIO%d, %lu Hz, duty: %d%%\n",
           MOTOR_PWM_GPIO,
           (unsigned long)config.pwm_frequency_hz,
           MOTOR_INITIAL_DUTY_PERCENT);
    printf("Direction: GPIO%d, initial level: 0\n", MOTOR_DIRECTION_GPIO);
    printf("Change MOTOR_INITIAL_DUTY_PERCENT in motor_test_main.c to adjust speed.\n");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
