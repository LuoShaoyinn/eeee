#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "jga25_2430_ce.h"

// Carrier mapping: M1 PWM is active-low and M1 direction is a separate GPIO.
#define M1_PWM_GPIO GPIO_NUM_23
#define M1_DIRECTION_GPIO GPIO_NUM_32
#define M1_TEST_DUTY_PERCENT 20
#define M1_TEST_RUN_TIME_MS 5000

void app_main(void)
{
    const jga25_2430_ce_config_t config = {
        .pwm_gpio = M1_PWM_GPIO,
        .direction_gpio = M1_DIRECTION_GPIO,
        .pwm_frequency_hz = 20000,
        .pwm_resolution_hz = 1000000,
        .mcpwm_group_id = 0,
    };
    jga25_2430_ce_handle_t motor;

    ESP_ERROR_CHECK(jga25_2430_ce_init(&config, &motor));
    ESP_ERROR_CHECK(jga25_2430_ce_set_direction(motor, false));
    ESP_ERROR_CHECK(jga25_2430_ce_stop(motor));

    printf("M1 motor test ready: PWM GPIO%d, DIR GPIO%d, stopped\n",
           M1_PWM_GPIO, M1_DIRECTION_GPIO);
    printf("Starting %d%% duty in 2 seconds; the motor will stop after %d ms.\n",
           M1_TEST_DUTY_PERCENT, M1_TEST_RUN_TIME_MS);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_ERROR_CHECK(jga25_2430_ce_set_duty_percent(motor,
                                                    M1_TEST_DUTY_PERCENT));
    printf("M1 running at %d%% duty\n", M1_TEST_DUTY_PERCENT);
    vTaskDelay(pdMS_TO_TICKS(M1_TEST_RUN_TIME_MS));

    ESP_ERROR_CHECK(jga25_2430_ce_stop(motor));
    printf("M1 stopped\n");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
