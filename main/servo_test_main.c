#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "standard_servo.h"

#define SERVO_SIGNAL_GPIO GPIO_NUM_27
#define SERVO_CENTER_US 1500
#define SERVO_MIN_TEST_US 1200
#define SERVO_MAX_TEST_US 1800
#define SERVO_STEP_US 10

void app_main(void)
{
    const standard_servo_config_t config = {
        .signal_gpio = SERVO_SIGNAL_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .channel = LEDC_CHANNEL_0,
        .frequency_hz = 50,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .min_pulse_us = SERVO_MIN_TEST_US,
        .max_pulse_us = SERVO_MAX_TEST_US,
    };
    standard_servo_handle_t servo;

    ESP_ERROR_CHECK(standard_servo_init(&config, &servo));
    ESP_ERROR_CHECK(standard_servo_set_pulse_us(servo, SERVO_CENTER_US));
    printf("Servo test started on GPIO%d at %lu Hz\n",
           SERVO_SIGNAL_GPIO, (unsigned long)config.frequency_hz);
    printf("Center pulse: %d us; test range: %d-%d us\n",
           SERVO_CENTER_US, SERVO_MIN_TEST_US, SERVO_MAX_TEST_US);

    while (true) {
        for (uint32_t pulse_us = SERVO_CENTER_US;
             pulse_us >= SERVO_MIN_TEST_US;
             pulse_us -= SERVO_STEP_US) {
            ESP_ERROR_CHECK(standard_servo_set_pulse_us(servo, pulse_us));
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        for (uint32_t pulse_us = SERVO_MIN_TEST_US;
             pulse_us <= SERVO_MAX_TEST_US;
             pulse_us += SERVO_STEP_US) {
            ESP_ERROR_CHECK(standard_servo_set_pulse_us(servo, pulse_us));
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        for (uint32_t pulse_us = SERVO_MAX_TEST_US;
             pulse_us >= SERVO_CENTER_US;
             pulse_us -= SERVO_STEP_US) {
            ESP_ERROR_CHECK(standard_servo_set_pulse_us(servo, pulse_us));
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
