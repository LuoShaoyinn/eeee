#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_SIGNAL_GPIO GPIO_NUM_27
#define SERVO_FREQUENCY_HZ 50
#define SERVO_PERIOD_US 20000
#define SERVO_CENTER_US 1500
#define SERVO_MIN_TEST_US 1200
#define SERVO_MAX_TEST_US 1800
#define SERVO_STEP_US 10

static void servo_set_pulse_us(uint32_t pulse_us)
{
    const uint32_t max_duty = (1U << LEDC_TIMER_16_BIT) - 1U;
    const uint32_t duty = (max_duty * pulse_us) / SERVO_PERIOD_US;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void servo_init(void)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = SERVO_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t channel_config = {
        .gpio_num = SERVO_SIGNAL_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

void app_main(void)
{
    servo_init();
    servo_set_pulse_us(SERVO_CENTER_US);
    printf("Servo test started on GPIO%d at %d Hz\n", SERVO_SIGNAL_GPIO, SERVO_FREQUENCY_HZ);
    printf("Center pulse: %d us; test range: %d-%d us\n",
           SERVO_CENTER_US, SERVO_MIN_TEST_US, SERVO_MAX_TEST_US);

    while (true) {
        for (uint32_t pulse_us = SERVO_CENTER_US;
             pulse_us >= SERVO_MIN_TEST_US;
             pulse_us -= SERVO_STEP_US) {
            servo_set_pulse_us(pulse_us);
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        for (uint32_t pulse_us = SERVO_MIN_TEST_US;
             pulse_us <= SERVO_MAX_TEST_US;
             pulse_us += SERVO_STEP_US) {
            servo_set_pulse_us(pulse_us);
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        for (uint32_t pulse_us = SERVO_MAX_TEST_US;
             pulse_us >= SERVO_CENTER_US;
             pulse_us -= SERVO_STEP_US) {
            servo_set_pulse_us(pulse_us);
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
