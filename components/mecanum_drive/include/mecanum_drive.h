#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MECANUM_DRIVE_WHEEL_COUNT 4

typedef enum {
    MECANUM_WHEEL_FRONT_LEFT = 0,
    MECANUM_WHEEL_FRONT_RIGHT,
    MECANUM_WHEEL_REAR_LEFT,
    MECANUM_WHEEL_REAR_RIGHT,
} mecanum_wheel_t;

typedef struct {
    gpio_num_t pwm_gpio;
    gpio_num_t direction_gpio;
    gpio_num_t encoder_gpio;
    bool invert_direction;
} mecanum_wheel_config_t;

typedef struct {
    mecanum_wheel_config_t wheels[MECANUM_DRIVE_WHEEL_COUNT];
    float encoder_pulses_per_output_rev;
    uint32_t no_load_output_rpm;
    uint32_t rated_output_rpm;
    uint32_t control_period_ms;
    uint32_t command_timeout_ms;
    uint32_t max_duty_percent;
} mecanum_drive_config_t;

typedef struct {
    float commanded[MECANUM_DRIVE_WHEEL_COUNT];
    float measured_rpm[MECANUM_DRIVE_WHEEL_COUNT];
    uint32_t encoder_edges[MECANUM_DRIVE_WHEEL_COUNT];
    uint64_t encoder_total_edges[MECANUM_DRIVE_WHEEL_COUNT];
    uint32_t duty_percent[MECANUM_DRIVE_WHEEL_COUNT];
    bool reversing[MECANUM_DRIVE_WHEEL_COUNT];
} mecanum_drive_telemetry_t;

esp_err_t mecanum_drive_init(const mecanum_drive_config_t *config);
esp_err_t mecanum_drive_set_twist(float forward, float strafe, float turn);
// Diagnostic control for one logical wheel. Clears all other wheel targets.
esp_err_t mecanum_drive_set_wheel(mecanum_wheel_t wheel, float speed);
esp_err_t mecanum_drive_stop(void);
esp_err_t mecanum_drive_get_telemetry(mecanum_drive_telemetry_t *telemetry);
esp_err_t mecanum_drive_set_pid_gains(float proportional_gain, float integral_gain);
esp_err_t mecanum_drive_get_pid_gains(float *proportional_gain, float *integral_gain);

#ifdef __cplusplus
}
#endif
