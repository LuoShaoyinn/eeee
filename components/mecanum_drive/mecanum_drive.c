#include "mecanum_drive.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "jga25_2430_ce.h"

#define CONTROL_PERIOD_MS 20
#define COMMAND_TIMEOUT_MS 300
#define ENCODER_PPR 86.4f
#define NO_LOAD_OUTPUT_RPM 620
#define RATED_OUTPUT_RPM 450
#define MAX_DUTY_PERCENT 50
#define ENCODER_GLITCH_FILTER_NS 10000
#define REVERSE_SETTLE_WINDOWS 5
#define REVERSE_SETTLE_MS 500
#define DIRECTION_SETTLE_MS 100
#define SPEED_MEASUREMENT_PERIOD_MS 60
#define SPEED_FILTER_ALPHA .45f
#define DUTY_ACCEL_PER_TICK 2.5f
#define DUTY_DECEL_PER_TICK 4.0f
#define MIN_RUNNING_DUTY_PERCENT 18.0f
#define SPEED_KP 0.050f
#define SPEED_KI 0.008f
#define SPEED_SYNC_MIN_TARGET 0.15f
#define SPEED_SYNC_MIN_RPM 50.0f
#define SPEED_SYNC_LEAD_RATIO 1.10f
#define SPEED_SYNC_FILTER_ALPHA 0.35f

typedef enum { WHEEL_RUNNING, WHEEL_BRAKING, WHEEL_SETTLING } wheel_state_t;
typedef struct {
    jga25_2430_ce_handle_t motor;
    pcnt_unit_handle_t encoder;
    float target, controller_target, duty, wanted_duty, integral, rpm;
    uint32_t edges, speed_edges, quiet_windows;
    uint64_t total_edges;
    int sign, requested_sign;
    int64_t state_started_us, speed_sample_started_us;
    wheel_state_t state;
    bool invert_direction, open_loop;
    bool has_speed_sample;
} wheel_t;

static const char *TAG = "mecanum_drive";
static wheel_t s_wheels[MECANUM_DRIVE_WHEEL_COUNT];
static uint32_t s_period_ms, s_timeout_ms, s_no_load_rpm, s_rated_rpm, s_max_duty;
static float s_ppr;
static float s_kp = SPEED_KP, s_ki = SPEED_KI;
static float s_sync_reference_rpm;
static int64_t s_last_command_us;
static SemaphoreHandle_t s_lock;
static bool s_initialized, s_soft_stop_requested, s_speed_sync_active;

static float clamp(float v) { return v > 1 ? 1 : (v < -1 ? -1 : v); }
static int sign_of(float v) { return (v > .001f) - (v < -.001f); }
static float ramp_duty(float current, float target, float increase_step, float decrease_step) {
    if (target > current) return fminf(target, current + increase_step);
    return fmaxf(target, current - decrease_step);
}
static esp_err_t set_direction(wheel_t *wheel, int sign) {
    bool reverse = sign < 0;
    if (wheel->invert_direction) reverse = !reverse;
    return jga25_2430_ce_set_direction(wheel->motor, reverse);
}
static void apply_duty(wheel_t *wheel, float duty) {
    duty = fmaxf(0, fminf(duty, s_max_duty));
    wheel->duty = duty;
    ESP_ERROR_CHECK(jga25_2430_ce_set_duty_percent(wheel->motor, lroundf(duty)));
}
static bool read_encoder(wheel_t *wheel, int64_t now_us) {
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(wheel->encoder, &count));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(wheel->encoder));
    wheel->edges = abs(count);
    wheel->total_edges += wheel->edges;
    wheel->speed_edges += wheel->edges;
    if (!wheel->speed_sample_started_us) {
        wheel->speed_sample_started_us = now_us;
        return false;
    }
    const int64_t elapsed_us = now_us - wheel->speed_sample_started_us;
    if (elapsed_us >= (int64_t)SPEED_MEASUREMENT_PERIOD_MS * 1000) {
        const float raw_rpm = (float)wheel->speed_edges * 60000000.0f /
                              (s_ppr * (float)elapsed_us);
        wheel->rpm = wheel->has_speed_sample ?
            wheel->rpm + SPEED_FILTER_ALPHA * (raw_rpm - wheel->rpm) : raw_rpm;
        wheel->has_speed_sample = true;
        wheel->speed_edges = 0;
        wheel->speed_sample_started_us = now_us;
        return true;
    }
    return false;
}
static void update_wheel(wheel_t *wheel, bool fresh, bool speed_updated, int64_t now_us) {
    if (!fresh) {
        // A lost controller link is an immediate, coasting safety stop.
        wheel->integral = 0; wheel->controller_target = 0; wheel->wanted_duty = 0;
        wheel->requested_sign = 0; wheel->state = WHEEL_RUNNING; apply_duty(wheel, 0); return;
    }
    const int target_sign = sign_of(wheel->target);
    if (wheel->state == WHEEL_BRAKING) {
        // Do not switch DIR under load. First remove drive with a bounded
        // PWM slope, then wait for a quiet encoder before changing direction.
        if (target_sign == wheel->sign && target_sign != 0) wheel->state = WHEEL_RUNNING;
        else {
            wheel->requested_sign = target_sign;
            wheel->integral = 0; wheel->wanted_duty = 0;
            apply_duty(wheel, ramp_duty(wheel->duty, 0, DUTY_ACCEL_PER_TICK, DUTY_DECEL_PER_TICK));
            if (wheel->duty > .5f) { wheel->quiet_windows = 0; return; }
            if (!wheel->requested_sign) {
                wheel->sign = 0; wheel->state = WHEEL_RUNNING; return;
            }
            if (!wheel->open_loop) {
                wheel->quiet_windows = wheel->edges ? 0 : wheel->quiet_windows + 1;
            }
            const bool settled = wheel->open_loop ?
                (now_us - wheel->state_started_us) / 1000 >= REVERSE_SETTLE_MS :
                wheel->quiet_windows >= REVERSE_SETTLE_WINDOWS &&
                (now_us - wheel->state_started_us) / 1000 >= REVERSE_SETTLE_MS;
            if (settled) {
                ESP_ERROR_CHECK(set_direction(wheel, wheel->requested_sign));
                wheel->sign = wheel->requested_sign;
                wheel->state = WHEEL_SETTLING;
                wheel->state_started_us = now_us;
            }
            return;
        }
    }
    if (!target_sign) {
        // A completed coast-to-stop must remain idle. Without this guard an
        // already stopped wheel repeatedly re-entered WHEEL_BRAKING on every
        // 20 ms tick, making telemetry report a false reversal state.
        if (wheel->sign == 0 && wheel->duty <= .5f && wheel->state == WHEEL_RUNNING) return;
        wheel->state = WHEEL_BRAKING; wheel->requested_sign = 0;
        wheel->state_started_us = now_us; wheel->quiet_windows = 0;
        return;
    }
    if (wheel->state == WHEEL_RUNNING && wheel->sign && target_sign != wheel->sign) {
        wheel->state = WHEEL_BRAKING; wheel->requested_sign = target_sign; wheel->quiet_windows = 0;
        wheel->state_started_us = now_us; wheel->integral = 0;
        return;
    }
    if (wheel->state == WHEEL_SETTLING) {
        apply_duty(wheel, 0);
        if ((now_us - wheel->state_started_us) / 1000 < DIRECTION_SETTLE_MS) return;
        wheel->state = WHEEL_RUNNING;
    }
    if (wheel->sign != target_sign) { ESP_ERROR_CHECK(set_direction(wheel, target_sign)); wheel->sign = target_sign; wheel->integral = 0; }
    if (wheel->open_loop) {
        wheel->integral = 0;
        wheel->wanted_duty = fabsf(wheel->target) * s_max_duty;
        apply_duty(wheel, ramp_duty(wheel->duty, wheel->wanted_duty,
                                    DUTY_ACCEL_PER_TICK, DUTY_DECEL_PER_TICK));
        return;
    }
    float target_rpm = fabsf(wheel->target) * s_no_load_rpm;
    // Preserve the requested kinematic wheel ratios when traction, load, or a
    // weak motor makes one wheel lag. The lagging wheel keeps its original
    // target so its PID can catch up; only wheels that lead it are reduced.
    const float normalized_rpm = wheel->rpm / fabsf(wheel->target);
    if (s_speed_sync_active && fabsf(wheel->target) >= SPEED_SYNC_MIN_TARGET &&
        normalized_rpm > s_sync_reference_rpm * SPEED_SYNC_LEAD_RATIO) {
        target_rpm = fminf(target_rpm, s_sync_reference_rpm * fabsf(wheel->target));
    }
    const float feedforward = fabsf(wheel->target) * s_max_duty;
    if (fabsf(wheel->target - wheel->controller_target) > .001f) {
        wheel->controller_target = wheel->target;
        wheel->integral = 0;
        wheel->wanted_duty = fmaxf(MIN_RUNNING_DUTY_PERCENT, feedforward);
    }
    if (speed_updated) {
        const float error = target_rpm - wheel->rpm;
        const float sample_seconds = SPEED_MEASUREMENT_PERIOD_MS / 1000.0f;
        wheel->integral = fmaxf(-100, fminf(100, wheel->integral + error * sample_seconds));
        wheel->wanted_duty = feedforward + s_kp * error + s_ki * wheel->integral;
        if (wheel->rpm < target_rpm * .15f) wheel->wanted_duty = fmaxf(wheel->wanted_duty, MIN_RUNNING_DUTY_PERCENT);
        wheel->wanted_duty = fmaxf(0, fminf(wheel->wanted_duty, s_max_duty));
    }
    const float ramped = ramp_duty(wheel->duty, wheel->wanted_duty,
                                   DUTY_ACCEL_PER_TICK, DUTY_DECEL_PER_TICK);
    apply_duty(wheel, ramped);
}
static void update_speed_sync_reference(const bool speed_updated[MECANUM_DRIVE_WHEEL_COUNT]) {
    bool sampled = false;
    float slowest_normalized_rpm = INFINITY;
    size_t eligible_wheels = 0;
    for (size_t i = 0; i < MECANUM_DRIVE_WHEEL_COUNT; ++i) {
        const wheel_t *wheel = &s_wheels[i];
        if (speed_updated[i]) sampled = true;
        if (!wheel->open_loop && wheel->has_speed_sample &&
            fabsf(wheel->target) >= SPEED_SYNC_MIN_TARGET && wheel->state == WHEEL_RUNNING) {
            slowest_normalized_rpm = fminf(slowest_normalized_rpm,
                                           wheel->rpm / fabsf(wheel->target));
            ++eligible_wheels;
        }
    }
    if (eligible_wheels < 2 || !sampled || slowest_normalized_rpm < SPEED_SYNC_MIN_RPM) {
        s_speed_sync_active = false;
        return;
    }
    s_sync_reference_rpm = s_speed_sync_active ?
        s_sync_reference_rpm + SPEED_SYNC_FILTER_ALPHA *
            (slowest_normalized_rpm - s_sync_reference_rpm) : slowest_normalized_rpm;
    s_speed_sync_active = true;
}
static void control_task(void *unused) {
    (void)unused;
    while (true) {
        int64_t now_us = esp_timer_get_time();
        xSemaphoreTake(s_lock, portMAX_DELAY);
        // s_last_command_us is 64-bit on a 32-bit target. Read it while the
        // UDP task is excluded so a torn read cannot falsely time out motion.
        bool fresh = s_soft_stop_requested ||
                     now_us - s_last_command_us <= (int64_t)s_timeout_ms * 1000;
        bool speed_updated[MECANUM_DRIVE_WHEEL_COUNT] = {false};
        for (size_t i = 0; i < MECANUM_DRIVE_WHEEL_COUNT; ++i)
            speed_updated[i] = !s_wheels[i].open_loop && read_encoder(&s_wheels[i], now_us);
        update_speed_sync_reference(speed_updated);
        for (size_t i = 0; i < MECANUM_DRIVE_WHEEL_COUNT; ++i)
            update_wheel(&s_wheels[i], fresh, speed_updated[i], now_us);
        xSemaphoreGive(s_lock);
        vTaskDelay(pdMS_TO_TICKS(s_period_ms));
    }
}
esp_err_t mecanum_drive_init(const mecanum_drive_config_t *config) {
    if (!config || s_initialized) return ESP_ERR_INVALID_STATE;
    s_period_ms = config->control_period_ms ?: CONTROL_PERIOD_MS; s_timeout_ms = config->command_timeout_ms ?: COMMAND_TIMEOUT_MS;
    s_ppr = config->encoder_pulses_per_output_rev > 0 ? config->encoder_pulses_per_output_rev : ENCODER_PPR;
    s_no_load_rpm = config->no_load_output_rpm ?: NO_LOAD_OUTPUT_RPM;
    s_rated_rpm = config->rated_output_rpm ?: RATED_OUTPUT_RPM;
    s_max_duty = config->max_duty_percent ?: MAX_DUTY_PERCENT;
    if (s_period_ms < 20 || s_max_duty > 100) return ESP_ERR_INVALID_ARG;
    s_lock = xSemaphoreCreateMutex(); if (!s_lock) return ESP_ERR_NO_MEM;
    for (size_t i = 0; i < MECANUM_DRIVE_WHEEL_COUNT; ++i) {
        const mecanum_wheel_config_t *cfg = &config->wheels[i]; wheel_t *wheel = &s_wheels[i];
        jga25_2430_ce_config_t motor = {cfg->pwm_gpio, cfg->direction_gpio, 20000, 1000000, i / 2};
        ESP_RETURN_ON_ERROR(jga25_2430_ce_init(&motor, &wheel->motor), TAG, "motor");
        pcnt_unit_config_t unit = {.low_limit=-32768, .high_limit=32767, .flags.accum_count=true};
        ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit, &wheel->encoder), TAG, "pcnt");
        pcnt_chan_config_t channel_config = {.edge_gpio_num=cfg->encoder_gpio, .level_gpio_num=-1}; pcnt_channel_handle_t channel;
        ESP_RETURN_ON_ERROR(pcnt_new_channel(wheel->encoder, &channel_config, &channel), TAG, "channel");
        ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD), TAG, "edge");
        // The ESP32 PCNT peripheral counts FG edges in hardware. Reject short
        // cable/EMI spikes while retaining the millisecond-scale real FG pulses.
        pcnt_glitch_filter_config_t filter = {.max_glitch_ns=ENCODER_GLITCH_FILTER_NS};
        ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(wheel->encoder, &filter), TAG, "filter");
        ESP_RETURN_ON_ERROR(pcnt_unit_enable(wheel->encoder), TAG, "enable");
        ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(wheel->encoder), TAG, "clear");
        ESP_RETURN_ON_ERROR(pcnt_unit_start(wheel->encoder), TAG, "start");
        wheel->invert_direction = cfg->invert_direction;
    }
    s_initialized = true; s_last_command_us = 0; s_soft_stop_requested = true;
    if (xTaskCreate(control_task, "drive_control", 4096, NULL, 8, NULL) != pdPASS) return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "ready: %ums, %.1f PPR, %u RPM no-load, %u RPM rated, %u%% cap",
             (unsigned)s_period_ms, (double)s_ppr, (unsigned)s_no_load_rpm,
             (unsigned)s_rated_rpm, (unsigned)s_max_duty);
    return ESP_OK;
}
esp_err_t mecanum_drive_set_twist(float forward, float strafe, float turn) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    float raw[] = {forward-strafe-turn, forward+strafe+turn, forward+strafe-turn, forward-strafe+turn}; float scale=1;
    for (size_t i=0;i<MECANUM_DRIVE_WHEEL_COUNT;i++) scale=fmaxf(scale,fabsf(raw[i]));
    xSemaphoreTake(s_lock,portMAX_DELAY);
    bool targets_changed = false;
    for(size_t i=0;i<MECANUM_DRIVE_WHEEL_COUNT;i++) {
        const float target = clamp(raw[i]/scale);
        targets_changed |= s_wheels[i].open_loop || fabsf(s_wheels[i].target - target) > .001f;
        s_wheels[i].target=target;
        s_wheels[i].open_loop=false;
    }
    if (targets_changed) s_speed_sync_active = false;
    s_soft_stop_requested = forward == 0 && strafe == 0 && turn == 0;
    s_last_command_us=esp_timer_get_time();
    xSemaphoreGive(s_lock); return ESP_OK;
}
esp_err_t mecanum_drive_set_wheel(mecanum_wheel_t wheel, float speed) {
    if (!s_initialized || wheel >= MECANUM_DRIVE_WHEEL_COUNT || speed < -1 || speed > 1) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (size_t i = 0; i < MECANUM_DRIVE_WHEEL_COUNT; ++i) {
        s_wheels[i].target = 0;
        s_wheels[i].open_loop = false;
    }
    s_wheels[wheel].target = speed;
    s_speed_sync_active = false;
    s_soft_stop_requested = speed == 0;
    s_last_command_us = esp_timer_get_time();
    xSemaphoreGive(s_lock);
    return ESP_OK;
}
esp_err_t mecanum_drive_set_wheel_open_loop(mecanum_wheel_t wheel, int duty_percent) {
    if (!s_initialized || wheel >= MECANUM_DRIVE_WHEEL_COUNT ||
        duty_percent < -100 || duty_percent > 100) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (size_t i = 0; i < MECANUM_DRIVE_WHEEL_COUNT; ++i) {
        s_wheels[i].target = 0;
        s_wheels[i].open_loop = false;
    }
    s_wheels[wheel].target = duty_percent / 100.0f;
    s_wheels[wheel].open_loop = duty_percent != 0;
    s_speed_sync_active = false;
    s_soft_stop_requested = duty_percent == 0;
    s_last_command_us = esp_timer_get_time();
    xSemaphoreGive(s_lock);
    return ESP_OK;
}
esp_err_t mecanum_drive_stop(void) { return mecanum_drive_set_twist(0,0,0); }
esp_err_t mecanum_drive_set_pid_gains(float proportional_gain, float integral_gain) {
    if (!s_initialized || proportional_gain < 0 || proportional_gain > 1 || integral_gain < 0 || integral_gain > 1) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_kp = proportional_gain;
    s_ki = integral_gain;
    for (size_t i = 0; i < MECANUM_DRIVE_WHEEL_COUNT; ++i) s_wheels[i].integral = 0;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}
esp_err_t mecanum_drive_get_pid_gains(float *proportional_gain, float *integral_gain) {
    if (!s_initialized || !proportional_gain || !integral_gain) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *proportional_gain = s_kp;
    *integral_gain = s_ki;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}
esp_err_t mecanum_drive_get_telemetry(mecanum_drive_telemetry_t *out) {
    if (!s_initialized || !out) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for(size_t i=0;i<MECANUM_DRIVE_WHEEL_COUNT;i++) { out->commanded[i]=s_wheels[i].target; out->measured_rpm[i]=s_wheels[i].rpm; out->encoder_edges[i]=s_wheels[i].edges; out->encoder_total_edges[i]=s_wheels[i].total_edges; out->duty_percent[i]=lroundf(s_wheels[i].duty); out->reversing[i]=s_wheels[i].state!=WHEEL_RUNNING; }
    out->sync_reference_rpm = s_sync_reference_rpm;
    out->speed_sync_active = s_speed_sync_active;
    xSemaphoreGive(s_lock); return ESP_OK;
}
