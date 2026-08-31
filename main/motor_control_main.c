#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_rom_crc.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "mecanum_drive.h"
#include "nvs_flash.h"
#include "standard_servo.h"
#include "wifi_credentials.h"

#define UDP_PORT 3333
#define COMMAND_BUFFER_SIZE 96
// Keep the radio off for Cubie UART integration. Set to 1 to restore the
// existing AP+station and UDP control path without changing command handling.
#define ROBOT_ENABLE_WIFI 0
#define ROBOT_ENABLE_PERIODIC_UART_LOGS 0
#define CUBIE_UART_NUM UART_NUM_0
#define CUBIE_UART_BAUD_RATE 115200
#define IMU_I2C_PORT I2C_NUM_0
#define IMU_I2C_SCL_GPIO GPIO_NUM_5
#define IMU_I2C_SDA_GPIO GPIO_NUM_33
#define IMU_I2C_ADDRESS 0x50
#define IMU_I2C_CLOCK_HZ 100000
#define IMU_I2C_TIMEOUT_MS 100
#define S3_SERVO_GPIO GPIO_NUM_17
#define S3_MIN_ANGLE_DEG 0
#define S3_CENTER_ANGLE_DEG 60
#define S3_MAX_ANGLE_DEG 120
#define S3_SLOW_STEP_DEG 1
#define S3_SLOW_STEP_MS 50
#define S3_PWM_FREQUENCY_HZ 50
#define S3_PWM_DUTY_RESOLUTION LEDC_TIMER_16_BIT
#define S3_PWM_PERIOD_US (1000000U / S3_PWM_FREQUENCY_HZ)
#define GA25_IN1_GPIO GPIO_NUM_12
#define GA25_IN2_GPIO GPIO_NUM_16
#define GA25_ENCODER_GPIO GPIO_NUM_14
#define GA25_PWM_FREQUENCY_HZ 20000
#define GA25_PWM_DUTY_RESOLUTION LEDC_TIMER_10_BIT
#define GA25_PWM_TIMER LEDC_TIMER_1
#define GA25_IN1_PWM_CHANNEL LEDC_CHANNEL_1
#define GA25_IN2_PWM_CHANNEL LEDC_CHANNEL_2
#define GA25_COMMAND_TIMEOUT_MS 500
#define GA25_RAMP_PERCENT_PER_TICK 2
#define GA25_ENCODER_GLITCH_FILTER_NS 10000
#define OTA_MAX_IMAGE_SIZE 0x1e0000U
#define OTA_RECEIVE_TIMEOUT_MS 5000
#define MECANUM_WHEEL_RADIUS_M 0.023f
#define MECANUM_HALF_WHEELBASE_M 0.095f
#define MECANUM_NO_LOAD_RPM 620.0f
#define TWO_PI_F 6.28318530718f
#define CUBIE_MAX_LINEAR_MPS 0.40f
#define CUBIE_MAX_YAW_RADPS 2.00f
static const char *TAG = "robot_control";
static const mecanum_wheel_t s_physical_motor_map[] = {
    MECANUM_WHEEL_REAR_RIGHT,  // M1
    MECANUM_WHEEL_REAR_LEFT,   // M2
    MECANUM_WHEEL_FRONT_LEFT,  // M3
    MECANUM_WHEEL_FRONT_RIGHT, // M4
};
typedef struct {
    bool accel_valid, gyro_valid, angle_valid;
    float accel_g[3], gyro_dps[3], angle_deg[3];
    uint32_t last_frame_ms;
    uint32_t frames;
    uint32_t i2c_transactions, i2c_errors;
} imu_telemetry_t;
static imu_telemetry_t s_imu;
static portMUX_TYPE s_imu_lock = portMUX_INITIALIZER_UNLOCKED;
static standard_servo_handle_t s3_servo;
static portMUX_TYPE s3_lock = portMUX_INITIALIZER_UNLOCKED;
static int s3_current_angle = S3_MIN_ANGLE_DEG;
static int s3_target_angle = S3_MIN_ANGLE_DEG;
static bool s3_enabled;
static bool s3_slow_move;
static portMUX_TYPE s_ga25_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_ga25_target_duty;
static int s_ga25_current_duty;
static int64_t s_ga25_last_command_us;
static pcnt_unit_handle_t s_ga25_encoder;
static uint32_t s_ga25_encoder_edges;
static uint64_t s_ga25_encoder_total_edges;

static void cubie_ota_update(const char *command);

static uint32_t s3_angle_to_pulse_us(int angle_deg) {
    return 1000U + ((uint32_t)angle_deg * 1000U) / S3_MAX_ANGLE_DEG;
}

static uint32_t s3_pulse_to_duty_ticks(uint32_t pulse_us) {
    const uint32_t max_duty = (1U << S3_PWM_DUTY_RESOLUTION) - 1U;
    return (max_duty * pulse_us) / S3_PWM_PERIOD_US;
}

static float mecanum_wheel_max_mps(void) {
    return TWO_PI_F * MECANUM_WHEEL_RADIUS_M * MECANUM_NO_LOAD_RPM / 60.0f;
}

static esp_err_t s3_set_angle(int angle_deg) {
    if (s3_servo == NULL) return ESP_ERR_INVALID_STATE;
    if (angle_deg < S3_MIN_ANGLE_DEG || angle_deg > S3_MAX_ANGLE_DEG) return ESP_ERR_INVALID_ARG;
    const uint32_t pulse_us = s3_angle_to_pulse_us(angle_deg);
    esp_err_t err = standard_servo_set_pulse_us(s3_servo, pulse_us);
    if (err == ESP_OK) { s3_current_angle = angle_deg; s3_enabled = true; }
    return err;
}

static esp_err_t s3_release(void) {
    if (s3_servo == NULL) return ESP_ERR_INVALID_STATE;
    portENTER_CRITICAL(&s3_lock); s3_slow_move = false; s3_enabled = false; portEXIT_CRITICAL(&s3_lock);
    return standard_servo_disable(s3_servo);
}

static esp_err_t s3_request_angle(int angle_deg) {
    if (angle_deg < S3_MIN_ANGLE_DEG || angle_deg > S3_MAX_ANGLE_DEG) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s3_lock); s3_target_angle = angle_deg; s3_slow_move = true; portEXIT_CRITICAL(&s3_lock);
    return ESP_OK;
}

static void s3_task(void *unused) {
    (void)unused;
    while (true) {
        int next = -1;
        portENTER_CRITICAL(&s3_lock);
        if (s3_slow_move) {
            if (s3_current_angle < s3_target_angle) next = s3_current_angle + S3_SLOW_STEP_DEG;
            else if (s3_current_angle > s3_target_angle) next = s3_current_angle - S3_SLOW_STEP_DEG;
            else s3_slow_move = false;
        }
        portEXIT_CRITICAL(&s3_lock);
        if (next >= 0) {
            if (s3_set_angle(next) != ESP_OK) ESP_LOGE(TAG, "S3 angle update failed");
        }
        vTaskDelay(pdMS_TO_TICKS(S3_SLOW_STEP_MS));
    }
}

static void ga25_apply_duty(int duty_percent) {
    const int magnitude = duty_percent < 0 ? -duty_percent : duty_percent;
    const uint32_t max_duty = (1U << GA25_PWM_DUTY_RESOLUTION) - 1U;
    const uint32_t duty = (max_duty * (uint32_t)magnitude) / 100U;
    const uint32_t in1_duty = duty_percent > 0 ? duty : 0;
    const uint32_t in2_duty = duty_percent < 0 ? duty : 0;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, GA25_IN1_PWM_CHANNEL, in1_duty));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, GA25_IN2_PWM_CHANNEL, in2_duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, GA25_IN1_PWM_CHANNEL));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, GA25_IN2_PWM_CHANNEL));
}

static esp_err_t ga25_request_duty(int duty_percent) {
    if (duty_percent < -100 || duty_percent > 100) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s_ga25_lock);
    s_ga25_target_duty = duty_percent;
    s_ga25_last_command_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_ga25_lock);
    return ESP_OK;
}

static void ga25_read_encoder(void) {
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(s_ga25_encoder, &count));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_ga25_encoder));
    portENTER_CRITICAL(&s_ga25_lock);
    s_ga25_encoder_edges = abs(count);
    s_ga25_encoder_total_edges += s_ga25_encoder_edges;
    portEXIT_CRITICAL(&s_ga25_lock);
}

static void ga25_task(void *unused) {
    (void)unused;
    while (true) {
        ga25_read_encoder();
        int target;
        portENTER_CRITICAL(&s_ga25_lock);
        target = s_ga25_target_duty;
        if (esp_timer_get_time() - s_ga25_last_command_us > (int64_t)GA25_COMMAND_TIMEOUT_MS * 1000) {
            target = s_ga25_target_duty = 0;
        }
        portEXIT_CRITICAL(&s_ga25_lock);

        // Decelerate to zero before reversing the L298N bridge direction.
        if ((s_ga25_current_duty > 0 && target < 0) ||
            (s_ga25_current_duty < 0 && target > 0)) target = 0;
        if (s_ga25_current_duty < target) {
            s_ga25_current_duty += GA25_RAMP_PERCENT_PER_TICK;
            if (s_ga25_current_duty > target) s_ga25_current_duty = target;
        } else if (s_ga25_current_duty > target) {
            s_ga25_current_duty -= GA25_RAMP_PERCENT_PER_TICK;
            if (s_ga25_current_duty < target) s_ga25_current_duty = target;
        }
        ga25_apply_duty(s_ga25_current_duty);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static int16_t imu_le_i16(const uint8_t *data) {
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static esp_err_t imu_read_registers(uint8_t reg, uint8_t *data, size_t length) {
    return i2c_master_write_read_device(IMU_I2C_PORT, IMU_I2C_ADDRESS, &reg, 1,
                                        data, length,
                                        pdMS_TO_TICKS(IMU_I2C_TIMEOUT_MS));
}

static void imu_task(void *unused) {
    (void)unused;
    const i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = IMU_I2C_SDA_GPIO,
        .scl_io_num = IMU_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = IMU_I2C_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(IMU_I2C_PORT, &config));
    ESP_ERROR_CHECK(i2c_driver_install(IMU_I2C_PORT, config.mode, 0, 0, 0));
    ESP_LOGI(TAG, "IMU I2C listening: JY61P addr 0x%02X, SCL GPIO%d, SDA GPIO%d",
             IMU_I2C_ADDRESS, IMU_I2C_SCL_GPIO, IMU_I2C_SDA_GPIO);
    while (true) {
        uint8_t accel[6], gyro[6], angle[6];
        const esp_err_t accel_err = imu_read_registers(0x34, accel, sizeof(accel));
        const esp_err_t gyro_err = accel_err == ESP_OK ? imu_read_registers(0x37, gyro, sizeof(gyro)) : accel_err;
        const esp_err_t angle_err = gyro_err == ESP_OK ? imu_read_registers(0x3D, angle, sizeof(angle)) : gyro_err;
        portENTER_CRITICAL(&s_imu_lock);
        ++s_imu.i2c_transactions;
        if (angle_err != ESP_OK) {
            ++s_imu.i2c_errors;
            portEXIT_CRITICAL(&s_imu_lock);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        for (size_t axis = 0; axis < 3; ++axis) {
            s_imu.accel_g[axis] = imu_le_i16(&accel[axis * 2]) * 16.0f / 32768.0f;
            s_imu.gyro_dps[axis] = imu_le_i16(&gyro[axis * 2]) * 2000.0f / 32768.0f;
            s_imu.angle_deg[axis] = imu_le_i16(&angle[axis * 2]) * 180.0f / 32768.0f;
        }
        s_imu.accel_valid = s_imu.gyro_valid = s_imu.angle_valid = true;
        s_imu.last_frame_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        ++s_imu.frames;
        portEXIT_CRITICAL(&s_imu_lock);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

#if ROBOT_ENABLE_WIFI
static void wifi_events(void *arg, esp_event_base_t base, int32_t event_id, void *data) {
    (void)arg;
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) { ESP_LOGW(TAG, "station disconnected; AP remains available"); esp_wifi_connect(); }
    else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) { ip_event_got_ip_t *event=data; ESP_LOGI(TAG,"station IP " IPSTR, IP2STR(&event->ip_info.ip)); }
}
static void start_wifi(void) {
    ESP_ERROR_CHECK(esp_netif_init()); ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap(); esp_netif_create_default_wifi_sta();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT(); ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_events, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_events, NULL));
    wifi_config_t ap = {.ap={.ssid=ROBOT_WIFI_AP_SSID,.ssid_len=sizeof(ROBOT_WIFI_AP_SSID)-1,.channel=1,.password=ROBOT_WIFI_AP_PASSWORD,.max_connection=2,.authmode=WIFI_AUTH_WPA2_PSK}};
    wifi_config_t sta = {.sta={.ssid=ROBOT_WIFI_STA_SSID,.password=ROBOT_WIFI_STA_PASSWORD,.threshold.authmode=WIFI_AUTH_WPA2_PSK}};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA)); ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&ap)); ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA,&sta)); ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "AP ready at 192.168.4.1, UDP/%d", UDP_PORT);
}
#endif
static const char *process_command(const char *command, char *reply, size_t reply_size) {
    if (!strcmp(command, "s3 stop") || !strcmp(command, "s3 release"))
        return s3_release() == ESP_OK ? "s3 released\n" : "error: s3\n";
    if (!strcmp(command, "s3 center"))
        return s3_request_angle(S3_CENTER_ANGLE_DEG) == ESP_OK ? "s3 moving to center\n" : "error: s3\n";
    int s3_angle; char s3_extra;
    if (sscanf(command, "s3 %d %c", &s3_angle, &s3_extra) == 1)
        return s3_request_angle(s3_angle) == ESP_OK ? "s3 moving slowly\n" : "error: S3 angle must be 0..120\n";
    if (!strcmp(command, "s3")) {
        portENTER_CRITICAL(&s3_lock);
        const int current_angle = s3_current_angle, target_angle = s3_target_angle;
        const bool enabled = s3_enabled, moving = s3_slow_move;
        portEXIT_CRITICAL(&s3_lock);
        const uint32_t current_pulse_us = s3_angle_to_pulse_us(current_angle);
        const uint32_t target_pulse_us = s3_angle_to_pulse_us(target_angle);
        const uint32_t current_duty_ticks = enabled ? s3_pulse_to_duty_ticks(current_pulse_us) : 0;
        const float current_duty_percent = enabled ?
            100.0f * (float)current_pulse_us / (float)S3_PWM_PERIOD_US : 0.0f;
        snprintf(reply, reply_size,
                 "s3 %s current %ddeg %luus %.2f%% (%lu/%u) target %ddeg %luus moving %u; commands: s3 ANGLE [0..120], s3 center, s3 release\n",
                 enabled ? "holding" : "released", current_angle,
                 (unsigned long)current_pulse_us, current_duty_percent,
                 (unsigned long)current_duty_ticks, (unsigned)((1U << S3_PWM_DUTY_RESOLUTION) - 1U),
                 target_angle, (unsigned long)target_pulse_us, moving);
        return reply;
    }
    if (!strcmp(command, "ga25")) {
        int target, current;
        uint32_t encoder_edges;
        uint64_t encoder_total;
        portENTER_CRITICAL(&s_ga25_lock);
        target = s_ga25_target_duty;
        encoder_edges = s_ga25_encoder_edges;
        encoder_total = s_ga25_encoder_total_edges;
        portEXIT_CRITICAL(&s_ga25_lock);
        current = s_ga25_current_duty;
        snprintf(reply, reply_size,
                 "ga25 target %d%% current %d%%; encoder %u edges/20ms total %llu; ga25 DUTY [-100..100], refresh within 500ms\n",
                 target, current, (unsigned)encoder_edges, (unsigned long long)encoder_total);
        return reply;
    }
    int ga25_duty; char ga25_extra;
    if (sscanf(command, "ga25 %d %c", &ga25_duty, &ga25_extra) == 1)
        return ga25_request_duty(ga25_duty) == ESP_OK ? "ok: ga25 L298N open-loop PWM\n" : "error: GA25 duty must be -100..100\n";
    if (!strcmp(command, "imu")) {
        imu_telemetry_t imu;
        portENTER_CRITICAL(&s_imu_lock); imu = s_imu; portEXIT_CRITICAL(&s_imu_lock);
        const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if (!imu.accel_valid && !imu.gyro_valid && !imu.angle_valid) {
            snprintf(reply, reply_size,
                     "imu waiting for JY61P I2C addr 0x50; transactions %lu errors %lu\n",
                     (unsigned long)imu.i2c_transactions, (unsigned long)imu.i2c_errors);
            return reply;
        }
        snprintf(reply, reply_size,
                 "imu age %lums frames %lu; i2c transactions %lu errors %lu; accel %.3f %.3f %.3f g; gyro %.1f %.1f %.1f dps; angle %.2f %.2f %.2f deg\n",
                 (unsigned long)(now_ms - imu.last_frame_ms), (unsigned long)imu.frames,
                 (unsigned long)imu.i2c_transactions, (unsigned long)imu.i2c_errors,
                 imu.accel_g[0], imu.accel_g[1], imu.accel_g[2],
                 imu.gyro_dps[0], imu.gyro_dps[1], imu.gyro_dps[2],
                 imu.angle_deg[0], imu.angle_deg[1], imu.angle_deg[2]);
        return reply;
    }
    if (!strcmp(command, "state")) {
        mecanum_drive_telemetry_t telemetry;
        imu_telemetry_t imu;
        int ga25_target, ga25_current;
        uint32_t ga25_edges;
        uint64_t ga25_total;
        const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if (mecanum_drive_get_telemetry(&telemetry) != ESP_OK) return "error: state unavailable\n";
        portENTER_CRITICAL(&s_imu_lock); imu = s_imu; portEXIT_CRITICAL(&s_imu_lock);
        portENTER_CRITICAL(&s_ga25_lock);
        ga25_target = s_ga25_target_duty;
        ga25_edges = s_ga25_encoder_edges;
        ga25_total = s_ga25_encoder_total_edges;
        portEXIT_CRITICAL(&s_ga25_lock);
        ga25_current = s_ga25_current_duty;
        snprintf(reply, reply_size,
                 "state ms %lu imu_age %lu gyro %.1f %.1f %.1f angle %.1f %.1f %.1f rpm %.0f %.0f %.0f %.0f fg %u %u %u %u ga25 %d %d %u %llu\n",
                 (unsigned long)now_ms,
                 (unsigned long)(imu.accel_valid ? now_ms - imu.last_frame_ms : UINT32_MAX),
                 imu.gyro_dps[0], imu.gyro_dps[1], imu.gyro_dps[2],
                 imu.angle_deg[0], imu.angle_deg[1], imu.angle_deg[2],
                 telemetry.measured_rpm[0], telemetry.measured_rpm[1],
                 telemetry.measured_rpm[2], telemetry.measured_rpm[3],
                 (unsigned)telemetry.encoder_edges[0], (unsigned)telemetry.encoder_edges[1],
                 (unsigned)telemetry.encoder_edges[2], (unsigned)telemetry.encoder_edges[3],
                 ga25_target, ga25_current, (unsigned)ga25_edges,
                 (unsigned long long)ga25_total);
        return reply;
    }
    if (!strcmp(command, "telemetry")) {
        mecanum_drive_telemetry_t telemetry;
        if (mecanum_drive_get_telemetry(&telemetry) != ESP_OK) return "error: telemetry unavailable\n";
        snprintf(reply, reply_size,
                 "target %.2f %.2f %.2f %.2f; rpm %.0f %.0f %.0f %.0f; fg %u %u %u %u; total %llu %llu %llu %llu; duty %u %u %u %u; reversing %u %u %u %u\n",
                 telemetry.commanded[0], telemetry.commanded[1], telemetry.commanded[2], telemetry.commanded[3],
                 telemetry.measured_rpm[0], telemetry.measured_rpm[1], telemetry.measured_rpm[2], telemetry.measured_rpm[3],
                 (unsigned)telemetry.encoder_edges[0], (unsigned)telemetry.encoder_edges[1],
                 (unsigned)telemetry.encoder_edges[2], (unsigned)telemetry.encoder_edges[3],
                 (unsigned long long)telemetry.encoder_total_edges[0],
                 (unsigned long long)telemetry.encoder_total_edges[1],
                 (unsigned long long)telemetry.encoder_total_edges[2],
                 (unsigned long long)telemetry.encoder_total_edges[3],
                 (unsigned)telemetry.duty_percent[0], (unsigned)telemetry.duty_percent[1],
                 (unsigned)telemetry.duty_percent[2], (unsigned)telemetry.duty_percent[3],
                 telemetry.reversing[0], telemetry.reversing[1], telemetry.reversing[2], telemetry.reversing[3]);
        return reply;
    }
    if (!strcmp(command,"stop")) {
        const esp_err_t motors = mecanum_drive_stop();
        const esp_err_t servo = s3_release();
        const esp_err_t ga25 = ga25_request_duty(0);
        return motors == ESP_OK && servo == ESP_OK && ga25 == ESP_OK ? "ok\n" : "error: stop failed\n";
    }
    if (!strcmp(command,"pid")) {
        float kp, ki;
        return mecanum_drive_get_pid_gains(&kp, &ki)==ESP_OK ?
            (snprintf(reply, reply_size, "pid kp %.4f ki %.4f\n", kp, ki), reply) : "error: pid unavailable\n";
    }
    float kp, ki; char pid_extra;
    if (sscanf(command,"pid %f %f %c",&kp,&ki,&pid_extra)==2)
        return mecanum_drive_set_pid_gains(kp,ki)==ESP_OK ?
            (snprintf(reply, reply_size, "pid kp %.4f ki %.4f\n", kp, ki), reply) : "error: invalid pid gains\n";
    unsigned raw_motor; int raw_duty; char raw_extra;
    if (sscanf(command,"raw %u %d %c",&raw_motor,&raw_duty,&raw_extra)==2 && raw_motor>=1 && raw_motor<=4 && raw_duty>=-100 && raw_duty<=100)
        return mecanum_drive_set_wheel_open_loop(s_physical_motor_map[raw_motor-1],raw_duty)==ESP_OK ? "ok: raw open-loop PWM; refresh within 500ms or it stops\n" : "error: raw failed\n";
    unsigned motor; float wheel_speed; char wheel_extra;
    if (sscanf(command,"wheel %u %f %c",&motor,&wheel_speed,&wheel_extra)==2 && motor>=1 && motor<=4 && wheel_speed>=-1 && wheel_speed<=1)
        return mecanum_drive_set_wheel(s_physical_motor_map[motor-1],wheel_speed)==ESP_OK ? "ok\n" : "error: wheel failed\n";
    float f,s,t; char extra;
    if (sscanf(command,"drive %f %f %f %c",&f,&s,&t,&extra)==3 && f>=-1&&f<=1&&s>=-1&&s<=1&&t>=-1&&t<=1) return mecanum_drive_set_twist(f,s,t)==ESP_OK ? "ok\n" : "error: drive failed\n";
    float vx, vy, wz;
    if (sscanf(command, "twist %f %f %f %c", &vx, &vy, &wz, &extra) == 3) {
        if (fabsf(vx) > CUBIE_MAX_LINEAR_MPS || fabsf(vy) > CUBIE_MAX_LINEAR_MPS ||
            fabsf(wz) > CUBIE_MAX_YAW_RADPS) {
            return "error: twist exceeds calibrated safety limit\n";
        }
        const float wheel_max_mps = mecanum_wheel_max_mps();
        const float forward = vx / wheel_max_mps;
        const float strafe = vy / wheel_max_mps;
        const float turn = (2.0f * MECANUM_HALF_WHEELBASE_M * wz) / wheel_max_mps;
        return mecanum_drive_set_twist(forward, strafe, turn) == ESP_OK ? "ok\n" : "error: twist failed\n";
    }
    return "error: state, imu, s3 ANGLE [0..120]|center|release, ga25 DUTY, raw M DUTY, pid [KP KI], wheel M SPEED, drive F S T, twist VX_MPS VY_MPS WZ_RADPS, telemetry, or stop\n";
}
#if ROBOT_ENABLE_WIFI
static void udp_task(void *unused) {
    (void)unused; int fd=socket(AF_INET,SOCK_DGRAM,IPPROTO_IP); if(fd<0){ESP_LOGE(TAG,"socket errno %d",errno);vTaskDelete(NULL);return;}
    struct sockaddr_in address={.sin_family=AF_INET,.sin_port=htons(UDP_PORT),.sin_addr.s_addr=htonl(INADDR_ANY)};
    if(bind(fd,(struct sockaddr *)&address,sizeof(address))<0){ESP_LOGE(TAG,"bind errno %d",errno);vTaskDelete(NULL);return;}
    while(true){ char buffer[COMMAND_BUFFER_SIZE], reply[256]; struct sockaddr_in sender; socklen_t len=sizeof(sender); int received=recvfrom(fd,buffer,sizeof(buffer)-1,0,(struct sockaddr *)&sender,&len); if(received<0)continue; buffer[received]='\0'; const char *response=process_command(buffer,reply,sizeof(reply)); sendto(fd,response,strlen(response),0,(struct sockaddr *)&sender,len); }
}
#endif
static void cubie_uart_task(void *unused) {
    (void)unused;
    const esp_err_t install = uart_driver_install(CUBIE_UART_NUM, 512, 0, 0, NULL, 0);
    if (install != ESP_OK && install != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Cubie UART driver failed: %s", esp_err_to_name(install));
        vTaskDelete(NULL);
        return;
    }
    char command[COMMAND_BUFFER_SIZE];
    size_t used = 0;
    ESP_LOGI(TAG, "Cubie UART0 command channel ready at %d baud", CUBIE_UART_BAUD_RATE);
    while (true) {
        uint8_t byte;
        if (uart_read_bytes(CUBIE_UART_NUM, &byte, 1, portMAX_DELAY) != 1) continue;
        if (byte == '\r') continue;
        if (byte == '\n') {
            if (used == 0) continue;
            command[used] = '\0';
            if (!strncmp(command, "ota ", 4)) {
                cubie_ota_update(command);
                used = 0;
                continue;
            }
            char reply[256];
            const char *response = process_command(command, reply, sizeof(reply));
            uart_write_bytes(CUBIE_UART_NUM, "@ ", 2);
            uart_write_bytes(CUBIE_UART_NUM, response, strlen(response));
            used = 0;
            continue;
        }
        if (used < sizeof(command) - 1) command[used++] = (char)byte;
        else used = 0;
    }
}
#if ROBOT_ENABLE_PERIODIC_UART_LOGS
static void telemetry_task(void *unused) {
    (void)unused; while(true) { mecanum_drive_telemetry_t t; if(mecanum_drive_get_telemetry(&t)==ESP_OK) ESP_LOGI(TAG,"rpm %.0f %.0f %.0f %.0f; duty %u %u %u %u; FG %u %u %u %u",t.measured_rpm[0],t.measured_rpm[1],t.measured_rpm[2],t.measured_rpm[3],(unsigned)t.duty_percent[0],(unsigned)t.duty_percent[1],(unsigned)t.duty_percent[2],(unsigned)t.duty_percent[3],(unsigned)t.encoder_edges[0],(unsigned)t.encoder_edges[1],(unsigned)t.encoder_edges[2],(unsigned)t.encoder_edges[3]); vTaskDelay(pdMS_TO_TICKS(1000)); }
}
#endif

static void cubie_ota_update(const char *command) {
    unsigned image_size;
    unsigned expected_crc;
    char extra;
    uint8_t buffer[512];
    if (sscanf(command, "ota %u %x %c", &image_size, &expected_crc, &extra) != 2 ||
        image_size == 0 || image_size > OTA_MAX_IMAGE_SIZE) {
        uart_write_bytes(CUBIE_UART_NUM, "@ OTA ERROR invalid command\n", 28);
        return;
    }
    const esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
    if (partition == NULL || image_size > partition->size) {
        uart_write_bytes(CUBIE_UART_NUM, "@ OTA ERROR partition\n", 22);
        return;
    }
    mecanum_drive_stop(); s3_release(); ga25_request_duty(0); ga25_apply_duty(0);
    esp_ota_handle_t handle;
    esp_err_t err = esp_ota_begin(partition, image_size, &handle);
    if (err != ESP_OK) { uart_write_bytes(CUBIE_UART_NUM, "@ OTA ERROR begin\n", 18); return; }
    uart_write_bytes(CUBIE_UART_NUM, "@ OTA READY\n", 12);
    uint32_t actual_crc = 0;
    size_t received = 0;
    while (received < image_size) {
        const size_t wanted = image_size - received < sizeof(buffer) ? image_size - received : sizeof(buffer);
        const int count = uart_read_bytes(CUBIE_UART_NUM, buffer, wanted, pdMS_TO_TICKS(OTA_RECEIVE_TIMEOUT_MS));
        if (count <= 0 || esp_ota_write(handle, buffer, count) != ESP_OK) { err = ESP_FAIL; break; }
        actual_crc = esp_rom_crc32_le(actual_crc, buffer, count); received += (size_t)count;
    }
    if (err == ESP_OK && received == image_size && actual_crc == expected_crc) {
        err = esp_ota_end(handle);
        if (err == ESP_OK) err = esp_ota_set_boot_partition(partition);
    } else { esp_ota_abort(handle); err = ESP_ERR_INVALID_CRC; }
    if (err != ESP_OK) { uart_write_bytes(CUBIE_UART_NUM, "@ OTA ERROR verify\n", 19); return; }
    uart_write_bytes(CUBIE_UART_NUM, "@ OTA OK REBOOT\n", 16);
    vTaskDelay(pdMS_TO_TICKS(100)); esp_restart();
}
void app_main(void) {
    esp_err_t err=nvs_flash_init(); if(err==ESP_ERR_NVS_NO_FREE_PAGES||err==ESP_ERR_NVS_NEW_VERSION_FOUND){ESP_ERROR_CHECK(nvs_flash_erase());err=nvs_flash_init();} ESP_ERROR_CHECK(err);
    mecanum_drive_config_t config={.wheels={
        // Physical connector map: M1 rear-right, M2 rear-left, M3 front-left, M4 front-right.
        [MECANUM_WHEEL_FRONT_LEFT]={GPIO_NUM_26,GPIO_NUM_27,GPIO_NUM_35,false}, // M3
        [MECANUM_WHEEL_FRONT_RIGHT]={GPIO_NUM_19,GPIO_NUM_13,GPIO_NUM_34,true}, // M4
        [MECANUM_WHEEL_REAR_LEFT]={GPIO_NUM_21,GPIO_NUM_25,GPIO_NUM_39,false}, // M2
        [MECANUM_WHEEL_REAR_RIGHT]={GPIO_NUM_23,GPIO_NUM_32,GPIO_NUM_36,true}, // M1
    },
        .encoder_pulses_per_output_rev=86.4f,.no_load_output_rpm=620,.rated_output_rpm=450,
        .control_period_ms=20,.command_timeout_ms=500,.max_duty_percent=100};
    ESP_ERROR_CHECK(mecanum_drive_init(&config));
    const standard_servo_config_t s3_config = {
        .signal_gpio = S3_SERVO_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0, .channel = LEDC_CHANNEL_0,
        .frequency_hz = S3_PWM_FREQUENCY_HZ, .duty_resolution = S3_PWM_DUTY_RESOLUTION,
        .min_pulse_us = 1000, .max_pulse_us = 2000,
    };
    ESP_ERROR_CHECK(standard_servo_init(&s3_config, &s3_servo));
    ESP_ERROR_CHECK(s3_set_angle(S3_MIN_ANGLE_DEG));
    const ledc_timer_config_t ga25_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE, .duty_resolution = GA25_PWM_DUTY_RESOLUTION,
        .timer_num = GA25_PWM_TIMER, .freq_hz = GA25_PWM_FREQUENCY_HZ, .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t ga25_in1_channel = {
        .gpio_num = GA25_IN1_GPIO, .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = GA25_IN1_PWM_CHANNEL, .timer_sel = GA25_PWM_TIMER, .duty = 0, .hpoint = 0,
    };
    const ledc_channel_config_t ga25_in2_channel = {
        .gpio_num = GA25_IN2_GPIO, .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = GA25_IN2_PWM_CHANNEL, .timer_sel = GA25_PWM_TIMER, .duty = 0, .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ga25_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&ga25_in1_channel));
    ESP_ERROR_CHECK(ledc_channel_config(&ga25_in2_channel));
    const pcnt_unit_config_t ga25_encoder_unit = {
        .low_limit = -32768, .high_limit = 32767, .flags.accum_count = true,
    };
    pcnt_channel_handle_t ga25_encoder_channel;
    ESP_ERROR_CHECK(pcnt_new_unit(&ga25_encoder_unit, &s_ga25_encoder));
    const pcnt_chan_config_t ga25_encoder_config = {
        .edge_gpio_num = GA25_ENCODER_GPIO, .level_gpio_num = -1,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(s_ga25_encoder, &ga25_encoder_config, &ga25_encoder_channel));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ga25_encoder_channel,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_HOLD));
    const pcnt_glitch_filter_config_t ga25_encoder_filter = {
        .max_glitch_ns = GA25_ENCODER_GLITCH_FILTER_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_ga25_encoder, &ga25_encoder_filter));
    ESP_ERROR_CHECK(pcnt_unit_enable(s_ga25_encoder));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_ga25_encoder));
    ESP_ERROR_CHECK(pcnt_unit_start(s_ga25_encoder));
    ga25_apply_duty(0);
#if ROBOT_ENABLE_WIFI
    start_wifi();
#else
    ESP_LOGI(TAG, "Wi-Fi and UDP disabled; Cubie UART0 is the control interface");
#endif
    xTaskCreate(imu_task,"imu",3072,NULL,4,NULL);
    xTaskCreate(s3_task,"s3",2048,NULL,3,NULL);
    xTaskCreate(ga25_task,"ga25",2048,NULL,3,NULL);
    xTaskCreate(cubie_uart_task,"cubie_uart",4096,NULL,4,NULL);
#if ROBOT_ENABLE_WIFI
    xTaskCreate(udp_task,"udp_control",4096,NULL,5,NULL);
#endif
#if ROBOT_ENABLE_PERIODIC_UART_LOGS
    xTaskCreate(telemetry_task,"telemetry",3072,NULL,3,NULL);
#endif
    ESP_LOGI(TAG,"chassis: 190 mm wheel-center square, 23 mm wheel radius");
}
