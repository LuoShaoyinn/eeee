#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct jy61p_uart_driver *jy61p_uart_handle_t;

typedef enum {
    JY61P_FRAME_UNKNOWN = 0,
    JY61P_FRAME_ACCEL = 0x51,
    JY61P_FRAME_GYRO = 0x52,
    JY61P_FRAME_ANGLE = 0x53,
    JY61P_FRAME_MAG = 0x54,
    JY61P_FRAME_QUATERNION = 0x59,
} jy61p_frame_type_t;

typedef struct {
    uart_port_t uart_num;
    int tx_gpio;
    int rx_gpio;
    int baud_rate;
    bool configure_magnetic_output;
} jy61p_uart_config_t;

typedef struct {
    jy61p_frame_type_t type;
    int16_t raw[4];
    float accel_g[3];
    float gyro_dps[3];
    float angle_deg[3];
} jy61p_uart_data_t;

esp_err_t jy61p_uart_init(const jy61p_uart_config_t *config,
                          jy61p_uart_handle_t *ret_driver);

esp_err_t jy61p_uart_read(jy61p_uart_handle_t driver,
                          jy61p_uart_data_t *data,
                          TickType_t timeout_ticks);

#ifdef __cplusplus
}
#endif
