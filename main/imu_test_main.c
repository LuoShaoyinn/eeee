#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "jy61p_uart.h"

#define IMU_UART_NUM UART_NUM_1
#define IMU_UART_TX_GPIO 22
#define IMU_UART_RX_GPIO 21
#define IMU_UART_BAUD_RATE 9600

static void print_data(const jy61p_uart_data_t *data)
{
    switch (data->type) {
    case JY61P_FRAME_ACCEL:
        printf("IMU accel: x=%+.3f g y=%+.3f g z=%+.3f g\n",
               data->accel_g[0], data->accel_g[1], data->accel_g[2]);
        break;
    case JY61P_FRAME_GYRO:
        printf("IMU gyro:  x=%+.1f dps y=%+.1f dps z=%+.1f dps\n",
               data->gyro_dps[0], data->gyro_dps[1], data->gyro_dps[2]);
        break;
    case JY61P_FRAME_ANGLE:
        printf("IMU angle:  roll=%+.2f deg pitch=%+.2f deg yaw=%+.2f deg\n",
               data->angle_deg[0], data->angle_deg[1], data->angle_deg[2]);
        break;
    case JY61P_FRAME_MAG:
        printf("IMU mag: raw=%" PRId16 ",%" PRId16 ",%" PRId16 ",%" PRId16 "\n",
               data->raw[0], data->raw[1], data->raw[2], data->raw[3]);
        break;
    default:
        printf("IMU frame 0x%02x: raw=%" PRId16 ",%" PRId16 ",%" PRId16 ",%" PRId16 "\n",
               data->type, data->raw[0], data->raw[1], data->raw[2], data->raw[3]);
        break;
    }
}

void app_main(void)
{
    const jy61p_uart_config_t config = {
        .uart_num = IMU_UART_NUM,
        .tx_gpio = IMU_UART_TX_GPIO,
        .rx_gpio = IMU_UART_RX_GPIO,
        .baud_rate = IMU_UART_BAUD_RATE,
        .configure_magnetic_output = true,
    };
    jy61p_uart_handle_t imu;

    ESP_ERROR_CHECK(jy61p_uart_init(&config, &imu));
    printf("JY61P UART test: UART1 GPIO%d(TX)/GPIO%d(RX), %d 8N1\n",
           IMU_UART_TX_GPIO, IMU_UART_RX_GPIO, IMU_UART_BAUD_RATE);
    printf("Waiting for 0x55 IMU frames...\n");

    while (true) {
        jy61p_uart_data_t data;
        esp_err_t err = jy61p_uart_read(imu, &data, pdMS_TO_TICKS(250));
        if (err == ESP_OK) {
            print_data(&data);
        } else if (err != ESP_ERR_TIMEOUT) {
            printf("IMU read error: %s\n", esp_err_to_name(err));
        }
    }
}
