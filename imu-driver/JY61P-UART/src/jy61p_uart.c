#include "jy61p_uart.h"

#include <stdlib.h>
#include <string.h>

#define JY61P_FRAME_LENGTH 11
#define JY61P_RX_BUFFER_SIZE 512

struct jy61p_uart_driver {
    uart_port_t uart_num;
    uint8_t frame[JY61P_FRAME_LENGTH];
    size_t used;
    jy61p_uart_stats_t stats;
};

static int16_t le_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool valid_frame(const uint8_t *frame)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < JY61P_FRAME_LENGTH - 1; ++i) checksum += frame[i];
    return frame[0] == 0x55 && checksum == frame[JY61P_FRAME_LENGTH - 1];
}

static void decode_frame(const uint8_t *frame, jy61p_uart_data_t *data)
{
    const int16_t x = le_i16(&frame[2]);
    const int16_t y = le_i16(&frame[4]);
    const int16_t z = le_i16(&frame[6]);
    memset(data, 0, sizeof(*data));
    data->type = (jy61p_frame_type_t)frame[1];
    data->raw[0] = x;
    data->raw[1] = y;
    data->raw[2] = z;
    data->raw[3] = le_i16(&frame[8]);
    if (data->type == JY61P_FRAME_ACCEL) {
        data->accel_g[0] = x / 32768.0f * 16.0f;
        data->accel_g[1] = y / 32768.0f * 16.0f;
        data->accel_g[2] = z / 32768.0f * 16.0f;
    } else if (data->type == JY61P_FRAME_GYRO) {
        data->gyro_dps[0] = x / 32768.0f * 2000.0f;
        data->gyro_dps[1] = y / 32768.0f * 2000.0f;
        data->gyro_dps[2] = z / 32768.0f * 2000.0f;
    } else if (data->type == JY61P_FRAME_ANGLE) {
        data->angle_deg[0] = x / 32768.0f * 180.0f;
        data->angle_deg[1] = y / 32768.0f * 180.0f;
        data->angle_deg[2] = z / 32768.0f * 180.0f;
    }
}

esp_err_t jy61p_uart_init(const jy61p_uart_config_t *config,
                          jy61p_uart_handle_t *ret_driver)
{
    if (config == NULL || ret_driver == NULL || config->baud_rate <= 0 ||
        config->rx_gpio < 0) return ESP_ERR_INVALID_ARG;
    jy61p_uart_handle_t driver = calloc(1, sizeof(*driver));
    if (driver == NULL) return ESP_ERR_NO_MEM;
    driver->uart_num = config->uart_num;
    const uart_config_t uart_config = {
        .baud_rate = config->baud_rate, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(config->uart_num, JY61P_RX_BUFFER_SIZE, 0, 0, NULL, 0);
    if (err == ESP_OK) err = uart_param_config(config->uart_num, &uart_config);
    if (err == ESP_OK) err = uart_set_pin(config->uart_num, config->tx_gpio,
                                           config->rx_gpio, UART_PIN_NO_CHANGE,
                                           UART_PIN_NO_CHANGE);
    if (err == ESP_OK) { *ret_driver = driver; return ESP_OK; }
    uart_driver_delete(config->uart_num);
    free(driver);
    return err;
}

esp_err_t jy61p_uart_read(jy61p_uart_handle_t driver, jy61p_uart_data_t *data,
                          TickType_t timeout_ticks)
{
    if (driver == NULL || data == NULL) return ESP_ERR_INVALID_ARG;
    while (true) {
        uint8_t byte;
        if (uart_read_bytes(driver->uart_num, &byte, 1, timeout_ticks) != 1) return ESP_ERR_TIMEOUT;
        ++driver->stats.received_bytes;
        if (driver->used == 0 && byte != 0x55) continue;
        driver->frame[driver->used++] = byte;
        if (driver->used != JY61P_FRAME_LENGTH) continue;
        if (valid_frame(driver->frame)) {
            decode_frame(driver->frame, data);
            driver->used = 0;
            ++driver->stats.valid_frames;
            return ESP_OK;
        }

        ++driver->stats.rejected_frames;
        memmove(driver->frame, &driver->frame[1], JY61P_FRAME_LENGTH - 1);
        driver->used = JY61P_FRAME_LENGTH - 1;
        while (driver->used > 0 && driver->frame[0] != 0x55) {
            memmove(driver->frame, &driver->frame[1], --driver->used);
        }
    }
}

void jy61p_uart_get_stats(jy61p_uart_handle_t driver, jy61p_uart_stats_t *stats)
{
    if (driver != NULL && stats != NULL) *stats = driver->stats;
}
