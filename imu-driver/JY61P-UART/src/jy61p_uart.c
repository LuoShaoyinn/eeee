#include "jy61p_uart.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/task.h"

#define JY61P_FRAME_LENGTH 11
#define JY61P_RX_BUFFER_SIZE 512

struct jy61p_uart_driver {
    uart_port_t uart_num;
    uint8_t frame[JY61P_FRAME_LENGTH];
    size_t used;
};

static int16_t le_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool valid_frame(const uint8_t *frame)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < JY61P_FRAME_LENGTH - 1; ++i) {
        checksum = (uint8_t)(checksum + frame[i]);
    }
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

static esp_err_t configure_output(jy61p_uart_handle_t driver)
{
    static const uint8_t unlock[] = {0xff, 0xaa, 0x69, 0x88, 0xb5};
    static const uint8_t output_mask[] = {0xff, 0xaa, 0x02, 0x1e, 0x00};
    static const uint8_t save[] = {0xff, 0xaa, 0x00, 0x00, 0x00};

    if (uart_write_bytes(driver->uart_num, unlock, sizeof(unlock)) < 0) {
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    if (uart_write_bytes(driver->uart_num, output_mask, sizeof(output_mask)) < 0) {
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    if (uart_write_bytes(driver->uart_num, save, sizeof(save)) < 0) {
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    return ESP_OK;
}

esp_err_t jy61p_uart_init(const jy61p_uart_config_t *config,
                          jy61p_uart_handle_t *ret_driver)
{
    if (config == NULL || ret_driver == NULL || config->baud_rate <= 0 ||
        config->tx_gpio < 0 || config->rx_gpio < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    jy61p_uart_handle_t driver = calloc(1, sizeof(*driver));
    if (driver == NULL) {
        return ESP_ERR_NO_MEM;
    }
    driver->uart_num = config->uart_num;

    const uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(config->uart_num, JY61P_RX_BUFFER_SIZE,
                                        0, 0, NULL, 0);
    if (err != ESP_OK) goto fail;
    err = uart_param_config(config->uart_num, &uart_config);
    if (err != ESP_OK) goto fail;
    err = uart_set_pin(config->uart_num, config->tx_gpio, config->rx_gpio,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) goto fail;
    if (config->configure_magnetic_output) {
        err = configure_output(driver);
        if (err != ESP_OK) goto fail;
    }

    *ret_driver = driver;
    return ESP_OK;

fail:
    uart_driver_delete(config->uart_num);
    free(driver);
    return err;
}

esp_err_t jy61p_uart_read(jy61p_uart_handle_t driver,
                          jy61p_uart_data_t *data,
                          TickType_t timeout_ticks)
{
    if (driver == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    while (true) {
        uint8_t byte;
        if (uart_read_bytes(driver->uart_num, &byte, 1, timeout_ticks) != 1) {
            return ESP_ERR_TIMEOUT;
        }
        if (driver->used == 0 && byte != 0x55) {
            continue;
        }
        driver->frame[driver->used++] = byte;
        if (driver->used != JY61P_FRAME_LENGTH) {
            continue;
        }
        if (valid_frame(driver->frame)) {
            decode_frame(driver->frame, data);
            driver->used = 0;
            return ESP_OK;
        }

        memmove(driver->frame, &driver->frame[1], JY61P_FRAME_LENGTH - 1);
        driver->used = JY61P_FRAME_LENGTH - 1;
        while (driver->used > 0 && driver->frame[0] != 0x55) {
            memmove(driver->frame, &driver->frame[1], driver->used - 1);
            --driver->used;
        }
    }
}
