idf_component_register(SRCS "motor_control_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES mecanum_drive standard-3wire driver esp_driver_uart esp_driver_i2c esp_wifi esp_event esp_netif nvs_flash app_update lwip)
