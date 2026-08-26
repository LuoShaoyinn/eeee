idf_component_register(SRCS "motor_control_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES mecanum_drive esp_wifi esp_event esp_netif nvs_flash lwip)
