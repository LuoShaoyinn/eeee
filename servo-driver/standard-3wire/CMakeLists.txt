idf_component_register(SRCS "src/standard_servo.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_gpio esp_driver_ledc)
