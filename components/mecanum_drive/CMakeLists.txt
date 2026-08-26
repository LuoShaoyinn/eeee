idf_component_register(SRCS "mecanum_drive.c"
                       INCLUDE_DIRS "include"
                       REQUIRES JCA25-2430-CE esp_driver_pcnt esp_timer)
