#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    while (true) {
        printf("Hello, ESP32!\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
