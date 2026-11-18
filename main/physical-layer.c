#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

void app_main(void)
{
printf("Physical Layer Project\n");
printf("ESP32 initialized successfully\n");

while (1) {
    printf("ESP32 is running...\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
}
}
