#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_timer.h"
#include "esp_log.h"

#include "shared.h"
#include "config.h"

static const char *TAG = "main";
// Definicion de Tareas
void vTaskIMU(void *pvParameters);

void app_main(void)
{
    // Creacion de Tareas
    xTaskCreatePinnedToCore(vTaskIMU, "IMU_Task", 8192, NULL, 3, NULL, 0);
}

// ══════════════════════════════════════════════════════════════════════
// TaskIMU — Core 0, 50 Hz
// ══════════════════════════════════════════════════════════════════════
void vTaskIMU(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PERIOD_IMU_MS);

    int64_t lastTimestamp = esp_timer_get_time();
    int64_t lastPrint = esp_timer_get_time();

    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, period);

        int64_t now = esp_timer_get_time();
        int64_t realPeriodUs = now - lastTimestamp;
        lastTimestamp = now;

        if ((now - lastPrint) >= 1000000)
        {
            ESP_LOGI(TAG, "Periodo real: %lld us (%.2f Hz)", realPeriodUs, 1000000.0f / realPeriodUs);

            lastPrint = now;
        }

        readIMUAndSend();
    }
}