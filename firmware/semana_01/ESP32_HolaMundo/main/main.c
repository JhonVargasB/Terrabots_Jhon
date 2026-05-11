#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const static char *TAG = "main";

// Prototipo de ducniones de aplciacion
esp_err_t gpio_init(void);

void app_main(void)
{
    // Inicializacion de hw
    ESP_ERROR_CHECK(gpio_init());

    while (1)
    {
        ESP_LOGI(TAG, "Hola dsde ESP32");
        gpio_set_level(GPIO_NUM_21, 0);
        vTaskDelay(pdMS_TO_TICKS(700));
        gpio_set_level(GPIO_NUM_21, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t gpio_init(void)
{
    gpio_config_t io_Config = {
        .pin_bit_mask = (1ULL << 21),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    gpio_config(&io_Config);
    return ESP_OK;
}