#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/ledc.h"

#include "pwm_ctrl.h"

static const char *TAG = "main";

#define GPIOBUTTON GPIO_NUM_9

// Prototipo de funciones de Aplicacion
static void gpio_init(void);

// Prototipo de tareas
static void vTaskMotor(void *pvParameters);
static void vTaskButton(void *pvParameters);

// Identificadores RTOS
SemaphoreHandle_t xSemaphoreEventsButton = NULL;
TaskHandle_t xTaskMotorHandle = NULL;

// Tareas ISR
void IRAM_ATTR buttonIsrHandler(void *arg)
{
    // Dar el semaforo binario
    xSemaphoreGiveFromISR(xSemaphoreEventsButton, NULL);
}

void app_main(void)
{

    // Inicializacion de Hardware
    xSemaphoreEventsButton = xSemaphoreCreateBinary();
    gpio_init();
    motor_init();

    xTaskCreate(vTaskMotor, "vTaskMotor", 2048, NULL, 5, &xTaskMotorHandle);
    xTaskCreate(vTaskButton, "vTaskButton", 2048, NULL, 7, NULL);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Definicion de Tareas
static void vTaskMotor(void *pvParameters)
{
    while (1)
    {
        ESP_LOGI(TAG, "Motor: sentido positivo, velocidad 100%%");
        motor_set(100);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Motor: sentido positivo, velocidad 40%%");
        motor_set(40);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Motor: sentido positivo, velocidad 80%%");
        motor_set(80);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Motor: sentido positivo, velocidad 60%%");
        motor_set(60);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Motor: sentido negativo, velocidad 100%%");
        motor_set(-100);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Motor: sentido negativo, velocidad 40%%");
        motor_set(-40);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Motor: sentido negativo, velocidad 80%%");
        motor_set(-80);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Motor: sentido negativo, velocidad 60%%");
        motor_set(-60);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void vTaskButton(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xSemaphoreEventsButton, portMAX_DELAY) == pdTRUE)
        {
            vTaskSuspend(xTaskMotorHandle);
            brake();
        }
    }
}

// Definicion de funciones de app

static void gpio_init(void)
{
    gpio_config_t io_Configin = {
        .pin_bit_mask = (1ULL << GPIOBUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE};

    gpio_config(&io_Configin);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIOBUTTON, buttonIsrHandler, NULL);
}
