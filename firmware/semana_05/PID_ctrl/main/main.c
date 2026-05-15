#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"


static const char *TAG = "main";
#define ENC1_A GPIO_NUM_1
#define ENC1_B GPIO_NUM_2

int counter = 0;

// Prototipo de funciones de Aplicacion
void initpcnt(void);

// Prototipo de tareas

// Identificadores de Recursos RTOS

// Identificadores de recursos de Hardware
pcnt_unit_handle_t g_pcnt1_unit;

// Iden

void app_main(void)
{

    // Inicializacion de Hardware
    initpcnt();
    while (1)
    {
        ESP_ERROR_CHECK(pcnt_unit_get_count(g_pcnt1_unit, &counter));
        ESP_LOGI(TAG, "El valor de la uentas es: %d",counter);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void initpcnt(void)

{
    pcnt_unit_config_t unit_cfg = {
        .high_limit = 30000,
        .low_limit = -30000,
    };
    pcnt_glitch_filter_config_t filt_cfg = {
        .max_glitch_ns = 2000,
    };
    // Canal 1
    pcnt_chan_config_t enc1_a_cfg = {
        .edge_gpio_num = ENC1_A,
        .level_gpio_num = ENC1_B,
    };
    pcnt_chan_config_t enc1_b_cfg = {
        .edge_gpio_num = ENC1_B,
        .level_gpio_num = ENC1_A,
    };

    // Identificadores de canal
    pcnt_channel_handle_t enc1_ch_a = NULL, enc1_ch_b = NULL;

    // Crear la unidad
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &g_pcnt1_unit));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(g_pcnt1_unit, &filt_cfg)); // Crearle glitching filter
    // Crear canales
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt1_unit, &enc1_a_cfg, &enc1_ch_a));
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt1_unit, &enc1_b_cfg, &enc1_ch_b));
    // Configración en cuadratura
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(enc1_ch_a,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(enc1_ch_a,
                                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(enc1_ch_b,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(enc1_ch_b,
                                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    // Habilitar unidad, establecer cont en 0 e iniciar
    ESP_ERROR_CHECK(pcnt_unit_enable(g_pcnt1_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcnt1_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(g_pcnt1_unit));
}
