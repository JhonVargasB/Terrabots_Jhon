#include <inttypes.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "shared.h"
#include "storage.h"

static const char *TAG = "main_sd";

static sd_card_t sd;

static esp_err_t sd_create_header_if_needed(void)
{
    char first_line[256] = {0};

    /*
     * Si el archivo no existe, o existe pero esta vacio, sd_read_file falla.
     * En ese caso escribimos el encabezado compatible con DATALOG_HEADER.
     */
    esp_err_t ret = sd_read_file(DATALOG_PATH, first_line, sizeof(first_line));
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Archivo existente: %s", DATALOG_PATH);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Creando encabezado en %s", DATALOG_PATH);
    return sd_write_file(DATALOG_PATH, DATALOG_HEADER "\n");
}

static esp_err_t sd_append_global_data(void)
{
    char row[384];
    uint64_t millis = (uint64_t)(esp_timer_get_time() / 1000ULL);

    /*
     * Esta fila mantiene el mismo orden de columnas que DATALOG_HEADER.
     * En esta prueba simple, los valores vienen de shared.c; si otros sensores
     * no estan corriendo, sus campos se guardan como 0.
     */
    int len = snprintf(row, sizeof(row),
                       "%" PRIu64 ","
                       "%.3f,%.3f,%.3f,"
                       "%.2f,%.2f,%.2f,%.2f,"
                       "%.8f,%.8f,%.2f,%u,%u,"
                       "%.2f,%.2f,%u,"
                       "%u,%u,%u,%u,%u,%u,%u,%u,%d,%d\n",
                       millis,
                       ImuData.roll, ImuData.pitch, ImuData.yaw,
                       temp_amb, humedad, presion, temp_lps,
                       gps_lat, gps_lon, gps_alt, (unsigned)gps_sats, (unsigned)gps_fix,
                       bms_voltaje, bms_corriente, (unsigned)bms_soc,
                       (unsigned)bms_celdas[0], (unsigned)bms_celdas[1],
                       (unsigned)bms_celdas[2], (unsigned)bms_celdas[3],
                       (unsigned)bms_celdas[4], (unsigned)bms_celdas[5],
                       (unsigned)bms_celdas[6], (unsigned)bms_celdas[7],
                       bms_t1, bms_t2);

    if (len < 0 || len >= (int)sizeof(row))
    {
        return ESP_ERR_NO_MEM;
    }

    return sd_write_file(DATALOG_PATH, row);
}

void app_main(void)
{
    sd.config = SD_DEFAULT_CONFIG();

    esp_err_t ret = sd_init(&sd);
    if (ret != ESP_OK)
    {
        sd_ok = false;
        ESP_LOGE(TAG, "No se pudo inicializar SD: %s", esp_err_to_name(ret));
        return;
    }

    sd_ok = true;
    ESP_LOGI(TAG, "SD montada en %s", sd.config.mount_point);

    ret = sd_create_header_if_needed();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo preparar el archivo CSV: %s", esp_err_to_name(ret));
        return;
    }

    while (true)
    {
        ret = sd_append_global_data();
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Fila escrita en %s", DATALOG_PATH);
        }
        else
        {
            ESP_LOGE(TAG, "Error escribiendo fila: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
