#include <stdio.h>
#include <inttypes.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "shared.h"
#include "sht40.h"
#include "V1_Rover_Pins.h"

static const char *TAG = "main_sht40";

static i2c_master_bus_handle_t i2c_bus = NULL;
static sht4xa_t sht40 = {0};

void app_main(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA1,
        .scl_io_num = SCL1,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    sht40.bus_handle = i2c_bus;

    esp_err_t err = sht4xa_init(&sht40);
    if (err == ESP_OK)
    {
        sht_ok = true;
        ESP_LOGI(TAG, "SHT40 ready, serial: 0x%08" PRIX32, sht40.data.serial);
    }
    else
    {
        sht_ok = false;
        ESP_LOGE(TAG, "SHT40 init failed: %s", esp_err_to_name(err));
    }

    while (1)
    {
        if (sht_ok)
        {
            err = sht4xa_measure_blocking_read(&sht40, SHT4XA_PRECISION_HIGH);
            if (err == ESP_OK)
            {
                temp_amb = sht40.data.temp;
                humedad = sht40.data.humidity;

                ESP_LOGI(TAG,
                         "Temp: %.2f C, Humidity: %.2f %%",
                         temp_amb,
                         humedad);
            }
            else
            {
                ESP_LOGW(TAG, "sht4xa_measure_blocking_read failed: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
