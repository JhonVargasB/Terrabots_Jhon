#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lps.h"
#include "shared.h"
#include "V1_Rover_Pins.h"

static const char *TAG = "main_lps";

static i2c_master_bus_handle_t i2c_bus = NULL;
static lps22hb_t lps = {0};

void app_main(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA2,
        .scl_io_num = SCL2,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    lps22hb_config_t config = LPS22HB_DEFAULT_CONFIG();
    lps.bus_handle = i2c_bus;
    lps.config = config;

    esp_err_t err = lps22hb_init(&lps);
    if (err == ESP_OK)
    {
        lps_ok = true;
        ESP_LOGI(TAG, "LPS22HB ready");
    }
    else
    {
        lps_ok = false;
        ESP_LOGE(TAG, "LPS22HB init failed: %s", esp_err_to_name(err));
    }

    while (1)
    {
        if (lps_ok)
        {
            err = lps22hb_read_data(&lps);
            if (err == ESP_OK)
            {
                presion = lps.data.pressure_hpa;
                temp_lps = lps.data.temperature_c;

                ESP_LOGI(TAG,
                         "Pressure: %.2f hPa (%.2f Pa), Temp: %.2f C",
                         lps.data.pressure_hpa,
                         lps.data.pressure_pa,
                         lps.data.temperature_c);
            }
            else
            {
                ESP_LOGW(TAG, "lps22hb_read_data failed: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
