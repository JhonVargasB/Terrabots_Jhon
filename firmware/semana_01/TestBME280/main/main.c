#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "bme280.h"

// BME280
static const char *TAG = "main";


i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = GPIO_NUM_6,
    .sda_io_num = GPIO_NUM_5,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false,
};


i2c_master_bus_handle_t bus_handle;
bme280_t bme280;

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    bme280.bus_handle = bus_handle;
    ESP_ERROR_CHECK(bme280_init(&bme280));
    while(1){
        ESP_ERROR_CHECK(bme280_get_data(&bme280));
        ESP_LOGI(TAG, "datos obtenidos: T = %f , H = %f , P = %f",bme280.data.temperature,bme280.data.humidity,bme280.data.pressure);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}