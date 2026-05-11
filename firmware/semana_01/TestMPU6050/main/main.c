#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "mpu.h"

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
mpu6050_t mpu6050;

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    mpu6050.bus_handle = bus_handle;
    ESP_ERROR_CHECK(mpu6050_init(&mpu6050));
    ESP_ERROR_CHECK(selftest_xg(&mpu6050));
    ESP_ERROR_CHECK(selftest_yg(&mpu6050));
    ESP_ERROR_CHECK(selftest_zg(&mpu6050));
    ESP_ERROR_CHECK(mpu6050_calibrate_accel(&mpu6050));
    ESP_ERROR_CHECK(mpu6050_calibrate_gyro(&mpu6050));
    while (1)
    {
        mpu6050_read_accel(&mpu6050);
        mpu6050_read_gyro(&mpu6050);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
