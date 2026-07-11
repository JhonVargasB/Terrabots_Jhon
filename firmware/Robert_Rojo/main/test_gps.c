#include <stdbool.h>
#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"
#include "shared.h"
#include "V1_Rover_Pins.h"

static const char *TAG = "main_gps";

static i2c_master_bus_handle_t i2c_bus = NULL;
static gps_t gps = {0};

static void gps_log_device_info(void)
{
    gps_device_info_t info = {0};

    esp_err_t err = gps_get_device_info(&gps, &info);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "GPS device info not available: %s", esp_err_to_name(err));
        return;
    }

    if (info.version_valid)
    {
        ESP_LOGI(TAG, "GPS SW: %s", info.software_version);
        ESP_LOGI(TAG, "GPS HW: %s", info.hardware_version);
    }

    if (info.unique_id_valid)
    {
        ESP_LOGI(TAG,
                 "GPS unique ID: %02X%02X%02X%02X%02X",
                 info.unique_id[0],
                 info.unique_id[1],
                 info.unique_id[2],
                 info.unique_id[3],
                 info.unique_id[4]);
    }
}

static void gps_publish_data(const gps_data_t *data)
{
    gps_ok = data->position_valid;
    gps_lat = data->latitude_deg;
    gps_lon = data->longitude_deg;
    gps_alt = data->altitude_msl_m;
    gps_sats = data->satellites_used;
    gps_fix = (uint8_t)data->fix_type;
}

static void gps_log_data(const gps_data_t *data)
{
    ESP_LOGI(TAG,
             "fix=%u valid=%u sats=%u lat=%.7f lon=%.7f alt=%.2f m hAcc=%.2f m vAcc=%.2f m speed=%.2f m/s",
             (unsigned)data->fix_type,
             data->position_valid ? 1U : 0U,
             (unsigned)data->satellites_used,
             data->latitude_deg,
             data->longitude_deg,
             data->altitude_msl_m,
             data->horizontal_accuracy_m,
             data->vertical_accuracy_m,
             data->ground_speed_mps);
}

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

    gps_config_t config = GPS_DEFAULT_CONFIG();
    config.i2c_clock_hz = 100000U;
    config.reset_pin = RST_GPS_1;
    config.safeboot_pin = SAFE_GPS_1;

    gps.bus_handle = i2c_bus;
    gps.config = config;

    esp_err_t err = gps_init(&gps);

    if (err != ESP_OK)
    {
        gps_ok = false;
        ESP_LOGE(TAG, "GPS init failed: %s", esp_err_to_name(err));

        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "GPS ready");
    gps_log_device_info();

    int waiting_ticks = 0;

    while (1)
    {
        bool new_data = false;

        err = gps_update(&gps, &new_data);

        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "gps_update failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (new_data)
        {
            gps_data_t data = {0};

            err = gps_get_data(&gps, &data);

            if (err == ESP_OK)
            {
                gps_publish_data(&data);
                gps_log_data(&data);
            }
            else
            {
                ESP_LOGW(TAG, "gps_get_data failed: %s", esp_err_to_name(err));
            }

            waiting_ticks = 0;
        }
        else
        {
            waiting_ticks++;

            if (waiting_ticks >= 20)
            {
                ESP_LOGI(TAG, "Waiting for NAV-PVT data...");
                waiting_ticks = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
