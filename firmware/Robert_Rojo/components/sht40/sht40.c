#include <stdio.h>
#include "sht40.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sht4xa: ";

static esp_err_t sht4xa_device_create(sht4xa_t *sht4xa)
{
    if (sht4xa == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    sht4xa->dev_cfg = (i2c_device_config_t){
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT4XA_I2C_ADDR_DEFAULT,
        .scl_speed_hz = 100000,
    };

    ESP_LOGI(TAG, "device_create for sht4xa sensors on ADDR %X", sht4xa->dev_cfg.device_address);

    esp_err_t err = i2c_master_bus_add_device(sht4xa->bus_handle, &sht4xa->dev_cfg, &sht4xa->i2c_dev);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "device_create success on 0x%x", sht4xa->dev_cfg.device_address);
        return err;
    }
    else
    {
        ESP_LOGE(TAG, "device_create error on 0x%x", sht4xa->dev_cfg.device_address);
        return err;
    }
}



static esp_err_t sht4xa_write_cmd(sht4xa_t *sht4xa, uint8_t cmd)
{
    if (sht4xa == NULL)
        return ESP_ERR_INVALID_ARG;

    return i2c_master_transmit(sht4xa->i2c_dev, &cmd, sizeof(cmd), -1);
}



static esp_err_t sht4xa_read_bytes(sht4xa_t *sht4xa,
                                   uint8_t *data,
                                   size_t len)
{
    if (sht4xa == NULL || data == NULL || len == 0)
        return ESP_ERR_INVALID_ARG;

    return i2c_master_receive(sht4xa->i2c_dev, data, len, -1);
}





static uint8_t sht4xa_crc8(const uint8_t *data,
                           size_t len)
{
    uint8_t crc = SHT4XA_CRC_INIT;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ SHT4XA_CRC_POLYNOMIAL;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static esp_err_t sht4xa_check_crc(const uint8_t *data,
                                  uint8_t crc_expected)
{
    uint8_t crc_calculated;

    crc_calculated = sht4xa_crc8(data, 2);

    if (crc_calculated != crc_expected)
    {
        ESP_LOGE(TAG,
                 "CRC error. Calc=0x%02X Recv=0x%02X",
                 crc_calculated,
                 crc_expected);

        return ESP_ERR_SHT4XA_CRC;
    }

    return ESP_OK;
}

static uint8_t sht4xa_precision_to_cmd(sht4xa_precision_t precision)
{
    switch (precision)
    {
    case SHT4XA_PRECISION_LOW:
        return SHT4XA_CMD_MEASURE_LOW;

    case SHT4XA_PRECISION_MEDIUM:
        return SHT4XA_CMD_MEASURE_MEDIUM;

    case SHT4XA_PRECISION_HIGH:
        return SHT4XA_CMD_MEASURE_HIGH;

    default:
        return SHT4XA_CMD_MEASURE_HIGH;
    }
}


static uint32_t sht4xa_precision_to_delay_ms(sht4xa_precision_t precision)
{
    switch (precision)
    {
    case SHT4XA_PRECISION_LOW:
        return SHT4XA_MEAS_LOW_DELAY_MS;

    case SHT4XA_PRECISION_MEDIUM:
        return SHT4XA_MEAS_MEDIUM_DELAY_MS;

    case SHT4XA_PRECISION_HIGH:
        return SHT4XA_MEAS_HIGH_DELAY_MS;

    default:
        return SHT4XA_MEAS_HIGH_DELAY_MS;
    }
}

static esp_err_t sht4xa_parse_measurement(sht4xa_t *sht4xa,
                                          const uint8_t raw[6])
{
    uint16_t raw_temperature;
    uint16_t raw_humidity;

    if (sht4xa_check_crc(raw, raw[2]) != ESP_OK)
    {
        ESP_LOGE(TAG, "Temperature CRC error");
        return ESP_ERR_SHT4XA_CRC;
    }

    if (sht4xa_check_crc(&raw[3], raw[5]) != ESP_OK)
    {
        ESP_LOGE(TAG, "Humidity CRC error");
        return ESP_ERR_SHT4XA_CRC;
    }


    raw_temperature = ((uint16_t)raw[0] << 8) | raw[1];
    raw_humidity = ((uint16_t)raw[3] << 8) | raw[4];


    sht4xa->data.temp =
        SHT4XA_TEMP_OFFSET_C +
        (SHT4XA_TEMP_SCALE_C *
         ((float)raw_temperature / SHT4XA_RAW_MAX));


    sht4xa->data.humidity =
        SHT4XA_RH_OFFSET +
        (SHT4XA_RH_SCALE *
         ((float)raw_humidity / SHT4XA_RAW_MAX));


    if (sht4xa->data.humidity > SHT4XA_RH_MAX)
        sht4xa->data.humidity = SHT4XA_RH_MAX;

    if (sht4xa->data.humidity < SHT4XA_RH_MIN)
        sht4xa->data.humidity = SHT4XA_RH_MIN;

    return ESP_OK;
}

static esp_err_t sht4xa_measure_start(sht4xa_t *sht4xa,
                               sht4xa_precision_t precision)
{
    uint8_t cmd = sht4xa_precision_to_cmd(precision);
    return sht4xa_write_cmd(sht4xa, cmd);
}

esp_err_t sht4xa_read(sht4xa_t *sht4xa)
{
    esp_err_t err;
    uint8_t raw[SHT4XA_MEAS_RESPONSE_LEN];

    if (sht4xa == NULL)
        return ESP_ERR_INVALID_ARG;

    err = sht4xa_read_bytes(sht4xa,
                            raw,
                            SHT4XA_MEAS_RESPONSE_LEN);
    if (err != ESP_OK)
        return err;

    return sht4xa_parse_measurement(sht4xa, raw);
}

esp_err_t sht4xa_measure_blocking_read(sht4xa_t *sht4xa,
                                       sht4xa_precision_t precision)
{
    esp_err_t err;
    uint32_t delay_ms;

    if (sht4xa == NULL)
        return ESP_ERR_INVALID_ARG;

    err = sht4xa_measure_start(sht4xa, precision);
    if (err != ESP_OK)
        return err;

    delay_ms = sht4xa_precision_to_delay_ms(precision);

    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    return sht4xa_read(sht4xa);
}




esp_err_t sht4xa_read_serial(sht4xa_t *sht4xa)
{
    esp_err_t err;
    uint8_t raw[SHT4XA_SERIAL_RESPONSE_LEN];
    uint16_t serial_msb;
    uint16_t serial_lsb;

    if (sht4xa == NULL)
        return ESP_ERR_INVALID_ARG;

    err = sht4xa_write_cmd(sht4xa, SHT4XA_CMD_READ_SERIAL);
    if (err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(1));

    err = sht4xa_read_bytes(sht4xa, raw, SHT4XA_SERIAL_RESPONSE_LEN);
    if (err != ESP_OK)
        return err;

    if (sht4xa_check_crc(raw, raw[2]) != ESP_OK)
    {
        ESP_LOGE(TAG, "Serial MSB CRC error");
        return ESP_ERR_SHT4XA_CRC;
    }

    if (sht4xa_check_crc(&raw[3], raw[5]) != ESP_OK)
    {
        ESP_LOGE(TAG, "Serial LSB CRC error");
        return ESP_ERR_SHT4XA_CRC;
    }

    serial_msb = ((uint16_t)raw[0] << 8) | raw[1];
    serial_lsb = ((uint16_t)raw[3] << 8) | raw[4];

    sht4xa->data.serial=
        ((uint32_t)serial_msb << 16) | serial_lsb;

    return ESP_OK;
}

esp_err_t sht4xa_soft_reset(sht4xa_t *sht4xa)
{
    esp_err_t err;

    if (sht4xa == NULL)
        return ESP_ERR_INVALID_ARG;

    err = sht4xa_write_cmd(sht4xa, SHT4XA_CMD_SOFT_RESET);
    if (err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(SHT4XA_RESET_DELAY_MS));

    return ESP_OK;
}

esp_err_t sht4xa_init(sht4xa_t *sht4xa)
{
    if (sht4xa == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = sht4xa_device_create(sht4xa);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "sht4xa_device_create failed");
        return err;
    }

    err = sht4xa_read_serial(sht4xa);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "sht4xa_read_serial failed");
        return err;
    }
    return ESP_OK;
}
