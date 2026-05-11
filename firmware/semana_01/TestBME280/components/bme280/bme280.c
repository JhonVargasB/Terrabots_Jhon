#include <stdio.h>
#include "bme280.h"
#include "bme280_defs.h"

static const char *TAG = "bme280";

bme280_settings settings = {
    .osr_p = BME280_OVERSAMPLING_4X,
    .osr_h = BME280_NO_OVERSAMPLING,
    .osr_t = BME280_OVERSAMPLING_1X,
    .filter = BME280_FILTER_COEFF_16,
    .standby_time = BME280_STANDBY_TIME_0_5_MS,
    .mode = BME280_POWERMODE_NORMAL};

static double compensate_temperature(bme280_t *bme280);
static double compensate_pressure(bme280_t *bme280);
static double compensate_humidity(bme280_t *bme280);

static esp_err_t bme280_device_create(bme280_t *bme280)
{
    if (bme280 == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    bme280->dev_cfg = (i2c_device_config_t){
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x76,
        .scl_speed_hz = 100000,
    };

    ESP_LOGI(TAG, "device_create for BME280 sensors on ADDR %X", bme280->dev_cfg.device_address);

    esp_err_t err = i2c_master_bus_add_device(bme280->bus_handle, &bme280->dev_cfg, &bme280->i2c_dev);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "device_create success on 0x%x", bme280->dev_cfg.device_address);
        return err;
    }
    else
    {
        ESP_LOGE(TAG, "device_create error on 0x%x", bme280->dev_cfg.device_address);
        return err;
    }
}

static esp_err_t bme280_read(bme280_t *bme280, uint8_t addr, uint8_t *dout, size_t size)
{
    return i2c_master_transmit_receive(bme280->i2c_dev, &addr, sizeof(addr), dout, size, -1);
}

static esp_err_t bme280_write(bme280_t *bme280, uint8_t addr, const uint8_t *din, size_t size)
{
    esp_err_t err;

    for (uint8_t i = 0; i < size; i++)
    {
        uint8_t dat[2] = {(addr + i), din[i]};
        if ((err = i2c_master_transmit(bme280->i2c_dev, dat, 2, -1)) != ESP_OK)
            return err;
    }
    return ESP_OK;
}

// Calibration data
static esp_err_t bme280_get_cal_data(bme280_t *bme280)
{
    uint8_t calib_data[26];
    uint8_t calib_data2[7];
    esp_err_t err = bme280_read(bme280, BME280_REG_TEMP_PRESS_CALIB_DATA, calib_data, sizeof(calib_data));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read temp and press calibration data");
        return err;
    }
    err = bme280_read(bme280, BME280_REG_HUMIDITY_CALIB_DATA, calib_data2, sizeof(calib_data2));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read hum calibration data");
        return err;
    }

    // Datos de alibracion leidos
    bme280_calib_data calib = {0};

    calib.dig_t1 = BME280_U16(calib_data[0], calib_data[1]);
    calib.dig_t2 = BME280_S16(calib_data[2], calib_data[3]);
    calib.dig_t3 = BME280_S16(calib_data[4], calib_data[5]);

    calib.dig_p1 = BME280_U16(calib_data[6], calib_data[7]);
    calib.dig_p2 = BME280_S16(calib_data[8], calib_data[9]);
    calib.dig_p3 = BME280_S16(calib_data[10], calib_data[11]);
    calib.dig_p4 = BME280_S16(calib_data[12], calib_data[13]);
    calib.dig_p5 = BME280_S16(calib_data[14], calib_data[15]);
    calib.dig_p6 = BME280_S16(calib_data[16], calib_data[17]);
    calib.dig_p7 = BME280_S16(calib_data[18], calib_data[19]);
    calib.dig_p8 = BME280_S16(calib_data[20], calib_data[21]);
    calib.dig_p9 = BME280_S16(calib_data[22], calib_data[23]);

    calib.dig_h1 = calib_data[25];
    calib.dig_h2 = BME280_S16(calib_data2[0], calib_data2[1]);
    calib.dig_h3 = calib_data2[2];
    calib.dig_h4 = (int16_t)((((uint16_t)calib_data2[3]) << 4) | (calib_data2[4] & 0x0F));
    calib.dig_h5 = (int16_t)(((uint16_t)calib_data2[5] << 4) | (calib_data2[4] >> 4));
    calib.dig_h6 = (int8_t)(calib_data2[6]);

    bme280->calib = calib;
    ESP_LOGI(TAG, "CALIB TEMP -> T1: %u, T2: %d, T3: %d",
             calib.dig_t1, calib.dig_t2, calib.dig_t3);
    ESP_LOGI(TAG, "CALIB PRESS -> P1: %u, P2: %d, P3: %d, P4: %d, P5: %d, P6: %d, P7: %d, P8: %d, P9: %d",
             calib.dig_p1, calib.dig_p2, calib.dig_p3,
             calib.dig_p4, calib.dig_p5, calib.dig_p6,
             calib.dig_p7, calib.dig_p8, calib.dig_p9);
    ESP_LOGI(TAG, "CALIB HUM -> H1: %u, H2: %d, H3: %u, H4: %d, H5: %d, H6: %d",
             calib.dig_h1, calib.dig_h2, calib.dig_h3,
             calib.dig_h4, calib.dig_h5, calib.dig_h6);
    return ESP_OK;
}

// Sensor settings
static esp_err_t bme280_set_sensor_settings(bme280_t *bme280, bme280_settings *settings)
{
    // Registro config
    uint8_t d1, d2;
    esp_err_t err = bme280_read(bme280, BME280_REG_CONFIG, &d1, 1);
    if (err != ESP_OK) return ESP_FAIL;

    d2 = (d1 & 0x02) | ((settings->standby_time & 0x07) << 5) | ((settings->filter & 0x07) << 2);
    err = bme280_write(bme280, BME280_REG_CONFIG, &d2, 1);
    if (err != ESP_OK)  return ESP_FAIL;

    // registro ctrl_hum
    err = bme280_read(bme280, BME280_REG_CTRL_HUM, &d1, 1);
    if (err != ESP_OK)  return ESP_FAIL;
    d2 = (d1 & 0xf8) | (settings->osr_h & 0x07);
    err = bme280_write(bme280, BME280_REG_CTRL_HUM, &d2, 1);
    if (err != ESP_OK)  return ESP_FAIL;

    // cambiar modo
    d2 = ((settings->osr_t & 0x07) << 5) | ((settings->osr_p & 0x07) << 2) | (0x03 & (settings->mode));
    err = bme280_write(bme280, BME280_REG_PWR_CTRL, &d2, 1);
    if (err != ESP_OK)  return ESP_FAIL;
    return err;
}

static esp_err_t bme280_get_uncompensated_data(bme280_t *bme280)
{
    if (bme280 == NULL)
        return ESP_ERR_INVALID_ARG;
    uint8_t data[8];
    esp_err_t err = bme280_read(bme280, BME280_REG_DATA, data, 8);
    if (err != ESP_OK)
        return ESP_FAIL;
    bme280->uncomp.humidity = BME280_CONCAT_BYTES(data[6], data[7]);
    bme280->uncomp.temperature = BME280_U20(data[3], data[4], data[5]);
    bme280->uncomp.pressure = BME280_U20(data[0], data[1], data[2]);
    return ESP_OK;
}

static esp_err_t bme280_get_compensated_data(bme280_t *bme280)
{
    if (bme280 == NULL)
        return ESP_ERR_INVALID_ARG;

    bme280->data.humidity = compensate_humidity(bme280);
    bme280->data.temperature = compensate_temperature(bme280);
    bme280->data.pressure = compensate_pressure(bme280);
    return ESP_OK;
}

esp_err_t bme280_get_data(bme280_t *bme280)
{
    bme280_get_uncompensated_data(bme280);
    bme280_get_compensated_data(bme280);
    return ESP_OK;
}

esp_err_t bme280_init(bme280_t *bme280)
{
    uint8_t dout;
    if (bme280 == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = bme280_device_create(bme280);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "bme280_device_create failed");
        return err;
    }
    err = bme280_read(bme280, BME280_REG_CHIP_ID, &dout, 1);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "bme280_init success, chip id: %02x", dout);
    }
    else
    {
        ESP_LOGE(TAG, "bme280_init failed");
        return err;
    }
    err = bme280_get_cal_data(bme280);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "bme280_get_cal_data failed");
        return err;
    }

    err = bme280_set_sensor_settings(bme280, &settings);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "bme280_get_cal_data failed");
        return err;
    }
    return ESP_OK;
}

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
static double compensate_temperature(bme280_t *bme280)
{
    double var1;
    double var2;
    double temperature;
    double temperature_min = -40;
    double temperature_max = 85;

    var1 = (((double)bme280->uncomp.temperature) / 16384.0 - ((double)bme280->calib.dig_t1) / 1024.0);
    var1 = var1 * ((double)bme280->calib.dig_t2);
    var2 = (((double)bme280->uncomp.temperature) / 131072.0 - ((double)bme280->calib.dig_t1) / 8192.0);
    var2 = (var2 * var2) * ((double)bme280->calib.dig_t3);
    bme280->calib.t_fine = (int32_t)(var1 + var2);

    temperature = (var1 + var2) / 5120.0;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 */
static double compensate_pressure(bme280_t *bme280)
{
    double var1;
    double var2;
    double var3;
    double pressure;
    double pressure_min = 30000.0;
    double pressure_max = 110000.0;

    var1 = ((double)bme280->calib.t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)bme280->calib.dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double)bme280->calib.dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double)bme280->calib.dig_p4) * 65536.0);
    var3 = ((double)bme280->calib.dig_p3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double)bme280->calib.dig_p2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)bme280->calib.dig_p1);

    /* Avoid exception caused by division by zero */
    if (var1 > (0.0))
    {
        pressure = 1048576.0 - (double)bme280->uncomp.pressure;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)bme280->calib.dig_p9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((double)bme280->calib.dig_p8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((double)bme280->calib.dig_p7)) / 16.0;

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else /* Invalid case */
    {
        pressure = pressure_min;
    }

    return pressure;
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in double data type.
 */
static double compensate_humidity(bme280_t *bme280)
{
    double humidity;
    double humidity_min = 0.0;
    double humidity_max = 100.0;
    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    double var6;

    var1 = ((double)bme280->calib.t_fine) - 76800.0;
    var2 = (((double)bme280->calib.dig_h4) * 64.0 + (((double)bme280->calib.dig_h5) / 16384.0) * var1);
    var3 = bme280->uncomp.humidity - var2;
    var4 = ((double)bme280->calib.dig_h2) / 65536.0;
    var5 = (1.0 + (((double)bme280->calib.dig_h3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)bme280->calib.dig_h6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((double)bme280->calib.dig_h1) * var6 / 524288.0);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }
    else if (humidity < humidity_min)
    {
        humidity = humidity_min;
    }

    return humidity;
}
