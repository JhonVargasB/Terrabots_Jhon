#include <stdio.h>
#include "lps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "lps";

static esp_err_t lps22hb_device_create(lps22hb_t *lps22hb)
{
    if (lps22hb == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (lps22hb->bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C bus_handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }

    lps22hb->dev_cfg = (i2c_device_config_t){
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = lps22hb->config.device_address,
        .scl_speed_hz = lps22hb->config.scl_speed_hz,
    };

    ESP_LOGI(TAG,
             "device_create for LPS22HB sensor on ADDR 0x%02X",
             lps22hb->dev_cfg.device_address);

    esp_err_t err = i2c_master_bus_add_device(
        lps22hb->bus_handle,
        &lps22hb->dev_cfg,
        &lps22hb->i2c_dev);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG,
                 "device_create success on 0x%02X",
                 lps22hb->dev_cfg.device_address);
    }
    else
    {
        ESP_LOGE(TAG,
                 "device_create error on 0x%02X",
                 lps22hb->dev_cfg.device_address);
    }

    return err;
}

static esp_err_t lps22hb_read(lps22hb_t *lps22hb, uint8_t addr, uint8_t *dout, size_t size)
{
    if (lps22hb == NULL || dout == NULL || size == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (lps22hb->i2c_dev == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return i2c_master_transmit_receive(lps22hb->i2c_dev, &addr, sizeof(addr), dout, size, -1);
}

static esp_err_t lps22hb_write(lps22hb_t *lps22hb, uint8_t addr, const uint8_t *din, size_t size)
{
    if (lps22hb == NULL || din == NULL || size == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (lps22hb->i2c_dev == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err;

    for (size_t i = 0; i < size; i++)
    {
        uint8_t dat[2] = {addr + i, din[i]};

        err = i2c_master_transmit(lps22hb->i2c_dev, dat, sizeof(dat), -1);

        if (err != ESP_OK)
            return err;
    }

    return ESP_OK;
}

esp_err_t lps22hb_init(lps22hb_t *lps22hb)
{
    if (lps22hb == NULL)

        return ESP_ERR_INVALID_ARG;

    esp_err_t err = lps22hb_device_create(lps22hb);
    if (err != ESP_OK)

        return err;

    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t whoami = 0;

    err = lps22hb_read(lps22hb, LPS_REG_WHO_AM_I_REG, &whoami, 1);
    if (err != ESP_OK)

        return err;

    if (whoami != LPS_WHO_AM_I_VALUE)
    {
        ESP_LOGE(TAG, "Invalid LPS22HB WHO_AM_I: 0x%02X", whoami);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X", whoami);

    uint8_t reg;

    /*
     * CTRL_REG2
     * IF_ADD_INC = 1 -> autoincremento de direcciones
     */
    reg = 0x00;
    reg |= (lps22hb->config.auto_increment ? (1 << 4) : 0);

    err = lps22hb_write(lps22hb, LPS_REG_CTRL_REG2, &reg, 1);
    if (err != ESP_OK)
        return err;

    /*
     * CTRL_REG1
     * bits 6:4 = ODR[2:0]
     * bit 3    = EN_LPFP
     * bit 2    = LPFP_CFG
     * bit 1    = BDU
     * bit 0    = SIM
     */
    reg = 0x00;
    reg |= ((lps22hb->config.odr & 0x07) << 4);
    reg |= (lps22hb->config.low_pass_filter_enable ? (1 << 3) : 0);
    reg |= ((lps22hb->config.low_pass_filter_cfg & 0x01) << 2);
    reg |= (lps22hb->config.block_data_update ? (1 << 1) : 0);

    err = lps22hb_write(lps22hb, LPS_REG_CTRL_REG1, &reg, 1);
    if (err != ESP_OK)
        return err;

    /*
     * RES_CONF
     * bit 0 = LC_EN
     */
    reg = 0x00;
    reg |= (lps22hb->config.low_current_mode ? (1 << 0) : 0);

    err = lps22hb_write(lps22hb, LPS_REG_RES_CONF_REG, &reg, 1);
    if (err != ESP_OK)
        return err;

    /*
     * FIFO_CTRL
     * bits 7:5 = F_MODE
     * bits 4:0 = WTM_POINT
     */
    reg = 0x00;
    reg |= ((lps22hb->config.fifo_mode & 0x07) << 5);
    reg |= (lps22hb->config.fifo_watermark & 0x1F);

    err = lps22hb_write(lps22hb, LPS_REG_FIFO_CTRL_REG, &reg, 1);
    if (err != ESP_OK)

        return err;

    ESP_LOGI(TAG, "lps22hb_init compeltado");

    return ESP_OK;
}

static esp_err_t lps22hb_read_sample(lps22hb_t *lps22hb, lps22hb_data_t *data)
{
    if (lps22hb == NULL || data == NULL)
        return ESP_ERR_INVALID_ARG;

    uint8_t raw[5];

    esp_err_t err = lps22hb_read(lps22hb, LPS_REG_PRESS_OUT_XL_REG, raw, sizeof(raw));

    if (err != ESP_OK)
        return err;

    int32_t pressure_raw = ((int32_t)raw[2] << 16) |
                           ((int32_t)raw[1] << 8) |
                           raw[0];
    if (pressure_raw & 0x00800000)
    {
        pressure_raw |= 0xFF000000;
    }

    int16_t temperature_raw = ((int16_t)raw[4] << 8) | raw[3];

    data->pressure_hpa = pressure_raw / 4096.0f;
    data->pressure_pa = data->pressure_hpa * 100.0f;
    data->temperature_c = temperature_raw / 100.0f;

    return ESP_OK;
}

esp_err_t lps22hb_read_data(lps22hb_t *lps22hb)
{
    if (lps22hb == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return lps22hb_read_sample(lps22hb, &lps22hb->data);
}

esp_err_t lps22hb_get_fifo_status(
    lps22hb_t *lps22hb,
    uint8_t *samples,
    bool *empty,
    bool *full,
    bool *watermark,
    bool *overrun)
{
    if (lps22hb == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg;

    esp_err_t err = lps22hb_read(
        lps22hb,
        LPS_REG_FIFO_STATUS_REG,
        &reg,
        1);

    if (err != ESP_OK)
    {
        return err;
    }

    if (samples)
        *samples = reg & 0x1F;

    if (empty)
        *empty = ((reg >> 5) & 0x01);

    if (full)
        *full = ((reg >> 6) & 0x01);

    if (watermark)
        *watermark = ((reg >> 7) & 0x01);

    if (overrun)
        *overrun = false;

    return ESP_OK;
}

esp_err_t lps22hb_read_fifo_sample(lps22hb_t *lps22hb)
{
    if (lps22hb == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return lps22hb_read_sample(
        lps22hb,
        &lps22hb->data);
}

esp_err_t lps22hb_read_fifo(lps22hb_t *lps22hb, lps22hb_data_t *buffer, uint8_t max_samples, uint8_t *samples_read)
{
    if (lps22hb == NULL ||
        buffer == NULL ||
        samples_read == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t available;
    bool empty;
    bool full;
    bool watermark;
    bool overrun;

    esp_err_t err = lps22hb_get_fifo_status(
        lps22hb,
        &available,
        &empty,
        &full,
        &watermark,
        &overrun);

    if (err != ESP_OK)
    {
        return err;
    }

    uint8_t n = available;

    if (n > max_samples)
    {
        n = max_samples;
    }

    for (uint8_t i = 0; i < n; i++)
    {
        err = lps22hb_read_sample(
            lps22hb,
            &buffer[i]);

        if (err != ESP_OK)
        {
            return err;
        }
    }

    *samples_read = n;

    return ESP_OK;
}
