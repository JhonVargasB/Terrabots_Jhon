#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

#include "lps_defs.h"

typedef struct
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t i2c_dev;
    i2c_device_config_t dev_cfg;

    lps22hb_config_t config;

    lps22hb_data_t data;

} lps22hb_t;

#define LPS22HB_DEFAULT_CONFIG()                   \
    {                                              \
        .device_address = 0x5C,                    \
                                                   \
        .scl_speed_hz = 100000,                    \
        .scl_wait_us = 1000,                       \
                                                   \
        .odr = LPS22HB_ODR_10_HZ,                  \
                                                   \
        .low_current_mode = false,                 \
                                                   \
        .low_pass_filter_enable = true,            \
        .low_pass_filter_cfg = LPS22HB_LPFP_DIV_9, \
                                                   \
        .block_data_update = true,                 \
                                                   \
        .auto_increment = true,                    \
                                                   \
        .fifo_mode = LPS22HB_FIFO_BYPASS,          \
        .fifo_watermark = 0,                       \
                                                   \
        .interrupt_latch = false,                  \
        .interrupt_low_pressure = false,           \
        .interrupt_high_pressure = false,          \
                                                   \
        .pressure_threshold = 0}

esp_err_t lps22hb_init(lps22hb_t *lps22hb);
esp_err_t lps22hb_read_fifo(lps22hb_t *lps22hb, lps22hb_data_t *buffer, uint8_t max_samples, uint8_t *samples_read);

esp_err_t lps22hb_read_fifo_sample(lps22hb_t *lps22hb);
esp_err_t lps22hb_get_fifo_status(lps22hb_t *lps22hb, uint8_t *samples, bool *empty, bool *full, bool *watermark, bool *overrun);
esp_err_t lps22hb_read_data(lps22hb_t *lps22hb);
