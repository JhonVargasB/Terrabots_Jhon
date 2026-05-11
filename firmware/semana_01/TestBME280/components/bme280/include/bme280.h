#ifndef BME280_H
#define BME280_H

#include "esp_log.h"
#include "driver/i2c_master.h"
#include "bme280_defs.h"

typedef struct
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t i2c_dev;
    i2c_device_config_t dev_cfg;
    bme280_calib_data calib;
    bme280_uncomp_data uncomp;
    bme280_data data;
} bme280_t;

esp_err_t bme280_init(bme280_t *bme280);

esp_err_t bme280_get_data(bme280_t *bme280);
    #endif