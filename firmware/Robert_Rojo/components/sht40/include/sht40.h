#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "sht40_defs.h"


typedef struct
{

    sht4xa_data_t data;

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t i2c_dev;
    i2c_device_config_t dev_cfg;

} sht4xa_t;




esp_err_t sht4xa_init(sht4xa_t *sht4xa);

esp_err_t sht4xa_measure_blocking_read(sht4xa_t *sht4xa, sht4xa_precision_t precision);

esp_err_t sht4xa_read(sht4xa_t *sht4xa);

esp_err_t sht4xa_read_serial(sht4xa_t *sht4xa);

esp_err_t sht4xa_soft_reset(sht4xa_t *sht4xa);

