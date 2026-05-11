#ifndef MPU_H
#define MPU_H

#include "esp_log.h"
#include "driver/i2c_master.h"
#include "mpu_defs.h"

typedef struct
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t i2c_dev;
    i2c_device_config_t dev_cfg;
    mpu6050_calib_data calib;
    mpu6050_settings config;
    mpu6050_scale scale;
} mpu6050_t;


esp_err_t mpu6050_init(mpu6050_t *mpu6050);
esp_err_t selftest_xg(mpu6050_t *mpu6050);
esp_err_t selftest_yg(mpu6050_t *mpu6050);
esp_err_t selftest_zg(mpu6050_t *mpu6050);
esp_err_t mpu6050_calibrate_accel(mpu6050_t *mpu6050);
esp_err_t mpu6050_read_accel(mpu6050_t *mpu6050);
esp_err_t mpu6050_read_gyro(mpu6050_t *mpu6050);
esp_err_t mpu6050_calibrate_gyro(mpu6050_t *mpu6050);
#endif