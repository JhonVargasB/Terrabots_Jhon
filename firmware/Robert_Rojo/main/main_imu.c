#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "imu.h"
#include "shared.h"
#include "usb_serial.h"
#include "V1_Rover_Pins.h"

static const char *TAG = "main_imu";

static bno08x_t imu = {0};
static i2c_master_bus_handle_t i2c_bus = NULL;
static usb_serial_t USBSerialIMU;

static uint32_t millis_now(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// Reenvia los reports activos. Esto es necesario si el BNO08x se reinicia.
void imuSetReports(void)
{
    esp_err_t err = bno08x_set_reports(&imu);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "imuSetReports failed: %s", esp_err_to_name(err));
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}

// Lee un evento del BNO08x, actualiza el ultimo ImuData_t y lo publica por queue/USB.
void readIMUAndSend(void)
{
    if (!imu_ok)
        return;

    static uint32_t lastResetCheck = 0;
    static ImuData_t data = {0};

    uint32_t now = millis_now();
    if ((now - lastResetCheck) > 1000)
    {
        if (bno08x_was_reset(&imu))
        {
            ESP_LOGW(TAG, "BNO08x reset detected, re-enabling reports");
            imuSetReports();
        }
        lastResetCheck = now;
    }

    esp_err_t err = bno08x_read_data(&imu);
    if (err == ESP_ERR_TIMEOUT)
        return;

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "bno08x_read_data failed: %s", esp_err_to_name(err));
        return;
    }

    const bno08x_data_t *sensor = bno08x_get_data(&imu);
    if (sensor == NULL)
        return;

    if (sensor->accel_updated)
    {
        data.ax = sensor->accel.x;
        data.ay = sensor->accel.y;
        data.az = sensor->accel.z;
    }

    if (sensor->gyro_updated)
    {
        data.gx = sensor->gyro.x;
        data.gy = sensor->gyro.y;
        data.gz = sensor->gyro.z;
    }

    if (sensor->rotation_updated)
    {
        data.qI = sensor->rotation_vector.i;
        data.qJ = sensor->rotation_vector.j;
        data.qK = sensor->rotation_vector.k;
        data.qR = sensor->rotation_vector.real;
        data.qAcc = sensor->rotation_vector.accuracy_rad;

        bno08x_quat_to_euler_deg(
            &sensor->rotation_vector,
            &data.roll,
            &data.pitch,
            &data.yaw);

        if (imuQueue != NULL)
        {
            xQueueOverwrite(imuQueue, &data);
        }

        if (streaming && usb_serial_connected(&USBSerialIMU))
        {
            usb_serial_printf(&USBSerialIMU,
                              "%.6f,%.6f,%.6f,%.6f,%.3f,",
                              data.qI,
                              data.qJ,
                              data.qK,
                              data.qR,
                              data.qAcc);

            usb_serial_printf(&USBSerialIMU,
                              "%.3f,%.3f,%.3f,",
                              data.ax,
                              data.ay,
                              data.az);

            usb_serial_printf(&USBSerialIMU,
                              "%.5f,%.5f,%.5f\n",
                              data.gx,
                              data.gy,
                              data.gz);
        }
    }
}

bool IMU_tareNow(bool zOnly)
{
    if (!imu_ok)
        return false;

    return bno08x_tare_now(&imu, zOnly) == ESP_OK;
}

bool IMU_saveTare(void)
{
    if (!imu_ok)
        return false;

    return bno08x_save_tare(&imu) == ESP_OK;
}

bool IMU_clearTare(void)
{
    if (!imu_ok)
        return false;

    return bno08x_clear_tare(&imu) == ESP_OK;
}

bool IMU_saveCalibration(void)
{
    if (!imu_ok)
        return false;

    return bno08x_save_calibration(&imu) == ESP_OK;
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

    bno08x_config_t imu_config = {
        .device_address = BNO08X_I2C_ADDR_LO,
        .scl_speed_hz = 400000,
        .bus_handle = i2c_bus,
        .int_pin = BNO_INT,
        .rst_pin = BNO_RST,
        .boot_pin = GPIO_NUM_NC,
        .enable_accel = true,
        .enable_gyro = true,
        .enable_mag = true,
        .enable_rotation_vector = true,
        .enable_linear_accel = true,
        .enable_gravity = true,
        .accel_interval_us = 20000,
        .gyro_interval_us = 20000,
        .mag_interval_us = 20000,
        .rotation_interval_us = 20000,
        .linear_accel_interval_us = 20000,
        .gravity_interval_us = 20000,
    };

    imuQueue = xQueueCreate(1, sizeof(ImuData_t));
    ESP_ERROR_CHECK(usb_serial_init(&USBSerialIMU, TINYUSB_CDC_ACM_0, "IMU"));

    esp_err_t err = bno08x_init(&imu, &imu_config);
    if (err == ESP_OK)
    {
        imu_ok = true;
        ESP_LOGI(TAG, "IMU ready");
    }
    else
    {
        imu_ok = false;
        ESP_LOGE(TAG, "IMU init failed: %s", esp_err_to_name(err));
    }

    while (1)
    {
        readIMUAndSend();
        if (!imu_ok)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
