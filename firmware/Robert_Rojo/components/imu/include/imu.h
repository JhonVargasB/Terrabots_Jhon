#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "imu_defs.h"
#include "driver/i2c_master.h"

#define BNO08X_DEFAULT_REPORT_INTERVAL_US 10000U

typedef struct
{
    bno08x_vector3_t accel;
    bno08x_vector3_t gyro;
    bno08x_vector3_t mag;
    bno08x_vector3_t linear_accel;
    bno08x_vector3_t gravity;

    bno08x_quaternion_t rotation_vector;
    bno08x_quaternion_t game_rotation_vector;
    bno08x_quaternion_t geomag_rotation_vector;
    bno08x_quaternion_t gyro_rotation_vector;

    uint32_t step_count;

    bno08x_accuracy_t accel_accuracy;
    bno08x_accuracy_t gyro_accuracy;
    bno08x_accuracy_t mag_accuracy;
    bno08x_accuracy_t linear_accel_accuracy;
    bno08x_accuracy_t gravity_accuracy;
    bno08x_accuracy_t rotation_accuracy;
    bno08x_accuracy_t game_rotation_accuracy;
    bno08x_accuracy_t geomag_rotation_accuracy;
    bno08x_accuracy_t gyro_rotation_accuracy;

    int64_t accel_timestamp_us;
    int64_t gyro_timestamp_us;
    int64_t mag_timestamp_us;
    int64_t linear_accel_timestamp_us;
    int64_t gravity_timestamp_us;
    int64_t rotation_timestamp_us;
    int64_t game_rotation_timestamp_us;
    int64_t geomag_rotation_timestamp_us;
    int64_t gyro_rotation_timestamp_us;

    bool accel_updated;
    bool gyro_updated;
    bool mag_updated;
    bool linear_accel_updated;
    bool gravity_updated;
    bool rotation_updated;
    bool game_rotation_updated;
    bool geomag_rotation_updated;
    bool gyro_rotation_updated;
    bool step_count_updated;

} bno08x_data_t;

typedef struct
{

    uint8_t device_address;
    uint32_t scl_speed_hz;
    i2c_master_bus_handle_t bus_handle;

    gpio_num_t int_pin;
    gpio_num_t rst_pin;
    gpio_num_t boot_pin;

    bool enable_accel;
    bool enable_gyro;
    bool enable_mag;
    bool enable_rotation_vector;
    bool enable_linear_accel;
    bool enable_gravity;

    uint32_t accel_interval_us;
    uint32_t gyro_interval_us;
    uint32_t mag_interval_us;
    uint32_t rotation_interval_us;
    uint32_t linear_accel_interval_us;
    uint32_t gravity_interval_us;

} bno08x_config_t;

typedef struct
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t i2c_dev;
    i2c_device_config_t dev_cfg;

    bno08x_config_t config;
    bno08x_data_t data;

    uint8_t tx_seq[6];
    uint8_t rx_seq[6];

    uint8_t tx_buffer[512];
    uint8_t rx_buffer[512];
    uint16_t rx_length;

    int64_t irq_timestamp_us;
    int32_t timebase_delta_us;
    uint8_t command_seq;
    bool reset_detected;

} bno08x_t;

// Inicializa el BNO08x: registra el dispositivo I2C, configura INT/RST,
// reinicia el sensor, valida Product ID y habilita los reports del config.
esp_err_t bno08x_init(bno08x_t *bno08x, const bno08x_config_t *config);

// Espera un evento por INT, lee un paquete SHTP y actualiza bno08x->data.
// Devuelve ESP_ERR_TIMEOUT si no llega interrupcion dentro del timeout interno.
esp_err_t bno08x_read_data(bno08x_t *bno08x);

// Entrega una referencia de solo lectura a la ultima muestra parseada.
// Los flags *_updated indican que campos cambiaron en la ultima lectura.
const bno08x_data_t *bno08x_get_data(const bno08x_t *bno08x);

// Reenvia los reportes habilitados en config. Usalo despues de un reset del BNO08x.
esp_err_t bno08x_set_reports(bno08x_t *bno08x);

// Devuelve true una sola vez cuando el driver detecta que el BNO08x reinicio.
bool bno08x_was_reset(bno08x_t *bno08x);

// Convierte un quaternion del BNO08x a angulos Euler en grados.
void bno08x_quat_to_euler_deg(const bno08x_quaternion_t *quat,
                              float *roll_deg,
                              float *pitch_deg,
                              float *yaw_deg);

// Comandos SH-2 de tare/calibracion. En este nivel se envia el comando;
// la confirmacion fina requiere leer el COMMAND_RESPONSE del sensor.
esp_err_t bno08x_tare_now(bno08x_t *bno08x, bool z_only);
esp_err_t bno08x_save_tare(bno08x_t *bno08x);
esp_err_t bno08x_clear_tare(bno08x_t *bno08x);
esp_err_t bno08x_save_calibration(bno08x_t *bno08x);
