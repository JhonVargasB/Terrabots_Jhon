#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

#include "gps_defs.h"

typedef struct
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t i2c_dev;
    i2c_device_config_t dev_cfg;

    gps_config_t config;
    gps_data_t data;

    gps_device_info_t info;
    gps_parser_t parser;

    bool initialized;

} gps_t;

#define GPS_DEFAULT_CONFIG()                           \
    ((gps_config_t)                                    \
    {                                                  \
        .i2c_address = GPS_I2C_ADDRESS_DEFAULT,        \
        .i2c_clock_hz = GPS_I2C_MAX_CLOCK_HZ,          \
                                                       \
        .reset_pin = GPIO_NUM_NC,                      \
        .safeboot_pin = GPIO_NUM_NC,                   \
                                                       \
        .measurement_period_ms = 200,                  \
        .navigation_ratio = 1,                         \
        .nav_pvt_output_rate = 1,                      \
                                                       \
        .dynamic_model = GPS_DYN_MODEL_PORTABLE,       \
        .configure_dynamic_model = false,              \
                                                       \
        .configuration_layers = GPS_CFG_LAYER_RAM,     \
        .disable_nmea_output = true,                   \
                                                       \
        .io_timeout_ms = 100,                          \
        .read_chunk_size = 128                         \
    })

/*
 * Inicializa el receptor u-blox por I2C/DDC.
 *
 * Antes de llamar esta funcion, el usuario debe crear el bus I2C y asignar:
 *   gps.bus_handle = bus;
 *   gps.config = GPS_DEFAULT_CONFIG();
 *
 * La inicializacion prueba la direccion I2C, crea el device handle, habilita
 * UBX por I2C, lee MON-VER/SEC-UNIQID cuando sea posible y configura NAV-PVT.
 */
esp_err_t gps_init(gps_t *gps);

/*
 * Lee los bytes pendientes del GPS y procesa tramas UBX.
 *
 * Si llega una trama UBX-NAV-PVT valida, actualiza gps->data y coloca
 * *new_data = true. Si no hay bytes pendientes, retorna ESP_OK con
 * *new_data = false.
 */
esp_err_t gps_update(gps_t *gps, bool *new_data);

/*
 * Copia la ultima muestra NAV-PVT decodificada.
 *
 * Retorna ESP_ERR_NOT_FOUND si el GPS esta inicializado pero todavia no se ha
 * recibido ningun NAV-PVT valido.
 */
esp_err_t gps_get_data(const gps_t *gps, gps_data_t *data);

/*
 * Copia la informacion leida del receptor, como MON-VER y unique ID.
 */
esp_err_t gps_get_device_info(const gps_t *gps, gps_device_info_t *info);

/*
 * Indica si la ultima posicion recibida tiene fix GNSS valido y coordenadas
 * LLH utilizables segun UBX-NAV-PVT.
 */
bool gps_has_valid_fix(const gps_t *gps);

/*
 * Indica si la ultima muestra NAV-PVT no supera max_age_ms.
 */
bool gps_data_is_fresh(const gps_t *gps, uint32_t max_age_ms);

/*
 * Remueve el device I2C del bus y limpia el estado interno del driver.
 * No elimina el bus I2C porque normalmente puede estar compartido.
 */
esp_err_t gps_deinit(gps_t *gps);
