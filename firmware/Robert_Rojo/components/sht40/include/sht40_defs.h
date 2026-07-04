#pragma once

#include <stdint.h>

 //I2C addresses

#define SHT4XA_I2C_ADDR_DEFAULT       0x44
#define SHT4XA_I2C_ADDR_ALT_1         0x45
#define SHT4XA_I2C_ADDR_ALT_2         0x46

//Commands

#define SHT4XA_CMD_MEASURE_HIGH       0xFD   // T&RH with high precision, 2B_T - 1B_CRC - 2B_RH - 1B_CRC
#define SHT4XA_CMD_MEASURE_MEDIUM     0xF6   // T&RH with medium precision,  2B_T - 1B_CRC - 2B_RH - 1B_CRC
#define SHT4XA_CMD_MEASURE_LOW        0xE0   // T&RH with low precision,  2B_T - 1B_CRC - 2B_RH - 1B_CRC

#define SHT4XA_CMD_READ_SERIAL        0x89   // 2B_SERIAL - 1B_CRC - 2B_SERIAL - 1B_CRC
#define SHT4XA_CMD_SOFT_RESET         0x94   //Soft Reset  [ACK]

#define SHT4XA_CMD_HEATER_200MW_1S    0x39   // Heater ON (200 mW)(1s) and  High Precision T&RH measurement
#define SHT4XA_CMD_HEATER_200MW_100MS 0x32   // Heater ON (200 mW)(0.1s) and High Precision T&RH measurement
#define SHT4XA_CMD_HEATER_110MW_1S    0x2F   // Heater ON (110 mW)(1.0s) and High Precision T&RH measurement
#define SHT4XA_CMD_HEATER_110MW_100MS 0x24   // Heater ON (110 mW)(0.1s) and High Precision T&RH measurement
#define SHT4XA_CMD_HEATER_20MW_1S     0x1E   // Heater ON (20 mW)(1.0s) and High Precision T&RH measurement
#define SHT4XA_CMD_HEATER_20MW_100MS  0x15   // Heater ON (20 mW)(0.1s) and High Precision T&RH measurement

/* =========================
 * Response lengths
 * ========================= */
#define SHT4XA_MEAS_RESPONSE_LEN      6
#define SHT4XA_SERIAL_RESPONSE_LEN    6

/* =========================
 * CRC-8
 * Polynomial: 0x31
 * Init:       0xFF
 * ========================= */
#define SHT4XA_CRC_POLYNOMIAL         0x31
#define SHT4XA_CRC_INIT               0xFF

/* =========================
 * Timing [ms]
 * Datasheet max:
 * Low:    1.6 ms
 * Medium: 4.5 ms
 * High:   8.3 ms
 * ========================= */
#define SHT4XA_MEAS_LOW_DELAY_MS      2
#define SHT4XA_MEAS_MEDIUM_DELAY_MS   5
#define SHT4XA_MEAS_HIGH_DELAY_MS     9

#define SHT4XA_RESET_DELAY_MS         1

#define SHT4XA_HEATER_100MS_DELAY_MS  110
#define SHT4XA_HEATER_1S_DELAY_MS     1100

/* =========================
 * Conversion constants
 * ========================= */
#define SHT4XA_RAW_MAX                65535.0f

#define SHT4XA_TEMP_OFFSET_C          (-45.0f)
#define SHT4XA_TEMP_SCALE_C           175.0f

#define SHT4XA_RH_OFFSET              (-6.0f)
#define SHT4XA_RH_SCALE               125.0f
#define SHT4XA_RH_MIN                 0.0f
#define SHT4XA_RH_MAX                 100.0f

/* =========================
 * Public enums
 * ========================= */
typedef enum
{
    SHT4XA_PRECISION_LOW = 0,
    SHT4XA_PRECISION_MEDIUM,
    SHT4XA_PRECISION_HIGH
} sht4xa_precision_t;

typedef enum
{
    SHT4XA_HEATER_POWER_20MW = 0,
    SHT4XA_HEATER_POWER_110MW,
    SHT4XA_HEATER_POWER_200MW
} sht4xa_heater_power_t;

typedef enum
{
    SHT4XA_HEATER_TIME_100MS = 0,
    SHT4XA_HEATER_TIME_1S
} sht4xa_heater_time_t;


typedef enum
{
    SHT4XA_STATE_IDLE = 0,
    SHT4XA_STATE_MEASURING,
    SHT4XA_STATE_HEATING,
    SHT4XA_STATE_ERROR
} sht4xa_state_t;

typedef struct 
{
    float temp;
    float humidity;
    uint32_t serial;
} sht4xa_data_t;



#define ESP_ERR_SHT4XA_CRC (ESP_ERR_INVALID_RESPONSE + 1)

