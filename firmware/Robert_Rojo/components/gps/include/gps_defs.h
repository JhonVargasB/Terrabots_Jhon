#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"

/* -------------------------------------------------------------------------- */
/*                              I2C constants                                 */
/* -------------------------------------------------------------------------- */

#define GPS_I2C_ADDRESS_DEFAULT          0x42U
#define GPS_I2C_MAX_CLOCK_HZ             400000U

#define GPS_I2C_REG_AVAILABLE_MSB        0xFDU
#define GPS_I2C_REG_AVAILABLE_LSB        0xFEU
#define GPS_I2C_REG_DATA_STREAM          0xFFU


//_________________________________________________________
#define GPS_FLUSH_TIMEOUT_MS  100U
#define GPS_FLUSH_CHUNK_SIZE  128U

#define GPS_UBX_CLASS_MON              0x0AU
#define GPS_UBX_ID_MON_VER             0x04U

#define GPS_UBX_CLASS_SEC              0x27U
#define GPS_UBX_ID_SEC_UNIQID          0x03U

#define GPS_UNIQUE_ID_LENGTH           5U
#define GPS_SW_VERSION_LENGTH          30U
#define GPS_HW_VERSION_LENGTH          10U

#define GPS_COMMAND_TIMEOUT_MS         1500U
#define GPS_STARTUP_DELAY_MS           500U
#define GPS_RESET_PULSE_MS             10U

#define GPS_ACK_READ_CHUNK_SIZE           64U
#define GPS_UBX_MAX_ACCEPTED_PAYLOAD      2048U
/* -------------------------------------------------------------------------- */
/*                              UBX constants                                 */
/* -------------------------------------------------------------------------- */

#define GPS_UBX_SYNC_1                   0xB5U
#define GPS_UBX_SYNC_2                   0x62U

#define GPS_UBX_CLASS_NAV                0x01U
#define GPS_UBX_CLASS_ACK                0x05U
#define GPS_UBX_CLASS_CFG                0x06U

#define GPS_UBX_ID_NAV_PVT               0x07U

#define GPS_UBX_ID_ACK_NAK               0x00U
#define GPS_UBX_ID_ACK_ACK               0x01U

#define GPS_UBX_ID_CFG_VALSET            0x8AU
#define GPS_UBX_ID_CFG_VALGET            0x8BU
#define GPS_UBX_ID_CFG_VALDEL            0x8CU

#define GPS_NAV_PVT_PAYLOAD_LENGTH       92U
#define GPS_UBX_MAX_PAYLOAD_LENGTH       512U

/* -------------------------------------------------------------------------- */
/*                           Configuration Key IDs                            */
/* -------------------------------------------------------------------------- */

#define GPS_CFG_I2C_ADDRESS              0x20510001UL
#define GPS_CFG_I2C_ENABLED              0x10510003UL

#define GPS_CFG_I2C_IN_UBX               0x10710001UL
#define GPS_CFG_I2C_IN_NMEA              0x10710002UL
#define GPS_CFG_I2C_IN_RTCM3X            0x10710004UL

#define GPS_CFG_I2C_OUT_UBX              0x10720001UL
#define GPS_CFG_I2C_OUT_NMEA             0x10720002UL

#define GPS_CFG_RATE_MEAS                0x30210001UL
#define GPS_CFG_RATE_NAV                 0x30210002UL
#define GPS_CFG_RATE_TIMEREF             0x20210003UL

#define GPS_CFG_MSGOUT_NAV_PVT_I2C       0x20910006UL

#define GPS_CFG_NAVSPG_DYNMODEL          0x20110021UL


#define GPS_RX_CHUNK_SIZE                       128U


/* UBX-NAV-PVT: byte "valid", offset 11 */
#define GPS_NAV_PVT_VALID_DATE_MASK             (1U << 0)
#define GPS_NAV_PVT_VALID_TIME_MASK             (1U << 1)
#define GPS_NAV_PVT_FULLY_RESOLVED_MASK          (1U << 2)

/* UBX-NAV-PVT: byte "flags", offset 21 */
#define GPS_NAV_PVT_GNSS_FIX_OK_MASK             (1U << 0)
#define GPS_NAV_PVT_DIFF_SOLN_MASK               (1U << 1)

/* UBX-NAV-PVT: campo flags3, offset 78 */
#define GPS_NAV_PVT_INVALID_LLH_MASK             (1U << 0)


/* -------------------------------------------------------------------------- */
/*                         Configuration layers                               */
/* -------------------------------------------------------------------------- */

typedef uint8_t gps_cfg_layers_t;

#define GPS_CFG_LAYER_RAM                (1U << 0)
#define GPS_CFG_LAYER_BBR                (1U << 1)
#define GPS_CFG_LAYER_FLASH              (1U << 2)

/* -------------------------------------------------------------------------- */
/*                           Dynamic platform model                           */
/* -------------------------------------------------------------------------- */

typedef enum
{
    GPS_DYN_MODEL_PORTABLE      = 0,
    GPS_DYN_MODEL_STATIONARY    = 2,
    GPS_DYN_MODEL_PEDESTRIAN    = 3,
    GPS_DYN_MODEL_AUTOMOTIVE    = 4,
    GPS_DYN_MODEL_SEA           = 5,
    GPS_DYN_MODEL_AIRBORNE_1G   = 6,
    GPS_DYN_MODEL_AIRBORNE_2G   = 7,
    GPS_DYN_MODEL_AIRBORNE_4G   = 8,
    GPS_DYN_MODEL_WRIST         = 9

} gps_dynamic_model_t;

/* -------------------------------------------------------------------------- */
/*                                Fix type                                    */
/* -------------------------------------------------------------------------- */

typedef enum
{
    GPS_FIX_NONE               = 0,
    GPS_FIX_DEAD_RECKONING     = 1,
    GPS_FIX_2D                 = 2,
    GPS_FIX_3D                 = 3,
    GPS_FIX_GNSS_DR_COMBINED   = 4,
    GPS_FIX_TIME_ONLY          = 5

} gps_fix_type_t;

/* -------------------------------------------------------------------------- */
/*                           User configuration                               */
/* -------------------------------------------------------------------------- */

typedef struct
{
    uint8_t i2c_address;
    uint32_t i2c_clock_hz;

    gpio_num_t reset_pin;
    gpio_num_t safeboot_pin;

    /*
     * Measurement period:
     * 1000 ms = 1 Hz
     *  200 ms = 5 Hz
     *  100 ms = 10 Hz
     */
    uint16_t measurement_period_ms;

    /*
     * Number of measurements per navigation solution.
     * Normally 1.
     */
    uint16_t navigation_ratio;

    /*
     * NAV-PVT output rate:
     * 0 = disabled
     * 1 = every navigation epoch
     * 2 = every two navigation epochs
     */
    uint8_t nav_pvt_output_rate;

    gps_dynamic_model_t dynamic_model;
    bool configure_dynamic_model;

    /*
     * RAM during development.
     * RAM | FLASH once verified.
     */
    gps_cfg_layers_t configuration_layers;

    bool disable_nmea_output;

    uint32_t io_timeout_ms;
    size_t read_chunk_size;

} gps_config_t;

/* -------------------------------------------------------------------------- */
/*                               GPS data                                     */
/* -------------------------------------------------------------------------- */

typedef struct
{
    /* Navigation epoch */
    uint32_t i_tow_ms;

    /* UTC date and time */
    uint16_t year;
    uint8_t month;
    uint8_t day;

    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    int32_t nano;

    bool valid_date;
    bool valid_time;
    bool fully_resolved;

    /* Fix information */
    gps_fix_type_t fix_type;
    uint8_t satellites_used;

    bool gnss_fix_ok;
    bool differential_solution;
    bool position_valid;

    /* Position */
    double latitude_deg;
    double longitude_deg;

    float height_ellipsoid_m;
    float altitude_msl_m;

    float horizontal_accuracy_m;
    float vertical_accuracy_m;

    /* Velocity in NED coordinates */
    float velocity_north_mps;
    float velocity_east_mps;
    float velocity_down_mps;

    float ground_speed_mps;
    float heading_motion_deg;

    float speed_accuracy_mps;
    float heading_accuracy_deg;

    float position_dop;

    /* Raw status fields, useful for debugging */
    uint8_t flags;
    uint8_t flags2;
    uint16_t flags3;

    /* Driver status */
    bool new_data;
    uint32_t update_counter;
    int64_t last_update_us;

} gps_data_t;


typedef struct
{
    uint8_t unique_id[GPS_UNIQUE_ID_LENGTH];

    char software_version[GPS_SW_VERSION_LENGTH + 1U];
    char hardware_version[GPS_HW_VERSION_LENGTH + 1U];

    bool unique_id_valid;
    bool version_valid;

} gps_device_info_t;


typedef enum
{
    GPS_PARSE_WAIT_SYNC_1 = 0,
    GPS_PARSE_WAIT_SYNC_2,
    GPS_PARSE_CLASS,
    GPS_PARSE_ID,
    GPS_PARSE_LENGTH_LSB,
    GPS_PARSE_LENGTH_MSB,
    GPS_PARSE_PAYLOAD,
    GPS_PARSE_CHECKSUM_A,
    GPS_PARSE_CHECKSUM_B

} gps_parser_state_t;


typedef enum
{
    GPS_ACK_PARSE_SYNC_1 = 0,
    GPS_ACK_PARSE_SYNC_2,
    GPS_ACK_PARSE_CLASS,
    GPS_ACK_PARSE_ID,
    GPS_ACK_PARSE_LENGTH_LSB,
    GPS_ACK_PARSE_LENGTH_MSB,
    GPS_ACK_PARSE_PAYLOAD,
    GPS_ACK_PARSE_CK_A,
    GPS_ACK_PARSE_CK_B

} gps_ack_parser_state_t;

typedef struct
{
    gps_ack_parser_state_t state;

    uint8_t message_class;
    uint8_t message_id;

    uint16_t payload_length;
    uint16_t payload_index;

    /*
     * Solo necesitamos guardar los dos primeros bytes,
     * porque ACK-ACK y ACK-NAK tienen payload de 2 bytes.
     */
    uint8_t payload[2];

    uint8_t checksum_a;
    uint8_t checksum_b;
    uint8_t received_checksum_a;

} gps_ack_parser_t;


typedef enum
{
    GPS_PARSE_RESULT_NONE = 0,
    GPS_PARSE_RESULT_FRAME_COMPLETE,
    GPS_PARSE_RESULT_CHECKSUM_ERROR

} gps_parse_result_t;


typedef struct
{
    gps_parser_state_t state;

    uint8_t message_class;
    uint8_t message_id;

    uint16_t payload_length;
    uint16_t payload_index;

    uint8_t checksum_a;
    uint8_t checksum_b;
    uint8_t received_checksum_a;

    bool payload_overflow;

    uint8_t payload[GPS_UBX_MAX_PAYLOAD_LENGTH];

} gps_parser_t;
