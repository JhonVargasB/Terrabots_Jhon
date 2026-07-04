#pragma once

#include <stdint.h>

/*=========================================================
 * I2C ADDRESS
 *========================================================*/

#define BNO08X_I2C_ADDR_LO 0x4A
#define BNO08X_I2C_ADDR_HI 0x4B


/*=========================================================
 * SHTP CHANNELS
 *========================================================*/

#define BNO08X_CHANNEL_COMMAND 0
#define BNO08X_CHANNEL_EXECUTABLE 1
#define BNO08X_CHANNEL_CONTROL 2
#define BNO08X_CHANNEL_INPUT_REPORTS 3
#define BNO08X_CHANNEL_WAKE_REPORTS 4
#define BNO08X_CHANNEL_GYRO_ROTATION 5

/*=========================================================
 * REPORT IDS
 *========================================================*/

#define BNO08X_REPORTID_ACCELEROMETER 0x01
#define BNO08X_REPORTID_GYROSCOPE 0x02
#define BNO08X_REPORTID_MAGNETOMETER 0x03
#define BNO08X_REPORTID_LINEAR_ACCELERATION 0x04
#define BNO08X_REPORTID_ROTATION_VECTOR 0x05
#define BNO08X_REPORTID_GRAVITY 0x06

#define BNO08X_REPORTID_GAME_ROTATION_VECTOR 0x08
#define BNO08X_REPORTID_GEOMAG_ROTATION_VECTOR 0x09

#define BNO08X_REPORTID_PRESSURE 0x0A
#define BNO08X_REPORTID_AMBIENT_LIGHT 0x0B
#define BNO08X_REPORTID_HUMIDITY 0x0C
#define BNO08X_REPORTID_PROXIMITY 0x0D
#define BNO08X_REPORTID_TEMPERATURE 0x0E

#define BNO08X_REPORTID_STEP_COUNTER 0x11
#define BNO08X_REPORTID_STABILITY_CLASSIFIER 0x13
#define BNO08X_REPORTID_SHAKE_DETECTOR 0x19

#define BNO08X_REPORTID_RAW_ACCELEROMETER 0x14
#define BNO08X_REPORTID_RAW_GYROSCOPE 0x15
#define BNO08X_REPORTID_RAW_MAGNETOMETER 0x16

#define BNO08X_REPORTID_GYRO_ROTATION_VECTOR 0x2A

#define BNO08X_REPORTID_TIMEBASE_REFERENCE 0xFB
/*=========================================================
 * COMMAND IDS
 *========================================================*/

#define BNO08X_COMMAND_ERRORS 0x01
#define BNO08X_COMMAND_COUNTER 0x02
#define BNO08X_COMMAND_TARE 0x03
#define BNO08X_COMMAND_INITIALIZE 0x04
#define BNO08X_COMMAND_DCD 0x06
#define BNO08X_COMMAND_ME_CALIBRATE 0x07
#define BNO08X_COMMAND_DCD_PERIOD_SAVE 0x09
#define BNO08X_COMMAND_OSCILLATOR 0x0A
#define BNO08X_COMMAND_CLEAR_DCD 0x0B

/*=========================================================
 * SH-2 CONTROL REPORT IDS
 * Channel: BNO08X_CHANNEL_CONTROL = 2
 *========================================================*/

#define BNO08X_CONTROL_GET_FEATURE_REQUEST 0xFE
#define BNO08X_CONTROL_SET_FEATURE_COMMAND 0xFD
#define BNO08X_CONTROL_GET_FEATURE_RESPONSE 0xFC

#define BNO08X_CONTROL_PRODUCT_ID_REQUEST 0xF9
#define BNO08X_CONTROL_PRODUCT_ID_RESPONSE 0xF8

#define BNO08X_CONTROL_FRS_WRITE_REQUEST 0xF7
#define BNO08X_CONTROL_FRS_WRITE_DATA 0xF6
#define BNO08X_CONTROL_FRS_WRITE_RESPONSE 0xF5

#define BNO08X_CONTROL_FRS_READ_REQUEST 0xF4
#define BNO08X_CONTROL_FRS_READ_RESPONSE 0xF3

#define BNO08X_CONTROL_COMMAND_REQUEST 0xF2
#define BNO08X_CONTROL_COMMAND_RESPONSE 0xF1

/*=========================================================
 * EXECUTABLE COMMANDS
 *========================================================*/

#define BNO08X_EXECUTABLE_RESET 1
#define BNO08X_EXECUTABLE_ON 2
#define BNO08X_EXECUTABLE_SLEEP 3

/*=========================================================
 * SENSOR ACCURACY
 *========================================================*/

typedef struct __attribute__((packed))
{
    uint8_t report_id;         // 0xFD
    uint8_t feature_report_id; // 0x05,0x01,etc
    uint8_t feature_flags;

    uint16_t change_sensitivity;

    uint32_t report_interval_us;

    uint32_t batch_interval_us;

    uint32_t sensor_specific;
} bno08x_set_feature_t;

typedef struct __attribute__((packed))
{
    uint8_t report_id;
    uint8_t sequence;

    uint8_t status_delay_msb;
    uint8_t delay_lsb;
} bno08x_report_header_t;


typedef struct __attribute__((packed))
{
    bno08x_report_header_t header;

    int16_t x;
    int16_t y;
    int16_t z;

} bno08x_vector3_report_t;

typedef enum
{
    BNO08X_ACCURACY_UNRELIABLE = 0,
    BNO08X_ACCURACY_LOW = 1,
    BNO08X_ACCURACY_MEDIUM = 2,
    BNO08X_ACCURACY_HIGH = 3
} bno08x_accuracy_t;


/*=========================================================
 * SHTP HEADER
 *========================================================*/

typedef struct __attribute__((packed))
{
    uint16_t length;
    uint8_t channel;
    uint8_t sequence;
} bno08x_shtp_header_t;

/*=========================================================
 * QUATERNION
 *========================================================*/

typedef struct
{
    float i;
    float j;
    float k;
    float real;
    float accuracy_rad;
} bno08x_quaternion_t;

/*=========================================================
 * VECTOR3
 *========================================================*/

typedef struct
{
    float x;
    float y;
    float z;
} bno08x_vector3_t;

static inline uint8_t bno08x_get_accuracy(
        const bno08x_report_header_t *h)
{
    return h->status_delay_msb & 0x03;
}

static inline uint16_t bno08x_get_delay_ticks(
        const bno08x_report_header_t *h)
{
    return
        (((uint16_t)(h->status_delay_msb >> 2)) << 8) |
        h->delay_lsb;
}

static inline uint32_t bno08x_get_delay_us(
        const bno08x_report_header_t *h)
{
    return bno08x_get_delay_ticks(h) * 100;
}