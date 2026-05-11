#ifndef BME280_DEFS_H
#define BME280_DEFS_H


#include <stdint.h>
//Address
#define BME280_I2C_ADDR_PRIM                      UINT8_C(0x76)
#define BME280_I2C_ADDR_SEC                       UINT8_C(0x77)

//Register Address
#define BME280_REG_CHIP_ID                        UINT8_C(0xD0)   //0X60 para BME280
#define BME280_REG_RESET                          UINT8_C(0xE0)
#define BME280_REG_CTRL_HUM                       UINT8_C(0xF2)   //Leer antes de escribir ojo
#define BME280_REG_STATUS                         UINT8_C(0xF3)
#define BME280_REG_PWR_CTRL                       UINT8_C(0xF4)
#define BME280_REG_CONFIG                         UINT8_C(0xF5)

//Calibration data
#define BME280_REG_TEMP_PRESS_CALIB_DATA          UINT8_C(0x88)  // 88 hasta A1
#define BME280_REG_HUMIDITY_CALIB_DATA            UINT8_C(0xE1)  // E1 hasta E7

//Data
//Presion:     0xF7[7:0] 0xF8[7:0] 0XF9[7:4]     _msb, _lsb, _xlsb
//Temp   :     0xFA[7:0] 0xFB[7:0] 0xFC[7:4]     _msb, _lsb, _xlsb
//Humedad:     0xFD[7:0] 0xFE[7:0]               _msb, _lsb
#define BME280_REG_DATA                           UINT8_C(0xF7)  //hasta el registro ...0xFE


/*  Soft reset command */
#define BME280_SOFT_RESET_COMMAND                 UINT8_C(0xB6)

/*  Sensor power modes */
#define BME280_POWERMODE_SLEEP                    UINT8_C(0x00)
#define BME280_POWERMODE_FORCED                   UINT8_C(0x01)
#define BME280_POWERMODE_NORMAL                   UINT8_C(0x03)

#define BME280_SENSOR_MODE_MSK                    UINT8_C(0x03)
#define BME280_SENSOR_MODE_POS                    UINT8_C(0x00)

/* Soft reset command */
#define BME280_SOFT_RESET_COMMAND                 UINT8_C(0xB6)

#define BME280_STATUS_IM_UPDATE                   UINT8_C(0x01)
#define BME280_STATUS_MEAS_DONE                   UINT8_C(0x08)

/*!  Oversampling macros */
#define BME280_NO_OVERSAMPLING                    UINT8_C(0x00)
#define BME280_OVERSAMPLING_1X                    UINT8_C(0x01)
#define BME280_OVERSAMPLING_2X                    UINT8_C(0x02)
#define BME280_OVERSAMPLING_4X                    UINT8_C(0x03)
#define BME280_OVERSAMPLING_8X                    UINT8_C(0x04)
#define BME280_OVERSAMPLING_16X                   UINT8_C(0x05)
#define BME280_OVERSAMPLING_MAX                   UINT8_C(16)


/* Standby duration selection macros */
#define BME280_STANDBY_TIME_0_5_MS                (0x00)
#define BME280_STANDBY_TIME_62_5_MS               (0x01)
#define BME280_STANDBY_TIME_125_MS                (0x02)
#define BME280_STANDBY_TIME_250_MS                (0x03)
#define BME280_STANDBY_TIME_500_MS                (0x04)
#define BME280_STANDBY_TIME_1000_MS               (0x05)
#define BME280_STANDBY_TIME_10_MS                 (0x06)
#define BME280_STANDBY_TIME_20_MS                 (0x07)

#define BME280_STANDBY_MSK                        UINT8_C(0xE0)
#define BME280_STANDBY_POS                        UINT8_C(0x05)

/* Filter coefficient selection macros */
#define BME280_FILTER_COEFF_OFF                   (0x00)
#define BME280_FILTER_COEFF_2                     (0x01)
#define BME280_FILTER_COEFF_4                     (0x02)
#define BME280_FILTER_COEFF_8                     (0x03)
#define BME280_FILTER_COEFF_16                    (0x04)

#define BME280_FILTER_MSK                         UINT8_C(0x1C)
#define BME280_FILTER_POS                         UINT8_C(0x02)


/*! @name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)



#define BME280_U16(lsb, msb) ((uint16_t)(((uint16_t)(msb) << 8) | (lsb)))
#define BME280_S16(lsb, msb) ((int16_t)(((uint16_t)(msb) << 8) | (lsb)))
#define BME280_U20(msb, lsb, xlsb) \
    ((uint32_t)(((uint32_t)(msb) << 12) | ((uint32_t)(lsb) << 4) | ((uint32_t)(xlsb) >> 4)))


/*!
 * @brief Calibration data
 */
typedef struct 
{
    /*! Calibration coefficient for the temperature sensor */
    uint16_t dig_t1;

    /*! Calibration coefficient for the temperature sensor */
    int16_t dig_t2;

    /*! Calibration coefficient for the temperature sensor */
    int16_t dig_t3;

    /*! Calibration coefficient for the pressure sensor */
    uint16_t dig_p1;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p2;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p3;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p4;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p5;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p6;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p7;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p8;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p9;

    /*! Calibration coefficient for the humidity sensor */
    uint8_t dig_h1;

    /*! Calibration coefficient for the humidity sensor */
    int16_t dig_h2;

    /*! Calibration coefficient for the humidity sensor */
    uint8_t dig_h3;

    /*! Calibration coefficient for the humidity sensor */
    int16_t dig_h4;

    /*! Calibration coefficient for the humidity sensor */
    int16_t dig_h5;

    /*! Calibration coefficient for the humidity sensor */
    int8_t dig_h6;

    /*! Variable to store the intermediate temperature coefficient */
    int32_t t_fine;
}bme280_calib_data;

/*!
 * @brief bme280 sensor structure which comprises of temperature, pressure and
 * humidity data
 */
typedef struct 
{
    /*! Compensated pressure */
    double pressure;

    /*! Compensated temperature */
    double temperature;

    /*! Compensated humidity */
    double humidity;
}bme280_data;

/*!
 * @brief bme280 sensor structure which comprises of uncompensated temperature,
 * pressure and humidity data
 */
typedef struct
{
    /*! Un-compensated pressure */
    uint32_t pressure;

    /*! Un-compensated temperature */
    uint32_t temperature;

    /*! Un-compensated humidity */
    uint32_t humidity;
}bme280_uncomp_data;

/*!
 * @brief bme280 sensor settings structure which comprises of mode,
 * oversampling and filter settings.
 */

typedef struct 
{
    /*! Pressure oversampling */
    uint8_t osr_p;

    /*! Temperature oversampling */
    uint8_t osr_t;

    /*! Humidity oversampling */
    uint8_t osr_h;

    /*! Filter coefficient */
    uint8_t filter;

    /*! Standby time */
    uint8_t standby_time;
    
    /*! mode time */
    uint8_t mode;
}bme280_settings;



#endif