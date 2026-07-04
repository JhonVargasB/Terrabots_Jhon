#pragma once

#include <stdbool.h>
#include <stdint.h>

// Registros de configuración
#define LPS_REG_INTERRUPT_CFG_REG      0x0B
#define LPS_REG_THS_P_L_REG            0x0C
#define LPS_REG_THS_P_H_REG            0x0D

// who am I
#define LPS_REG_WHO_AM_I_REG           0x0F

// Control
#define LPS_REG_CTRL_REG1              0x10
#define LPS_REG_CTRL_REG2              0x11
#define LPS_REG_CTRL_REG3              0x12

// FIFO
#define LPS_REG_FIFO_CTRL_REG          0x14

// Referencia de presión
#define LPS_REG_REF_P_XL_REG           0x15
#define LPS_REG_REF_P_L_REG            0x16
#define LPS_REG_REF_P_H_REG            0x17

// Offset de presión
#define LPS_REG_RPDS_L_REG             0x18
#define LPS_REG_RPDS_H_REG             0x19

// Resolución
#define LPS_REG_RES_CONF_REG           0x1A

// Estado e interrupciones
#define LPS_REG_INT_SOURCE_REG         0x25
#define LPS_REG_FIFO_STATUS_REG        0x26
#define LPS_REG_STATUS_REG             0x27

// Salida de presión
#define LPS_REG_PRESS_OUT_XL_REG       0x28
#define LPS_REG_PRESS_OUT_L_REG        0x29
#define LPS_REG_PRESS_OUT_H_REG        0x2A

// Salida de temperatura
#define LPS_REG_TEMP_OUT_L_REG         0x2B
#define LPS_REG_TEMP_OUT_H_REG         0x2C

// Resolución LPFP
#define LPS_REG_LPFP_RES_REG           0x33


#define LPS_WHO_AM_I_VALUE  0xB1

typedef enum
{
    LPS22HB_ODR_POWER_DOWN = 0,
    LPS22HB_ODR_1_HZ,
    LPS22HB_ODR_10_HZ,
    LPS22HB_ODR_25_HZ,
    LPS22HB_ODR_50_HZ,
    LPS22HB_ODR_75_HZ
} lps22hb_odr_t;


typedef enum
{
    LPS22HB_LPFP_DIV_2 = 0,
    LPS22HB_LPFP_DIV_9 = 1
} lps22hb_lpfp_cfg_t;


typedef enum
{
    LPS22HB_FIFO_BYPASS = 0,
    LPS22HB_FIFO_FIFO,
    LPS22HB_FIFO_STREAM,
    LPS22HB_FIFO_STREAM_DYNAMIC,
    LPS22HB_FIFO_STREAM_TO_FIFO,
    LPS22HB_FIFO_BYPASS_TO_STREAM,
    LPS22HB_FIFO_BYPASS_TO_FIFO
} lps22hb_fifo_mode_t;


typedef struct
{
    float pressure_hpa;
    float pressure_pa;

    float temperature_c;

} lps22hb_data_t;



typedef struct
{
    uint8_t device_address;

    uint32_t scl_speed_hz;
    uint32_t scl_wait_us;

    lps22hb_odr_t odr;

    bool low_current_mode;

    bool low_pass_filter_enable;
    lps22hb_lpfp_cfg_t low_pass_filter_cfg;

    bool block_data_update;

    bool auto_increment;

    lps22hb_fifo_mode_t fifo_mode;
    uint8_t fifo_watermark;

    bool interrupt_latch;
    bool interrupt_low_pressure;
    bool interrupt_high_pressure;

    uint16_t pressure_threshold;

} lps22hb_config_t;

