#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// #include "USBCDC.h"   // para usar USBCDC

// ══════════════════════════════════════════════════════════════════════
// STRUCTS
// ══════════════════════════════════════════════════════════════════════

typedef struct
{
    float roll;
    float pitch;
    float yaw;

    float qI;
    float qJ;
    float qK;
    float qR;
    float qAcc;

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;
} ImuData_t;



typedef struct
{
    uint32_t timestamp;
    float roll, pitch, yaw;
    float temp_amb, humedad; //
    float presion, temp_lps;//
    double lat, lon, alt;//
    uint8_t sats, fix;//
    float bms_v, bms_a;//
    uint8_t bms_soc;//
    uint16_t celdas[8];//
    int bms_t1, bms_t2;
} LogData_t;

extern ImuData_t ImuData;
extern LogData_t LogData;






// ══════════════════════════════════════════════════════════════════════
// FLAGS DE ESTADO
// ══════════════════════════════════════════════════════════════════════
extern bool sd_ok;
extern bool imu_ok;
extern bool sht_ok;
extern bool lps_ok;
extern bool gps_ok;
extern bool twai_ok;

// ══════════════════════════════════════════════════════════════════════
// DATOS SENSORES
// ══════════════════════════════════════════════════════════════════════

// SHT40
extern float temp_amb;
extern float humedad;

// LPS22
extern float presion;
extern float temp_lps;

// GPS
extern double gps_lat;
extern double gps_lon;
extern double gps_alt;
extern uint8_t gps_sats;
extern uint8_t gps_fix;

// BMS (protegidas por bmsMutex)
extern float bms_voltaje;
extern float bms_corriente;
extern uint8_t bms_soc;
extern uint16_t bms_celdas[8];
extern int bms_t1;
extern int bms_t2;
extern int bms_tmos;  //

// Relay
extern bool relay_state;

// ══════════════════════════════════════════════════════════════════════
// SINCRONIZACIÓN FreeRTOS
// ══════════════════════════════════════════════════════════════════════
extern SemaphoreHandle_t bmsMutex;
extern QueueHandle_t imuQueue;

// ══════════════════════════════════════════════════════════════════════
// TIMERS XBEE
// ══════════════════════════════════════════════════════════════════════
extern uint32_t t_sys;
extern uint32_t t_bat;
extern uint32_t t_env;
extern uint32_t t_bms;
extern uint32_t t_cells;
extern uint8_t heartbeat;

//extern USBCDC USBSerialIMU;
extern bool streaming; // controla el flujo de datos IMU por USB

