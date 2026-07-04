#include <stdio.h>
#include "shared.h"



// ══════════════════════════════════════════════════════════════════════
// FLAGS DE ESTADO
// ══════════════════════════════════════════════════════════════════════
bool sd_ok   = false;
bool imu_ok  = false;
bool sht_ok  = false;
bool lps_ok  = false;
bool gps_ok  = false;
bool twai_ok = false;
bool streaming = false;

// ══════════════════════════════════════════════════════════════════════
// DATOS SENSORES
// ══════════════════════════════════════════════════════════════════════
ImuData_t ImuData = {0};
LogData_t LogData = {0};



float temp_amb = 0, humedad = 0;
float presion  = 0, temp_lps = 0;

double  gps_lat  = 0, gps_lon = 0, gps_alt = 0;
uint8_t gps_sats = 0, gps_fix = 0;

float    bms_voltaje   = 0;
float    bms_corriente = 0;
uint8_t  bms_soc       = 0;
uint16_t bms_celdas[8] = {0};
int      bms_t1 = 0, bms_t2 = 0;


 int bms_tmos = 0;

bool relay_state = false;


// ══════════════════════════════════════════════════════════════════════
// SINCRONIZACIÓN FreeRTOS
// ══════════════════════════════════════════════════════════════════════
SemaphoreHandle_t bmsMutex = NULL;
QueueHandle_t     imuQueue = NULL;

// ══════════════════════════════════════════════════════════════════════
// TIMERS XBEE
// ══════════════════════════════════════════════════════════════════════
uint32_t t_sys   = 0;
uint32_t t_bat   = 0;
uint32_t t_env   = 0;
uint32_t t_bms   = 0;
uint32_t t_cells = 0;
uint8_t  heartbeat = 0;