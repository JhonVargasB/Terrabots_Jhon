#pragma once

// ══════════════════════════════════════════════════════════════════════
// XBEE / SERIAL
// ══════════════════════════════════════════════════════════════════════
#define XBEE        Serial1
#define XBEE_TX     35
#define XBEE_RX     6
#define XBEE_BAUD   115200

#define XBEE_DEBUG  1   // 1 = imprime frames en ASCII por Serial, 0 = silencio

// ══════════════════════════════════════════════════════════════════════
// TAREAS FreeRTOS — períodos en ms
// ══════════════════════════════════════════════════════════════════════
#define PERIOD_IMU_MS       20      // 50 Hz
#define PERIOD_LOGGING_MS   1000    // 1 Hz
#define PERIOD_CAN_MS       10
#define PERIOD_SERIAL_MS    100
#define PERIOD_BLINK_MS     500
#define PERIOD_XBEE_MS      50      // resolución del scheduler XBee

// ══════════════════════════════════════════════════════════════════════
// XBEE — intervalos de envío en ms
// ══════════════════════════════════════════════════════════════════════
#define XBEE_INTERVAL_SYS_IMU   500
#define XBEE_INTERVAL_BAT_GPS   1000
#define XBEE_INTERVAL_ENV       2000
#define XBEE_INTERVAL_BMS       5000
#define XBEE_INTERVAL_CELLS     10000

// ══════════════════════════════════════════════════════════════════════
// TWAI CAN
// ══════════════════════════════════════════════════════════════════════
#define CAN_ID_BMS_MAIN     0x02F4
#define CAN_ID_BMS_CELLS_LO 0x18E028F4
#define CAN_ID_BMS_CELLS_HI 0x18E128F4
#define CAN_ID_BMS_TEMPS    0x18F228F4  

// ══════════════════════════════════════════════════════════════════════
// SD
// ══════════════════════════════════════════════════════════════════════
#define DATALOG_PATH    "/sdcard/datalog.csv"

#define DATALOG_HEADER  "millis,roll,pitch,yaw,temp_amb,humedad,presion,temp_lps," \
                        "lat,lon,alt,sats,fix," \
                        "bms_v,bms_a,bms_soc," \
                        "c1,c2,c3,c4,c5,c6,c7,c8,bms_t1,bms_t2"
