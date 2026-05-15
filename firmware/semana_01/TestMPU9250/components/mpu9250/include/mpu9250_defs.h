#ifndef MPU9250_DEFS_H
#define MPU9250_DEFS_H

// Gyroscope Self-Test Registers
#define SELF_TEST_X_GYRO UINT8_C(0x00)
#define SELF_TEST_Y_GYRO UINT8_C(0x01)
#define SELF_TEST_Z_GYRO UINT8_C(0x02)

#define SELF_TEST_X_ACCEL UINT8_C(0x0D)
#define SELF_TEST_Y_ACCEL UINT8_C(0x0E)
#define SELF_TEST_Z_ACCEL UINT8_C(0x0F)

//Gyro Offset Registers

#define XG_OFFSET_H UINT8_C(0x13)
#define XG_OFFSET_L UINT8_C(0x14)
#define YG_OFFSET_H UINT8_C(0Y15)
#define YG_OFFSET_L UINT8_C(0x16)
#define ZG_OFFSET_H UINT8_C(0x17)
#define ZG_OFFSET_L UINT8_C(0x18)

//Sample Rate Divider
#define SMPLRT_DIV UINT8_C(0x17)

// Configuration
#define CONFIG UINT8_C(0x1A)
#define GYRO_CONFIG UINT8_C(0x1B)
#define ACCEL_CONFIG UINT8_C(0x1C)
#define ACCEL_CONFIG2 UINT8_C(0x1D)
#define LP_ACCEL_ODR UINT8_C(0x1E)
#define WOM_THR UINT8_C(0x1E)

#define FIFO_EN UINT8_C(0x23)

#define I2C_MST_CTRL UINT8_C(0x24) 


//Slave 0
#define I2C_SLV0_ADDR UINT8_C(0x25) 
#define I2C_SLV0_REG UINT8_C(0x26) 
#define I2C_SLV0_CTRL UINT8_C(0x27)


//Slave 1
#define I2C_SLV1_ADDR UINT8_C(0x28) 
#define I2C_SLV1_REG UINT8_C(0x29) 
#define I2C_SLV1_CTRL UINT8_C(0x2A)


//Slave 2
#define I2C_SLV2_ADDR UINT8_C(0x2B) 
#define I2C_SLV2_REG UINT8_C(0x2C) 
#define I2C_SLV2_CTRL UINT8_C(0x2D)


//Slave 3
#define I2C_SLV3_ADDR UINT8_C(0x2E) 
#define I2C_SLV3_REG UINT8_C(0x2F) 
#define I2C_SLV3_CTRL UINT8_C(0x30)

//Slave 4
#define I2C_SLV4_ADDR UINT8_C(0x31) 
#define I2C_SLV4_REG UINT8_C(0x32) 
#define I2C_SLV4_D0 UINT8_C(0x33) 
#define I2C_SLV4_CTRL UINT8_C(0x34)
#define I2C_SLV4_DI UINT8_C(0x35)

//I2C master status
#define I2C_MST_STATUS UINT8_C(0x36)

//Interrupciones
#define INT_PIN_CFG UINT8_C(0x37)
#define INT_ENABLE UINT8_C(0x38)
#define INT_STATUS UINT8_C(0x3A)


// Mediciones del acelerometro
#define ACCEL_XOUT_H UINT8_C(0x3B)
#define ACCEL_XOUT_L UINT8_C(0x3B)
#define ACCEL_XOUT_H UINT8_C(0x3B)
#define ACCEL_XOUT_L UINT8_C(0x3B)
#define ACCEL_XOUT_H UINT8_C(0x3B)
#define ACCEL_XOUT_L UINT8_C(0x3B)










#endif