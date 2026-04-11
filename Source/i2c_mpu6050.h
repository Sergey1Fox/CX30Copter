/**
  ******************************************************************************
  * @file    i2c_mpu6050.h
  * @brief   I2C driver for MPU6050 IMU
  *          I2C clock 200KHz
  ******************************************************************************
  */

#ifndef __I2C_MPU6050_H
#define __I2C_MPU6050_H

#include "stm8s.h"

/* MPU6050 I2C Address */
#define MPU6050_ADDR        0x68

/* MPU6050 Register Addresses */
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_INT_ENABLE      0x38
#define MPU_REG_ACCEL_XOUT_H    0x3B
#define MPU_REG_ACCEL_XOUT_L    0x3C
#define MPU_REG_ACCEL_YOUT_H    0x3D
#define MPU_REG_ACCEL_YOUT_L    0x3E
#define MPU_REG_ACCEL_ZOUT_H    0x3F
#define MPU_REG_ACCEL_ZOUT_L    0x40
#define MPU_REG_TEMP_OUT_H      0x41
#define MPU_REG_TEMP_OUT_L      0x42
#define MPU_REG_GYRO_XOUT_H     0x43
#define MPU_REG_GYRO_XOUT_L     0x44
#define MPU_REG_GYRO_YOUT_H     0x45
#define MPU_REG_GYRO_YOUT_L     0x46
#define MPU_REG_GYRO_ZOUT_H     0x47
#define MPU_REG_GYRO_ZOUT_L     0x48
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_WHO_AM_I        0x75

/* MPU6050 Data Structure */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050_Data_t;

/* External state */
extern volatile uint8_t mpu_data_ready;
extern MPU6050_Data_t mpu_data;

void I2C_MPU_Init(void);
uint8_t I2C_MPU_ReadReg(uint8_t reg);
void I2C_MPU_WriteReg(uint8_t reg, uint8_t value);
uint8_t I2C_MPU_ReadMulti(uint8_t reg, uint8_t *data, uint8_t len);
uint8_t MPU6050_ReadAllData(void);

#endif /* __I2C_MPU6050_H */
