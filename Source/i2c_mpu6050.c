/**
  ******************************************************************************
  * @file    i2c_mpu6050.c
  * @brief   I2C driver for MPU6050 IMU
  ******************************************************************************
  */

#include "i2c_mpu6050.h"

/* MPU6050 initialization sequence from MPU_Init.txt */
typedef struct {
    uint8_t reg;
    uint8_t value;
} MPU_InitStep_t;

/* Parsed initialization sequence from MPU_Init.txt */
static const MPU_InitStep_t mpu_init_sequence[] = {
    /* Step 9-21: Read WHO_AM_I (0x75) - expect 0x68 */
    /* This is a read, handled separately */
    
    /* Step 24-36: Read CONFIG (0x1A) - expect 0x00 */
    /* This is a read, handled separately */
    
    /* Step 39-47: Write PWR_MGMT_1 (0x6B) = 0x80 (reset) */
    {0x6B, 0x80},
    
    /* Step 51-58: Write PWR_MGMT_1 (0x6B) = 0x03 (PLL with Z gyro) */
    {0x6B, 0x03},
    
    /* Step 62-69: Write SMPLRT_DIV (0x19) = 0x01 (sample rate divider) */
    {0x19, 0x01},
    
    /* Step 73-80: Write CONFIG (0x1A) = 0x03 (DLPF 42Hz) */
    {0x1A, 0x03},
    
    /* Step 84-91: Write GYRO_CONFIG (0x1B) = 0x18 (+/-2000 deg/s) */
    {0x1B, 0x18},
    
    /* Step 95-102: Write ACCEL_CONFIG (0x1C) = 0x10 (+/-8g) */
    {0x1C, 0x10},
    
    /* Step 106-113: Write INT_ENABLE (0x38) = 0x00 */
    {0x38, 0x00},
    
    /* Step 117-124: Write ?? = 0x10 */
    {0x37, 0x10},  /* INT_PIN_CFG */
};

#define MPU_INIT_STEPS (sizeof(mpu_init_sequence) / sizeof(mpu_init_sequence[0]))

/* External state */
volatile uint8_t mpu_data_ready = 0;
MPU6050_Data_t mpu_data;

/* I2C timeout */
#define I2C_TIMEOUT 10000

void I2C_MPU_Init(void)
{
    uint16_t i;
    uint16_t timeout;
    
    /* Enable I2C clock */
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);
    
    /* I2C Init: 200KHz */
    I2C_DeInit();
    I2C_Init(400000,                    /* 400KHz */
             MPU6050_ADDR,              /* Own address (not used as master) */
             I2C_DUTYCYCLE_2,           /* Duty cycle */
             I2C_ACK_CURR,              /* Acknowledge */
             I2C_ADDMODE_7BIT,          /* 7-bit addressing */
             (CLK_GetClockFreq() / 1000000)); /* Input clock in MHz */
    I2C_Cmd(ENABLE);
    
    /* Send initialization sequence */
    for (i = 0; i < MPU_INIT_STEPS; i++) {
        /* Wait until busy flag is cleared */
        timeout = I2C_TIMEOUT;
        while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY) && timeout > 0) {
            timeout--;
        }
        
        /* Generate START */
        I2C_GenerateSTART(ENABLE);
        timeout = I2C_TIMEOUT;
        while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT) && timeout > 0) {
            timeout--;
        }
        
        /* Send MPU address with write */
        I2C_Send7bitAddress(MPU6050_ADDR, I2C_DIRECTION_TX);
        timeout = I2C_TIMEOUT;
        while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout > 0) {
            timeout--;
        }
        
        /* Send register address */
        I2C_SendData(mpu_init_sequence[i].reg);
        timeout = I2C_TIMEOUT;
        while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout > 0) {
            timeout--;
        }
        
        /* Send data */
        I2C_SendData(mpu_init_sequence[i].value);
        timeout = I2C_TIMEOUT;
        while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout > 0) {
            timeout--;
        }
        
        /* Generate STOP */
        I2C_GenerateSTOP(ENABLE);
        
        /* Small delay between writes */
        {
            volatile uint16_t delay;
            for (delay = 0; delay < 5000; delay++);
        }
    }
    
    /* Verify WHO_AM_I */
    {
        uint8_t who_am_i = I2C_MPU_ReadReg(MPU_REG_WHO_AM_I);
        (void)who_am_i;  /* Should be 0x68 */
    }
}

uint8_t I2C_MPU_ReadReg(uint8_t reg)
{
    uint8_t data = 0;
    uint16_t timeout;
    
    /* Wait until bus is free */
    timeout = I2C_TIMEOUT;
    while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY) && timeout > 0) {
        timeout--;
    }
    
    /* Generate START */
    I2C_GenerateSTART(ENABLE);
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT) && timeout > 0) {
        timeout--;
    }
    
    /* Send MPU address with write */
    I2C_Send7bitAddress(MPU6050_ADDR, I2C_DIRECTION_TX);
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout > 0) {
        timeout--;
    }
    
    /* Send register address */
    I2C_SendData(reg);
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout > 0) {
        timeout--;
    }
    
    /* Generate repeated START */
    I2C_GenerateSTART(ENABLE);
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT) && timeout > 0) {
        timeout--;
    }
    
    /* Send MPU address with read */
    I2C_Send7bitAddress(MPU6050_ADDR, I2C_DIRECTION_RX);
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && timeout > 0) {
        timeout--;
    }
    
    /* Disable ACK before reading last byte */
    I2C_AcknowledgeConfig(I2C_ACK_NONE);
    
    /* Generate STOP */
    I2C_GenerateSTOP(ENABLE);
    
    /* Wait for data */
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) && timeout > 0) {
        timeout--;
    }
    
    /* Read data */
    data = I2C_ReceiveData();
    
    /* Re-enable ACK for next read */
    I2C_AcknowledgeConfig(I2C_ACK_CURR);
    
    return data;
}

void I2C_MPU_WriteReg(uint8_t reg, uint8_t value)
{
    uint16_t timeout;
    
    /* Wait until bus is free */
    timeout = I2C_TIMEOUT;
    while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY) && timeout > 0) {
        timeout--;
    }
    
    /* Generate START */
    I2C_GenerateSTART(ENABLE);
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT) && timeout > 0) {
        timeout--;
    }
    
    /* Send MPU address with write */
    I2C_Send7bitAddress(MPU6050_ADDR, I2C_DIRECTION_TX);
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout > 0) {
        timeout--;
    }
    
    /* Send register address */
    I2C_SendData(reg);
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout > 0) {
        timeout--;
    }
    
    /* Send data */
    I2C_SendData(value);
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout > 0) {
        timeout--;
    }
    
    /* Generate STOP */
    I2C_GenerateSTOP(ENABLE);
}

uint8_t I2C_MPU_ReadMulti(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint16_t timeout;
    
    /* Wait until bus is free */
    timeout = I2C_TIMEOUT;
    while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY) && timeout > 0) {
        timeout--;
    }
    
    /* Generate START */
    I2C_GenerateSTART(ENABLE);
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT) && timeout > 0) {
        timeout--;
    }
    
    /* Send MPU address with write */
    I2C_Send7bitAddress(MPU6050_ADDR, I2C_DIRECTION_TX);
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout > 0) {
        timeout--;
    }
    
    /* Send register address */
    I2C_SendData(reg);
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout > 0) {
        timeout--;
    }
    
    /* Generate repeated START */
    I2C_GenerateSTART(ENABLE);
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT) && timeout > 0) {
        timeout--;
    }
    
    /* Send MPU address with read */
    I2C_Send7bitAddress(MPU6050_ADDR, I2C_DIRECTION_RX);
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && timeout > 0) {
        timeout--;
    }
    
    /* Read bytes */
    for (i = 0; i < len; i++) {
        timeout = I2C_TIMEOUT;
        while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) && timeout > 0) {
            timeout--;
        }
        
        if (i == (len - 1)) {
            /* Last byte: disable ACK and generate STOP */
            I2C_AcknowledgeConfig(I2C_ACK_NONE);
            I2C_GenerateSTOP(ENABLE);
        }
        
        data[i] = I2C_ReceiveData();
    }
    
    /* Re-enable ACK */
    I2C_AcknowledgeConfig(I2C_ACK_CURR);
    
    return (timeout > 0) ? 1 : 0;
}

uint8_t MPU6050_ReadAllData(void)
{
    uint8_t data[14];
    
    /* Read 14 bytes starting from ACCEL_XOUT_H */
    if (I2C_MPU_ReadMulti(MPU_REG_ACCEL_XOUT_H, data, 14)) {
        mpu_data.accel_x = (int16_t)((data[0] << 8) | data[1]);
        mpu_data.accel_y = (int16_t)((data[2] << 8) | data[3]);
        mpu_data.accel_z = (int16_t)((data[4] << 8) | data[5]);
        mpu_data.temp = (int16_t)((data[6] << 8) | data[7]);
        mpu_data.gyro_x = (int16_t)((data[8] << 8) | data[9]);
        mpu_data.gyro_y = (int16_t)((data[10] << 8) | data[11]);
        mpu_data.gyro_z = (int16_t)((data[12] << 8) | data[13]);
        
        mpu_data_ready = 1;
        return 1;
    }
    
    mpu_data_ready = 0;
    return 0;
}
