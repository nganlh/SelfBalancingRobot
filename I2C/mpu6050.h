/*
 * mpu6050.h
 *
 *  Created on: 1 thg 4, 2021
 *      Author: huungan
 */

#ifndef I2C_MPU6050_H_
#define I2C_MPU6050_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"

#define MPU6050_SMPLRT_DIV                  0x19
#define MPU6050_INT_PIN_CFG                 0x37
#define MPU6050_ACCEL_XOUT_H                0x3B
#define MPU6050_GYRO_XOUT_H                 0x43
#define MPU6050_PWR_MGMT_1                  0x6B
#define MPU6050_WHO_AM_I                    0x75
#define MPU6050_ADDRESS                     0x68
#define MPU6050_WHO_AM_I_ID                 0x68

// Scale factor for +-2000deg/s and +-8g - see datasheet:
#define MPU6050_GYRO_SCALE_FACTOR_2000      16.384f
#define MPU6050_ACC_SCALE_FACTOR_8          4096.0f

// Gyro angle drifting values for reseting gyroAngle
#define MAX_DRIFTING_ANGLE                  8
#define MAX_DRIFTING_COUNTER                5000

void initI2C(void);
void i2cWrite(uint8_t addr, uint8_t regAddr, uint8_t data);
void i2cWriteData(uint8_t addr, uint8_t regAddr, uint8_t *date, uint8_t length);
uint8_t i2cRead(uint8_t addr, uint8_t regAddr);
void i2cReadData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length);
void initMPU6050(void);
void getMPU6050Data(void);
float Comp_Filter(float *p_gyroValue, float *p_accelValue);

#ifdef __cplusplus
}
#endif


#endif /* I2C_MPU6050_H_ */
