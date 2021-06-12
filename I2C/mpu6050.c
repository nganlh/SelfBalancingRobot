/*
 * mpu6050.c
 *
 *  Created on: 1 thg 4, 2021
 *      Author: huungan
 */

#include <I2C/mpu6050.h>

extern int16_t accaxisX, accaxisY, accaxisZ, gyroaxisX, gyroaxisY, gyroaxisZ;

void initI2C(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1); // Enable I2C1 peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable GPIOA peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    // Use alternate function
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6); // Use pin with I2C SCL peripheral
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7); // Use pin with I2C peripheral

    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true); // Enable and set frequency to 400 kHz

    SysCtlDelay(2); // Insert a few cycles after enabling the I2C to allow the clock to be fully activated
}

void i2cWrite(uint8_t addr, uint8_t regAddr, uint8_t data) {
    i2cWriteData(addr, regAddr, &data, 1);
}

void i2cWriteData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length) {
    I2CMasterSlaveAddrSet(I2C1_BASE, addr, false); // Set to write mode

    I2CMasterDataPut(I2C1_BASE, regAddr); // Place address into data register
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send start condition
    while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
    uint8_t i = 0;
    for (i = 0; i < length - 1; i++) {
        I2CMasterDataPut(I2C1_BASE, data[i]); // Place data into data register
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT); // Send continues condition
        while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
    }

    I2CMasterDataPut(I2C1_BASE, data[length - 1]); // Place data into data register
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); // Send finish condition
    while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
}

uint8_t i2cRead(uint8_t addr, uint8_t regAddr) {
    I2CMasterSlaveAddrSet(I2C1_BASE, addr, false); // Set to write mode

    I2CMasterDataPut(I2C1_BASE, regAddr); // Place address into data register
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND); // Send data
    while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done

    I2CMasterSlaveAddrSet(I2C1_BASE, addr, true); // Set to read mode

    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); // Tell master to read data
    while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
    return I2CMasterDataGet(I2C1_BASE); // Read data
}

void i2cReadData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length) {
    I2CMasterSlaveAddrSet(I2C1_BASE, addr, false); // Set to write mode

    I2CMasterDataPut(I2C1_BASE, regAddr); // Place address into data register
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND); // Send data
    while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done

    I2CMasterSlaveAddrSet(I2C1_BASE, addr, true); // Set to read mode

    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); // Send start condition
    while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
    data[0] = I2CMasterDataGet(I2C1_BASE); // Place data into data register
    uint8_t i = 1;
    for (i = 1; i < length - 1; i++) {
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT); // Send continues condition
        while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
        data[i] = I2CMasterDataGet(I2C1_BASE); // Place data into data register
    }

    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH); // Send finish condition
    while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
    data[length - 1] = I2CMasterDataGet(I2C1_BASE); // Place data into data register
}
void initMPU6050(void) {
    uint8_t i2cBuffer[5]; // Buffer for I2C data
    i2cBuffer[0] = i2cRead(MPU6050_ADDRESS, MPU6050_WHO_AM_I);
    i2cWrite(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, (1 << 7)); // Reset device, this resets all internal registers to their default values
    SysCtlDelay(SysCtlClockGet()/100);
    while (i2cRead(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1) & (1 << 7)) {
    // Wait for the bit to clear
    };
    SysCtlDelay(SysCtlClockGet()/100);
    i2cWrite(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, (1 << 3) | (1 << 0)); // Disable sleep mode, disable temperature sensor and use PLL as clock reference

    i2cBuffer[0] = 0; // Set the sample rate to 1kHz - 1kHz/(1+0) = 1kHz
    i2cBuffer[1] = 0x03; // Disable FSYNC and set 41 Hz Gyro filtering, 1 KHz sampling
    i2cBuffer[2] = 3 << 3; // Set Gyro Full Scale Range to +-2000deg/s
    i2cBuffer[3] = 2 << 3; // Set Accelerometer Full Scale Range to +-8g
    i2cBuffer[4] = 0x03; // 41 Hz Acc filtering
    i2cWriteData(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, i2cBuffer, 5); // Write to all five registers at once


    /* Enable Raw Data Ready Interrupt on INT pin */
    i2cBuffer[0] = (1 << 5) | (1 << 4); // Enable LATCH_INT_EN and INT_ANYRD_2CLEAR
    // When this bit is equal to 1, the INT pin is held high until the interrupt is cleared
    // When this bit is equal to 1, interrupt status is cleared if any read operation is performed
    i2cBuffer[1] = (1 << 0);            // Enable RAW_RDY_EN - When set to 1, Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin
    i2cWriteData(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG, i2cBuffer, 2); // Write to both registers at once

}
void getMPU6050Data(void) {
    uint8_t buf[14];
    i2cReadData(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, buf, 14); // Note that we can't write directly into MPU6050_t, because of endian conflict. So it has to be done manually

    accaxisX = (buf[0] << 8) | buf[1];
    accaxisY = (buf[2] << 8) | buf[3];
    accaxisZ = (buf[4] << 8) | buf[5];

    gyroaxisX = (buf[8] << 8) | buf[9];
    gyroaxisY = (buf[10] << 8) | buf[11];
    gyroaxisZ = (buf[12] << 8) | buf[13];
}

float Comp_Filter(float *p_gyroValue, float *p_accelValue)
{
    // Reset gyroAngle if it's value drift too much
//    static int driftingCounter = 0;
//    if(abs(*p_gyroValue-*p_accelValue) > MAX_DRIFTING_ANGLE)  driftingCounter++;
//    else    driftingCounter = 0;
//    if(driftingCounter > MAX_DRIFTING_COUNTER)  *p_gyroValue = *p_accelValue;

    // Filter
    float alpha = 0.97;
    return (float)(*p_gyroValue)*alpha + (float)(*p_accelValue)*(1-alpha);
}
