

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

struct IMU {
    IMU(){}
    int16_t accData[3];
    int16_t gyroData[3];
};

void imu_init();
void i2c_init();
void I2C_writeByte(uint8_t address, uint8_t reg, uint8_t data);
void init_IMU();
void configure_IMU();
void fetchIMU(int16_t *accData, int16_t *gyrData);
void convert_IMU();

#endif //IMU_H
