

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

void imu_init();
void init_i2c();
void I2C_writeByte(uint8_t address, uint8_t reg, uint8_t data);
void init_IMU();
void configure_IMU();
void read_IMU();
void convert_IMU();

#endif //IMU_H
