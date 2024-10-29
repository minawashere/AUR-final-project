#include <Arduino.h>
#include <Wire.h>
#include "imu.h"


#define MPU6050_ADDR 0x68
#define ACC_FACTOR 16384.0
#define GYRO_FACTOR 131.0
#define G_TO_MS2 9.81


// void i2c_init() {
//   Wire.begin(SDA_PIN, SCL_PIN);
//   Wire.setClock(400000);  // Fast mode I2C
// }

void I2C_WriteByte(uint8_t address, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// void configure_IMU() {
//   I2C_WriteByte(MPU6050_ADDR, 0x1C, 0x00);  // Set accelerometer to ±2g
//   I2C_WriteByte(MPU6050_ADDR, 0x1B, 0x00);  // Set gyroscope to ±250°/s
// }

void IMU::fetchIMU() {
  // Read accelerometer data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_ADDR, 6);
  for (int i = 0; i < 3; i++) {
    int16_t rawAcc = (Wire.read() << 8) | Wire.read();
    accData[i] = (rawAcc / ACC_FACTOR) * G_TO_MS2;
  }
  // Read gyroscope data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43); // Starting register for gyroscope data
  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_ADDR, 6);
  for (int i = 0; i < 3; i++) {
    int16_t rawGyr = (Wire.read() << 8) | Wire.read();
    gyroData[i] = rawGyr / GYRO_FACTOR;
  }
}

void IMU::IMU_init(const uint8_t SDA_PIN, const uint8_t SCL_PIN) {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // Fast mode I2C
  I2C_WriteByte(MPU6050_ADDR, 0x6B, 0x00); // Power management register
  I2C_WriteByte(MPU6050_ADDR, 0x1C, 0x00); // Set accelerometer to ±2g
  I2C_WriteByte(MPU6050_ADDR, 0x1B, 0x00); // Set gyroscope to ±250°/s
  for (int i = 0; i < 1000; i++) {
    fetchIMU();
    base_GyZ += accData[2];
    delay(5);
  }
  base_GyZ /= -1000;
}


