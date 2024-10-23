#include <Arduino.h>
#include <Wire.h>


#define SDA_PIN 21
#define SCL_PIN 22
#define MPU6050_ADDR 0x68 //(AD0 IS CONNECTED TO THE GROUND)
#define ACC_FACTOR 16384.0
#define GYRO_FACTOR 131.0

struct IMU{
    IMU(){}
    int16_t accData[3];
    int16_t gyroData[3];
};


void i2c_init()
{
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // fast mode
}

void I2C_WriteByte(uint8_t address, uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void imu_init()
{
    I2C_WriteByte(MPU6050_ADDR, 0x6B, 0x00); //power management register
}

void configure_IMU()
{
    I2C_WriteByte(MPU6050_ADDR, 0x1C, 0x00); //set acc 2g
    I2C_WriteByte(MPU6050_ADDR, 0x1B, 0x00); //set gy250 rev/s
}

void fetchIMU(int16_t *accData, int16_t *gyrData)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); //starting register
    Wire.endTransmission(false);
    if (Wire.endTransmission() != 0)
    {
        Serial.println("Error: Communication failed with MPU6050.");
        return;
    }
    Wire.requestFrom(MPU6050_ADDR, 12);
    for (int i = 0; i < 3; i++)
    {
        accData[i] = ((Wire.read() << 8) | Wire.read())/ ACC_FACTOR;
        gyrData[i] = ((Wire.read() << 8) | Wire.read()) / GYRO_FACTOR;
    }
}

// void convertData(int16_t *accData, int16_t *gyrData, float *convAccData, float *conGyrData)
// {
//     for (int i = 0; i < 3; i++)
//     {
//         convAccData[i] = accData[i] / ACC_FACTOR;
//         conGyrData[i] = gyrData[i] / GYRO_FACTOR;
//     }
// }