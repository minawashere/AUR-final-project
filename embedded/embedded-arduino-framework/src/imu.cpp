#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU6050_ADDR 0x68 //(AD0 IS CONNECTED TO THE GROUND)

typedef struct
{
    int16_t accData[3];
    int16_t gyrData[3];
} imuData;

QueueHandle_t imuQueue;

void i2c_init()
{
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // fast mode
    Serial.println("I2C init");
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
    I2C_WriteByte(MPU6050_ADDR, 0x6B, 0x00);
}

void configure_IMU()
{
    I2C_WriteByte(MPU6050_ADDR, 0x1C, 0x00);
    I2C_WriteByte(MPU6050_ADDR, 0x1B, 0x00);
}

void read_IMU(int16_t *accData, int16_t *gyrData)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    if (Wire.endTransmission() != 0)
    {
        Serial.println("Error: Communication failed with MPU6050.");
        return;
    }
    Wire.requestFrom(MPU6050_ADDR, 6);
    for (int i = 0; i < 3; i++)
    {
        accData[i] = (Wire.read() << 8) | Wire.read();
    }
    Wire.write(0x43);
    Wire.requestFrom(MPU6050_ADDR, 6);
    for (int i = 0; i < 3; i++)
    {
        gyrData[i] = (Wire.read() << 8) | Wire.read();
    }
}

void convertData(int16_t *accData, int16_t *gyrData, float *convAccData, float *conGyrData)
{
    float accFactor = 16384.0;
    float gyrFactor = 131.0;
    for (int i = 0; i < 3; i++)
    {
        convAccData[i] = accData[i] / accFactor;
        conGyrData[i] = gyrData[i] / gyrFactor;
    }
}

while(1) {
    imuData data;
    float convAccData[3],convGyrData[3];
    read_IMU(data.accData,data.gyrData);

}