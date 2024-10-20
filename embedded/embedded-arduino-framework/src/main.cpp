#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU6050_ADDR 0x68 // AD0 is connected to GND

typedef struct
{
    int16_t accData[3];
    int16_t gyrData[3];
} imuData;

QueueHandle_t imuQueue;

void init_i2c()
{
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // Fast mode
}

void I2C_WriteByte(uint8_t address, uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void init_IMU()
{
    I2C_WriteByte(MPU6050_ADDR, 0x6B, 0x00); // Wake up the MPU6050
}

void configure_IMU()
{
    I2C_WriteByte(MPU6050_ADDR, 0x1C, 0x00); // Set accelerometer range
    I2C_WriteByte(MPU6050_ADDR, 0x1B, 0x00); // Set gyroscope range
}

void readData(int16_t *accData, int16_t *gyrData)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // Start reading from ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14); // Read 14 bytes total

    if (Wire.available() == 14)
    {
        for (int i = 0; i < 3; i++)
        {
            accData[i] = (Wire.read() << 8) | Wire.read(); // Accelerometer data
            gyrData[i] = (Wire.read() << 8) | Wire.read(); // Gyroscope data
        }
        // Skip temperature reading (2 bytes)
        Wire.read(); // Temperature high
        Wire.read(); // Temperature low
    }
    else
    {
        Serial.println("Error: Communication failed with MPU6050.");
    }
}

void convertData(int16_t *accData, int16_t *gyrData, float *convAccData, float *convGyrData)
{
    float accFactor = 16384.0; // 2g range
    float gyrFactor = 131.0;   // 250°/s range

    for (int i = 0; i < 3; i++)
    {
        convAccData[i] = accData[i] / accFactor;
        convGyrData[i] = gyrData[i] / gyrFactor;
    }
}

void Task1_readData(void *pvParameters)
{
    imuData data;
    while (1)
    {
        readData(data.accData, data.gyrData);
        if (xQueueSend(imuQueue, &data, portMAX_DELAY) != pdTRUE)
        {
            Serial.println("Error: Failed to send the data to the queue");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 ms
    }
}

void Task2_convertData(void *pvParameters)
{
    imuData data;
    float convAccData[3], convGyrData[3];
    while (1)
    {
        if (xQueueReceive(imuQueue, &data, portMAX_DELAY) == pdTRUE)
        {
            convertData(data.accData, data.gyrData, convAccData, convGyrData);
            // Use the converted data as needed
            Serial.println(convGyrData[0]); // Example output
        }
    }
}

void setup()
{
    Serial.begin(115200);
    init_i2c();
    init_IMU();
    configure_IMU();

    imuQueue = xQueueCreate(20, sizeof(imuData));
    if (imuQueue == NULL)
    {
        Serial.println("Error initializing queue");
    }

    xTaskCreate(Task1_readData, "Task1_readData", 2048, NULL, 1, NULL);
    xTaskCreate(Task2_convertData, "Task2_convertData", 2048, NULL, 1, NULL);
}

void loop()
{
    // No need to do anything in the loop; tasks handle the work
}
