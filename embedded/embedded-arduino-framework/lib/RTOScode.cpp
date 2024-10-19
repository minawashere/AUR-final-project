#include <Arduino.h>
// #include <Arduino_FreeRTOS.h>

#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU6050_ADDR 0x68 //(AD0 IS CONNECTED TO THE GROUND)

typedef struct
{
    int16_t accData[3];
    int16_t gyrData[3];
} imuData;

QueueHandle_t imuQueue;

void init_i2c()
{
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // fast mode
}

void I2C_WriteByte(uint8_t address, uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(address); // Start communication with device at given address
    Wire.write(reg);                 // Specify register
    Wire.write(data);                // Write data to the register
    Wire.endTransmission();          // End transmission
}

void init_IMU()
{
    I2C_WriteByte(MPU6050_ADDR, 0x6B, 0x00);
}

void configure_IMU()
{
    I2C_WriteByte(MPU6050_ADDR, 0x1C, 0x00);
    I2C_WriteByte(MPU6050_ADDR, 0x1B, 0x00);
}

void readData(int16_t *accData, int16_t *gyrData)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    if (Wire.endTransmission() != 0)
    {
        Serial.println("Error: Communication failed with MPU6050.");
        return;
    }
    Wire.requestFrom(MPU6050_ADDR, 12);
    for (int i = 0; i < 3; i++)
    {
        accData[i] = (Wire.read() << 8) | Wire.read();
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

void Task1_readData(void *pvParameters)
{
    imuData data;
    while (1)
    {
        readData(data.accData, data.gyrData);
        if (xQueueSend(imuQueue, &data, portMAX_DELAY) != pdTRUE)
        {
            Serial.println("Error:failed to send the data to the queue");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void Task2_convertData(void *pvParameters)
{
    imuData data;
    float convAccData[3], convGyrData[3];
    while (1)
    {
        if (xQueueReceive(imuQueue, &data, portMAX_DELAY) == pdTRUE)
            convertData(data.accData, data.gyrData, convAccData, convGyrData);
        // Serial.println(convAccData[0]);
        Serial.println(convGyrData[0]);

        // use the converted data
    }
}

void setup()
{
    Serial.begin(9600);

    init_i2c();
    init_IMU();
    configure_IMU();

    imuQueue = xQueueCreate(20, sizeof(imuData));
    if (imuQueue == NULL)
    {
        Serial.print("Error in initalizing queue");
    }
    xTaskCreate(Task1_readData, "Task1_readData", 2048, NULL, 1, NULL);
    xTaskCreate(Task2_convertData, "Task2_convertData", 2048, NULL, 1, NULL);
}

void loop()
{
    // put your main code here, to run repeatedly:
}
