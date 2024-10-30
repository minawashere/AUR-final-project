#ifndef IMU_H
#define IMU_H
#include <Arduino.h>
#include <Wire.h>


struct IMU {
    IMU(const uint8_t sda_pin, const uint8_t scl_pin);
    float base_GyZ = 0;
    int16_t accData[3];
    int16_t gyroData[3];

    void fetchIMU();

    // void IMU_init();
};


#endif //IMU_H
