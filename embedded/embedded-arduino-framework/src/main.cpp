#include <Arduino.h>
#include <imu.h> // Your custom IMU library

IMU imu;

void setup() {
    Serial.begin(115200);
    delay(200);
    i2c_init();
    IMU_init();
    Serial.println("Hello World!");
}



void loop() {
    fetchIMU(imu.accData, imu.gyroData);

    Serial.print("Accx ");
    Serial.print(imu.accData[0]);
    Serial.print(" Accy ");
    Serial.print(imu.accData[1]);
    Serial.print(" Accz ");
    Serial.print(imu.accData[2]);
    Serial.print(" Gyx ");
    Serial.print(imu.gyroData[0]);

    Serial.print(" GyY");
    Serial.print(imu.gyroData[1]);

    Serial.print(" GyZ");
    Serial.print(imu.gyroData[2]);


    Serial.println("");

    delay(20);
}