#include <Arduino.h>
#include <imu.h> // Your custom IMU library

IMU imu;

void setup() {
    Serial.begin(115200);
    delay(200);
    i2c_init();
}



void loop() {
    fetchIMU(imu.accData, imu.gyroData);

    Serial.print("Gyx ");
    Serial.print(imu.accData[0]);
    Serial.print(" Gyy ");
    Serial.print(imu.accData[1]);
    Serial.print(" Gyz ");
    Serial.print(imu.accData[2]);
    Serial.print(" Gyx ");
    Serial.print(imu.gyroData[0]);

    Serial.print(" accx");
    Serial.print(imu.gyroData[1]);

    Serial.print(" accy");
    Serial.print(imu.gyroData[2]);


    Serial.println("");

    delay(20);
}