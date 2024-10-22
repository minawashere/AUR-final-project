
#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>

class ServoDriver
{
public:
    ServoDriver(int pin);
    void write(float angle);

private:
    int servoPin;
    void setServoPin();
    float calcPulseWidth(float angle);
};

#endif
