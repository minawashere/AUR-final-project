
#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>

class Servo
{
public:
    explicit Servo(int pin);
    void write(float speed);

private:
    int servoPin;
    void setServoPin();
    float calcPulseWidth(float speed);
};

#endif
