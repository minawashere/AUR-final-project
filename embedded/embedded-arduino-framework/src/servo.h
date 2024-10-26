#ifndef SERVO_H
#define SERVO_H

#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "driver/timer.h"

class Servo
{
public:
    Servo(int pin);
    void write(float speed); // Public function to control servo
    void setServoPin();      // Make this public if you want to call it externally

private:
    int servoPin;
    float calcPulseWidth(float speed);
    void setupTimer();
    void setTimer(float pulseWidth);
    static void IRAM_ATTR onTimer(void *arg);
};

#endif
