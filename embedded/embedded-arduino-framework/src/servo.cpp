
#include "ServoDriver.h"

ServoDriver::ServoDriver(int pin) : servoPin(pin)
{
    setServoPin();
}

void ServoDriver::setServoPin()
{
    gpio_pad_select_gpio(servoPin);
    gpio_set_direction(static_cast<gpio_num_t>(servoPin), GPIO_MODE_OUTPUT);
}

float ServoDriver::calcPulseWidth(float angle)
{
    float pulseWidth = 1000 + (angle * 1000 / 180);
    return pulseWidth;
}

void ServoDriver::write(float angle)
{
    GPIO.out_w1ts = (1 << servoPin); // Set pin high
    delayMicroseconds(static_cast<int>(calcPulseWidth(angle)));
    GPIO.out_w1tc = (1 << servoPin); // Set pin low
}
