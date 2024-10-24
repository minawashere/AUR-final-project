
#include "servo.h"

Servo::Servo(int pin) : servoPin(pin)
{
    setServoPin();
}

void Servo::setServoPin()
{
    gpio_pad_select_gpio(servoPin);
    gpio_set_direction(static_cast<gpio_num_t>(servoPin), GPIO_MODE_OUTPUT);
}

float Servo::calcPulseWidth(float speed)
{
    float pulseWidth = 1500 + (speed * 500);
    return pulseWidth;
}

void Servo::write(float speed)
{
    GPIO.out_w1ts = (1 << servoPin); // Set pin high
    delayMicroseconds(static_cast<int>(calcPulseWidth(angle)));
    GPIO.out_w1tc = (1 << servoPin); // Set pin low
}
