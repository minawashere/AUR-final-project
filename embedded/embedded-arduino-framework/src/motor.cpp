#include "motor.h"
#include "Arduino.h"


void Motor::drive(const Direction direction, const uint16_t speed){    const int l = (direction == FORWARDS) ? HIGH : LOW;
    digitalWrite(pin1, l);
    digitalWrite(pin2, !l);
    analogWrite(pwm_pin, speed);
}