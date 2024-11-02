#include "motor.h"
#include "Arduino.h"
#define map(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)



void Motor::drive(const Direction direction, uint16_t speed){
    const int l = (direction == FORWARDS) ? HIGH : LOW;
    speed = map(speed, 0, 110, 0, 255);
    digitalWrite(pin1, l);
    digitalWrite(pin2, !l);
    analogWrite(pwm_pin, speed);
}