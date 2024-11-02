
#include "servo.h"
#include "Arduino.h"
#define map(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

void Servo::write(const double angle) const {
    ledcWrite(lecdChannel, map(angle, 0, 180, 205, 410));
}
