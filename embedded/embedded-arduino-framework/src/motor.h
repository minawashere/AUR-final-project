#pragma once
#include "Arduino.h"

/*use ledc instead of analogWrite()*/

enum Direction { FORWARDS = 1, BACKWARDS = -1 };

struct Motor {
private:
    unsigned long prev_count = 0;
    unsigned long prev_time = 0;
    uint8_t pin1;
    uint8_t pin2;
    uint8_t pwm_pin;

public:
    Motor(const uint8_t pin1, const uint8_t pin2, const uint8_t pwm_pin) : pin1(pin1), pin2(pin2), pwm_pin(pwm_pin) {
        pinMode(pwm_pin, OUTPUT);
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
    }

    void drive(Direction direction, uint16_t speed);
};
