#pragma once
#include <Arduino.h>


class Servo {
public:
    Servo(const uint8_t pin, const uint8_t lecdChannel): pin(pin), lecdChannel(lecdChannel) {
        pinMode(pin, OUTPUT);
        ledcSetup(lecdChannel, 50, 12);
        ledcAttachPin(pin, lecdChannel);
    }

    void write(double angle) const;

private:
    uint8_t pin;
    uint8_t lecdChannel;
};


