#include "IRsensor.h"
#include <Arduino.h>


IRsensor::IRsensor(uint8_t pin) : pin(pin) {
    pinMode(pin, INPUT);
}

bool IRsensor::read() {
    return (bool)digitalRead(pin);
}
