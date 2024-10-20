#include "IRsensor.h"
#include <Arduino.h>


IRsensor::IRsensor(int pin) : pin(pin) {
    pinMode(pin, INPUT);
}

bool IRsensor::read() {
    return (bool)digitalRead(pin);
}
