#ifndef IRSENSOR_H
#define IRSENSOR_H
#include <Arduino.h>


class IRsensor {
    uint8_t pin;
public:
    IRsensor(uint8_t pin);
    bool read();
};


#endif //IRSENSOR_H
