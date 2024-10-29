#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    uint8_t pinA, pinB;
    Encoder(const uint8_t pinA, const uint8_t pinB);
    int32_t position();
    boolint8_t direction();
    uint32_t pulses();


    double rpm();




private:
    uint32_t prev_pulses;
    unsigned long prev_time = micros();
    double m_rpm = 0;
};

#endif
