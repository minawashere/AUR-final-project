#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    Encoder(int pinA, int pinB);
    void begin();

    int getPosition();
    bool getDirection();
    float calcRPM();

    static void encoderISR_A();
    static void encoderISR_B();
    static int getPulseCount();

private:
    int pinA, pinB;                // Encoder pin definitions
    static Encoder* instance;      // Pointer to the encoder instance
    int last_pulses;
    unsigned long prev_time;
    double rpm;
};

#endif
