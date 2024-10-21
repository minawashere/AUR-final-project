#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder
{
public:
    Encoder(int pinA, int pinB); // Constructor to initialize the encoder pins
    void begin();                // Method to set up pins and interrupts
    int getPosition();           // Method to get the current position
    bool getDirection();         // Method to get the rotation direction

private:
    int pinA, pinB;
    volatile int encoderPosition;
    volatile bool encoderDirection;

    static void IRAM_ATTR encoderISR_A(); // Interrupt service routines
    static void IRAM_ATTR encoderISR_B();
};

#endif
