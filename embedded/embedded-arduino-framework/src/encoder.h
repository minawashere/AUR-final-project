#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#define PPR 22                // Pulses per revolution
#define ENCODER1_PIN_A 21
#define ENCODER1_PIN_B 22

struct Encoder {
    Encoder(int pinA, int pinB); // Constructor to initialize the encoder pins
    void begin();                // Method to set up pins and interrupts
    int getPosition() const;     // Method to get the current position
    Direction getDirection() const; // Method to get the rotation direction
    float getDistance() const;   // Method to calculate distance moved
    uint8_t getSpeed() const;    // Method to calculate speed

private:
    int pinA, pinB;              // Encoder pins
    volatile int encoderPosition;   // Encoder position counter
    volatile Direction encoderDirection;  // Rotation direction

    static void IRAM_ATTR encoderISR_A(); // Interrupt service routines for each channel
    static void IRAM_ATTR encoderISR_B();
};

#endif
