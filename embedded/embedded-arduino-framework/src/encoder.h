#pragma once

#include <Arduino.h>
#include <motor.h>

#define GEAR_RATIO 46.8
#define PULSES_PER_REV 11

struct Encoder {
    Encoder(const uint8_t pinA, const uint8_t pinB): pinA(pinA), pinB(pinB) {
        pinMode(pinA, INPUT);
        pinMode(pinB, INPUT);
    }

    volatile int32_t m_position = 0;
    volatile Direction m_direction = FORWARDS;
    volatile uint32_t pulse_count = 0;

    double rpm();

private:
    const uint8_t pinA, pinB;
    uint32_t prev_pulses = 0;
    unsigned long prev_time = micros();
    double prev_rpm = 0;
};
