#pragma once
#include <Arduino.h>

enum Direction {
    FORWARDS = 1,
    BACKWARDS = -1,
};

class Encoder {
public:
    Encoder(const uint8_t pinA, const uint8_t pinB, void (*isr)());


    int32_t position() const { return m_position; }
    Direction direction() const { return m_direction; }
    uint32_t pulse_count() const { return m_pulses; }

    double rpm();

    void handleISR() {
        this->m_pulses++;
        this->m_direction = (GPIO.in & (1 << this->pinB)) ? BACKWARDS : FORWARDS;
        this->m_position += m_direction;
    }

private:
    const uint8_t pinA, pinB;
    volatile int32_t m_position = 0;
    volatile Direction m_direction = FORWARDS;
    volatile uint32_t m_pulses = 0;

    uint32_t prev_pulses = 0;
    unsigned long prev_time = micros();
    double prev_rpm = 0;
};
