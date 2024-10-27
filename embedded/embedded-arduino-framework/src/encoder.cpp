#include "encoder.h"
#include <float.h>

#define PULSES_PER_REV 11
#define GEAR_RATIO 46.8


Encoder::Encoder(const uint8_t pinA, const uint8_t pinB, void (*isr)()) : pinA(pinA), pinB(pinB) {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);

    attachInterrupt(digitalPinToInterrupt(pinA), isr, RISING);
}


double Encoder::rpm() {
    const unsigned long time = micros();
    const double dt = (time - prev_time) / 1000000.0;
    const uint32_t dp = pulse_count() - prev_pulses;

    this->prev_time = time;
    this->prev_pulses = pulse_count();

    if (dt < DBL_EPSILON) return prev_rpm;
    return dp / dt * 60 / PULSES_PER_REV / GEAR_RATIO * static_cast<double>(direction());
}

