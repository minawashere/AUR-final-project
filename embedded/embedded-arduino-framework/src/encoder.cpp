#include "encoder.h"
#include "Arduino.h"
#include "float.h"

double Encoder::rpm() {
    const unsigned long time = micros();
    const double dt = (time - prev_time) / 1000000.0;
    const uint32_t dp = pulse_count - prev_pulses;

    this->prev_time = time;
    this->prev_pulses = pulse_count;

    if (dt < DBL_EPSILON) return prev_rpm;
    prev_rpm = dp / dt * 60 / PULSES_PER_REV / GEAR_RATIO * static_cast<double>(m_direction);
    return prev_rpm;
}


// #include "encoder.h"
// #include <float.h>
//
// #define PULSES_PER_REV 11
// #define GEAR_RATIO 46.8
//
//
// Encoder::Encoder(const uint8_t pinA, const uint8_t pinB, void (*isr)()) : pinA(pinA), pinB(pinB) {
//     pinMode(pinA, INPUT);
//     pinMode(pinB, INPUT);
//
//     attachInterrupt(digitalPinToInterrupt(pinA), isr, RISING);
// }
//
//
// double Encoder::rpm() {
//     const unsigned long time = micros();
//     const double dt = (time - prev_time) / 1000000.0;
//     const uint32_t dp = pulse_count() - prev_pulses;
//
//     this->prev_time = time;
//     this->prev_pulses = pulse_count();
//
//     if (dt < DBL_EPSILON) return prev_rpm;
//     return dp / dt * 60 / PULSES_PER_REV / GEAR_RATIO * static_cast<double>(direction());
// }
//
// #include "encoder.h"
//
// #define PPR 22  // Pulses per revolution
//
// // Static variable definitions
// volatile int encoderPosition = 0;
// volatile int encoderDirection = 0;
// volatile int pulseCount = 0;
// Encoder* Encoder::instance = nullptr; // Initialize the static instance pointer
// volatile unsigned long last_time = 0;
//
// Encoder::Encoder(const int pinA, const int pinB) : pinA(pinA), pinB(pinB) {
//     last_time = micros();
// }
//
// void Encoder::begin() {
//     Serial.println("Initializing encoder...");
//
//     pinMode(pinA, INPUT);
//     pinMode(pinB, INPUT);
//
//     instance = this;
//
//     attachInterrupt(digitalPinToInterrupt(pinA), encoderISR_A, RISING);
//     attachInterrupt(digitalPinToInterrupt(pinB), encoderISR_B, RISING);
//
//     Serial.println("Encoder initialized.");
// }
//
// int Encoder::getPosition() {
//
//     noInterrupts();
//     int position = encoderPosition;
//     interrupts();
//     return position;
// }
//
// bool Encoder::getDirection() {
//     // noInterrupts();
//     // bool direction = encoderDirection;
//     // interrupts();
//     return encoderDirection;
// }
//
// int Encoder::getPulseCount(){
//     // noInterrupts();
//     // int count = pulseCount;
//     // interrupts();
//     return pulseCount;
// }
//
// void Encoder::encoderISR_A() {
//     if (instance) {
//         bool pinAState = digitalRead(instance->pinA);
//         bool pinBState = digitalRead(instance->pinB);
//
//         if (pinAState != pinBState) {
//             encoderPosition++;
//             encoderDirection = 1;
//         } else {
//             encoderPosition--;
//             encoderDirection = 0;
//         }
//         pulseCount++;
//     }
// }
//
// void Encoder::encoderISR_B() {
//     if (instance) {
//         bool pinAState = digitalRead(instance->pinA);
//         bool pinBState = digitalRead(instance->pinB);
//
//         if (pinAState == pinBState) {
//             encoderPosition++;
//             encoderDirection = 1;
//         } else {
//             encoderPosition--;
//             encoderDirection = 0;
//         }
//         pulseCount++;
//     }
// }
//
// float Encoder::calcRPM() {
//     const unsigned long time = micros();
//     const double dt = (time - last_time) / 1000000.0;
//     if (dt > 0) {
//         rpm = (pulseCount - last_pulses)  * 60.0 / dt / PPR / 46.8;
//     }
//
//     last_time = time;
//     last_pulses = pulseCount;
//     return rpm;
// }
