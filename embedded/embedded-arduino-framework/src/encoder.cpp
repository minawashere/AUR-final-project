// #include "encoder.h"
// #include "Arduino.h"
//
// #define PPR 11
// #define GEAR_RATIO 46.8
//
//
// volatile int32_t positionRight = 0;
// volatile bool directionRight = 0;
// volatile uint32_t countRight = 0;
//
// volatile int32_t positionLeft = 0;
// volatile int8_t directionLeft = 0;
// volatile uint32_t countLeft = 0;
//
// Encoder *encoderRight = nullptr;
// Encoder *encoderLeft = nullptr;
//
// void isrRight();
//
// void isrLeft();
//
// Encoder::Encoder(const uint8_t pinA, const uint8_t pinB) : pinA(pinA), pinB(pinB) {
//     pinMode(pinA, INPUT);
//     pinMode(pinB, INPUT);
//
//     if (encoderRight == nullptr) {
//         encoderRight = this;
//         attachInterrupt(digitalPinToInterrupt(pinA), isrRight, RISING);
//     } else {
//         encoderLeft = this;
//         attachInterrupt(digitalPinToInterrupt(pinA), isrLeft, RISING);
//     }
// }
//
//
// int32_t Encoder::position() {
//     return this == encoderRight ? positionRight : positionLeft;
// }
//
// bool Encoder::direction() {
//     return this == encoderRight ? directionRight : directionLeft;
// }
//
// uint32_t Encoder::pulses() {
//     return this == encoderRight ? countRight : countLeft;
// }
//
//
// double Encoder::rpm() {
//     const unsigned long time = micros();
//     const double dt = (time - prev_time) / 1000000.0;
//     if (dt > 0) {
//         this->m_rpm = (countRight - prev_pulses) * 60.0 / dt / PPR / GEAR_RATIO;
//     }
//
//     prev_time = time;
//     prev_pulses = countRight;
//     return m_rpm;
// }
//
// void isrRight() {
//     countRight++;
//     // directionRight = (GPIO.in & (1 << encoderRight->pinB)) ? -1 : 1;
// }
