#include "encoder.h"

#define PPR 11


volatile int positionRight = 0;
volatile int directionRight = 0;
volatile int countRight = 0;

volatile int positionLeft = 0;
volatile int directionLeft = 0;
volatile int countLeft = 0;

Encoder *encoder = nullptr;
Encoder *encoder = nullptr;


volatile unsigned long last_time = 0;

Encoder::Encoder(const int pinA, const int pinB) : pinA(pinA), pinB(pinB) {
    last_time = micros();
}

void Encoder::begin() {
    Serial.println("Initializing encoder...");

    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);

    instance = this;

    attachInterrupt(digitalPinToInterrupt(pinA), encoderISR_A, RISING);
    attachInterrupt(digitalPinToInterrupt(pinB), encoderISR_B, RISING);

    Serial.println("Encoder initialized.");
}

int Encoder::getPosition() {
    noInterrupts();
    int position = positionRight;
    interrupts();
    return position;
}

bool Encoder::getDirection() {
    // noInterrupts();
    // bool direction = encoderDirection;
    // interrupts();
    return directionRight;
}

int Encoder::getPulseCount() {
    // noInterrupts();
    // int count = pulseCount;
    // interrupts();
    return countRight;
}

void Encoder::encoderISR_A() {
    if (instance) {
        bool pinAState = digitalRead(instance->pinA);
        bool pinBState = digitalRead(instance->pinB);

        if (pinAState != pinBState) {
            positionRight++;
            directionRight = 1;
        } else {
            positionRight--;
            directionRight = 0;
        }
        countRight++;
    }
}

void Encoder::encoderISR_B() {
    if (instance) {
        bool pinAState = digitalRead(instance->pinA);
        bool pinBState = digitalRead(instance->pinB);

        if (pinAState == pinBState) {
            positionRight++;
            directionRight = 1;
        } else {
            positionRight--;
            directionRight = 0;
        }
        countRight++;
    }
}

float Encoder::calcRPM() {
    const unsigned long time = micros();
    const double dt = (time - last_time) / 1000000.0;
    if (dt > 0) {
        rpm = (countRight - last_pulses) * 60.0 / dt / PPR / 46.8;
    }

    last_time = time;
    last_pulses = countRight;
    return rpm;
}
