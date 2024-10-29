#include "Arduino.h"
#include "float.h"

#include "ArduinoJson.h"
#include "PubSubClient.h"
#include "WiFi.h"

#include "PID.h"
#include "imu.h"
#include "servo.h"

#define GEAR_RATIO 46.8
#define PULSES_PER_REV 11

#define ENCODER1_PIN_A 2
#define ENCODER1_PIN_B 4
#define ENCODER2_PIN_A 18
#define ENCODER2_PIN_B 19

#define MOTOR1_PIN_A 20
#define MOTOR1_PIN_B 21
#define MOTOR2_PIN_A 22
#define MOTOR2_PIN_B 23

#define MOTOR1_PWM 25
#define MOTOR2_PWM 26

#define MAX_SPEED 0.4 // m/s


enum Direction { FORWARDS = 1, BACKWARDS = -1 };

struct Encoder {
    Encoder(const uint8_t pinA, const uint8_t pinB): pinA(pinA), pinB(pinB) {
        pinMode(pinA, INPUT);
        pinMode(pinB, INPUT);
    }

    volatile int32_t m_position = 0;
    volatile Direction m_direction = FORWARDS;
    volatile uint32_t pulse_count = 0;

    double rpm() {
        const unsigned long time = micros();
        const double dt = (time - prev_time) / 1000000.0;
        const uint32_t dp = pulse_count - prev_pulses;

        this->prev_time = time;
        this->prev_pulses = pulse_count;

        if (dt < DBL_EPSILON) return prev_rpm;
        prev_rpm = dp / dt * 60 / PULSES_PER_REV / GEAR_RATIO * static_cast<double>(m_direction);
        return prev_rpm;
    }

private:
    const uint8_t pinA, pinB;
    uint32_t prev_pulses = 0;
    unsigned long prev_time = micros();
    double prev_rpm = 0;
};


struct Motor {
private:
    unsigned long prev_count = 0;
    unsigned long prev_time = 0;
    uint8_t pin1;
    uint8_t pin2;
    uint8_t pwm_pin;

public:
    Motor(const uint8_t pin1, const uint8_t pin2, const uint8_t pwm_pin) : pin1(pin1), pin2(pin2), pwm_pin(pwm_pin),
                                                                                                {
        pinMode(pwm_pin, OUTPUT);
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
    }

    void drive(const Direction direction, const uint8_t speed) const {
        const int l = (direction == FORWARDS) ? HIGH : LOW;
        digitalWrite(pin1, l);
        digitalWrite(pin2, !l);
        analogWrite(pwm_pin, speed);
    }
};

float softStart(const float target, const float current, const float factor) {
    return factor * current + (1 - factor) * target;
}


static auto encoderRight = Encoder(ENCODER1_PIN_A, ENCODER1_PIN_B);
static auto encoderLeft = Encoder(ENCODER2_PIN_A, ENCODER2_PIN_B);

static auto pidRight = PID(1, 0, 0, MAX_SPEED);
static auto pidLeft = PID(1, 1, 0, MAX_SPEED);

static auto motorRight = Motor(MOTOR1_PIN_A, MOTOR1_PIN_B, MOTOR1_PWM);
static auto motorLeft = Motor(MOTOR2_PIN_A, MOTOR2_PIN_B, MOTOR2_PWM);

void isrRight() {
    encoderRight.pulse_count++;
    encoderRight.m_direction = (GPIO.in & (1 << ENCODER1_PIN_B)) ? BACKWARDS : FORWARDS;
    encoderRight.m_position += encoderRight.m_direction;
}

void isrLeft() {
    encoderLeft.pulse_count++;
    encoderLeft.m_direction = (GPIO.in & (1 << ENCODER2_PIN_B)) ? BACKWARDS : FORWARDS;
    encoderLeft.m_position += encoderLeft.m_direction;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Hello World");
    delay(1000);

    attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), isrRight, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), isrLeft, RISING);
}

void loop() {
}
