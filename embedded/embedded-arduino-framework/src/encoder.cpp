#include "Encoder.h"

// Static variables to store encoder states
volatile int Encoder::encoderPosition = 0;
volatile int Encoder::encoderDirection = 0;
volatile int Encoder::pulseCount = 0;

#define PPR 44 // pulses per rev
// #define WHEEL_DIAMETER                            // Wheel diameter in cm (example)
// #define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER) // Circumference in cm
// #define DISTANCE_PER_PULSE

Encoder::Encoder(int pinA, int pinB)
{
    this->pinA = pinA;
    this->pinB = pinB;
}

void Encoder::begin()
{
    // Setup pins
    gpio_pad_select_gpio(pinA);
    gpio_set_direction(static_cast<gpio_num_t>(pinA), GPIO_MODE_INPUT);
    gpio_set_pull_mode(static_cast<gpio_num_t>(pinA), GPIO_PULLUP_ONLY);

    gpio_pad_select_gpio(pinB);
    gpio_set_direction(static_cast<gpio_num_t>(pinB), GPIO_MODE_INPUT);
    gpio_set_pull_mode(static_cast<gpio_num_t>(pinB), GPIO_PULLUP_ONLY);

    // Attach interrupts
    gpio_isr_handler_add(static_cast<gpio_num_t>(pinA), encoderISR_A, NULL);
    gpio_isr_handler_add(static_cast<gpio_num_t>(pinB), encoderISR_B, NULL);
}

int Encoder::getPosition()
{
    return encoderPosition;
}

bool Encoder::getDirection()
{
    return encoderDirection;
}

// float Encoder::getDistance()
// {
//     return encoderPosition * DISTANCE_PER_PULSE;
// }

// ISR for pin A
void IRAM_ATTR Encoder::encoderISR_A()
{
    bool pinAState = (GPIO.in >> ENCODER_PIN_A) & 0x1;
    bool pinBState = (GPIO.in >> ENCODER_PIN_B) & 0x1;

    if (pinAState != pinBState)
    {
        encoderPosition++;
        encoderDirection = 1;
    }
    else
    {
        encoderPosition--;
        encoderDirection = 0;
    }
    pulseCount++;
}

// ISR for pin B
void IRAM_ATTR Encoder::encoderISR_B()
{
    bool pinAState = (GPIO.in >> ENCODER_PIN_A) & 0x1;
    bool pinBState = (GPIO.in >> ENCODER_PIN_B) & 0x1;

    if (pinAState == pinBState)
    {
        encoderPosition++;
        encoderDirection = 1;
    }
    else
    {
        encoderPosition--;
        encoderDirection = -1;
    }
    pulseCount++;
}

float Encoder::calcRPM()
{
    float rpm = (pulseCount / PPR) * 60;
    pulseCount = 0;
    return rpm;
}
