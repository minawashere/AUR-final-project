#include "Encoder.h"

// Static variables to store encoder states
volatile int Encoder::encoderPosition = 0;
volatile bool Encoder::encoderDirection = 0;

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
        encoderDirection = 0;
    }
}
