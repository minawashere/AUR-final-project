#include "encoder.h"
#include "esp_intr_alloc.h"
#include "soc/gpio_reg.h"

volatile int Encoder::encoderPosition = 0;
volatile bool encoderDirection = 0;
volatile int Encoder::pulseCount = 0;

#define PPR 22 // pulses per revolution

Encoder::Encoder(const int pinA, const int pinB)
{
    this->pinA = pinA;
    this->pinB = pinB;
}

void Encoder::begin()
{
    gpio_reset_pin(static_cast<gpio_num_t>(pinA));
    gpio_set_direction(static_cast<gpio_num_t>(pinA), GPIO_MODE_INPUT);
    gpio_set_pull_mode(static_cast<gpio_num_t>(pinA), GPIO_PULLUP_ONLY);

    gpio_reset_pin(static_cast<gpio_num_t>(pinB));
    gpio_set_direction(static_cast<gpio_num_t>(pinB), GPIO_MODE_INPUT);
    gpio_set_pull_mode(static_cast<gpio_num_t>(pinB), GPIO_PULLUP_ONLY);

    // Install ISR service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

    // Attach interrupts
    gpio_isr_handler_add(static_cast<gpio_num_t>(pinA), encoderISR_A, this);
    gpio_isr_handler_add(static_cast<gpio_num_t>(pinB), encoderISR_B, this);
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
void Encoder::encoderISR_A(void *arg)
{
    Encoder *enc = static_cast<Encoder *>(arg); // Cast the argument back to the Encoder object
    bool pinAState = (REG_READ(GPIO_IN_REG) >> enc->pinA) & 0x1;
    bool pinBState = (REG_READ(GPIO_IN_REG) >> enc->pinB) & 0x1;

    if (pinAState != pinBState)
    {
        Encoder::encoderPosition++;
        Encoder::encoderDirection = 1;
    }
    else
    {
        Encoder::encoderPosition--;
        Encoder::encoderDirection = 0;
    }
    Encoder::pulseCount++;
}

// ISR for pin B
void Encoder::encoderISR_B(void *arg)
{
    Encoder *enc = static_cast<Encoder *>(arg); // Cast the argument back to the Encoder object
    bool pinAState = (REG_READ(GPIO_IN_REG) >> enc->pinA) & 0x1;
    bool pinBState = (REG_READ(GPIO_IN_REG) >> enc->pinB) & 0x1;

    if (pinAState == pinBState)
    {
        Encoder::encoderPosition++;
        Encoder::encoderDirection = 1;
    }
    else
    {
        Encoder::encoderPosition--;
        Encoder::encoderDirection = 0;
    }
    Encoder::pulseCount++;
}

float Encoder::calcRPM()
{
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    float timeInterval = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    float rpm = (pulseCount / PPR) * (60.0 / timeInterval);
    pulseCount = 0;
    return rpm;
}
