#include "encoder.h"
#include "Arduino.h"
#include "ArduinoJson"

void isr();

void isr2();

static auto encoderLeft = Encoder(1, 2, isr);
static auto encoderRight = Encoder(1, 2, isr2);

void IRAM_ATTR isr() { encoderLeft.handleISR(); }
void IRAM_ATTR isr2() { encoderRight.handleISR(); }

void setup() {
    Serial.begin(115200);
}


void loop() {
}
