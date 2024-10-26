#include <Arduino.h>
#include "encoder.h"

Encoder encoder(2 , 4);



void setup() {
    Serial.begin(115200);
    encoder.begin();
    pinMode(5, OUTPUT);
    analogWrite(5, 100);
}

void loop() {
    Serial.println(encoder.get());
    delay(200);
}