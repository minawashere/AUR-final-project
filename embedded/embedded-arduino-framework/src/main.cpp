#include "Arduino.h"
#include "servo.h"
#include "WiFi.h"
#include "PubSubClient.h"
#include "ArduinoJson.h"


auto wheel_vel = "/wheel_vel";
auto cmd_servo = "/cmd_servo";


static auto servo = Servo(2, 0);

void setup() {
    Serial.begin(9600);
}

void loop() {
    for (int i = 0; i < 4000; i++) {
        ledcWrite(0, i);
        Serial.println(i);
        delay(100);
    }
}

