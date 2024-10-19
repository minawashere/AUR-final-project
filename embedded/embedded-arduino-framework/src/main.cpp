#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

static const int led_pin = 2;

void toggleLED(void *parameter) {
    while(1) {
        digitalWrite(led_pin, HIGH);
        vTaskDelay(5 / portTICK_PERIOD_MS);
        digitalWrite(led_pin, LOW);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}


void setup() {
    pinMode(led_pin, OUTPUT);

    xTaskCreatePinnedToCore(
        toggleLED,
        "toggleLED",
        1024,
        NULL,
        1,
        NULL,
        app_cpu);

}

void loop() {
// write your code here
}