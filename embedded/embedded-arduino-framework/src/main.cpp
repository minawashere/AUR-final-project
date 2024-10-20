#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"

void setup() {
    Serial.begin(115200);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    i2c_init();
    imu_init();
}

void loop() {

}