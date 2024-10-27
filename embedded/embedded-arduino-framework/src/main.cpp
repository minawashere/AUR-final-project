#include "encoder.h"
#include "Arduino.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char *ssid = "Mina's Galaxy Note20 Ultra 5G";
const char *password = "loli1414";

const char *mqtt_broker = "mqtt.eclipseprojects.io";
const char *topic = "Motion Commands";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
const unsigned long interval = 20; // 20 ms for 50 Hz

// on message mqtt
void callback(const char *topic, const byte *payload, unsigned int length);

//setting encoder's ISR
void isr();

void isr2();

static auto encoderLeft = Encoder(1, 2, isr);
static auto encoderRight = Encoder(1, 2, isr2);

void IRAM_ATTR isr() { encoderLeft.handleISR(); }
void IRAM_ATTR isr2() { encoderRight.handleISR(); }

void callback(const char *topic, const byte *payload, const unsigned int length) {
    // Handle incoming messages
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the Wi-Fi network");

    // Connect to MQTT broker
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);
    while (!client.connected()) {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
        if (client.connect(client_id.c_str())) {
            Serial.println("Public EMQX MQTT broker connected");
        } else {
            Serial.print("Failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }

    // Publish and subscribe
    client.publish(topic, "MQTT CONNECTED");
    client.subscribe(topic);
}

void loop() {
    // Get the current time
    unsigned long currentTime = millis();

    // Send data every 20ms (50 Hz)
    if (currentTime - lastMsgTime >= interval) {
        lastMsgTime = currentTime;

        // IMU data using your library
        int16_t accData[3];
        int16_t gyrData[3];

        // Print IMU data to Serial Monitor
        Serial.print("Acc X: ");
        Serial.print(accData[0]) / 16384.0;
        Serial.print(" Acc Y: ");
        Serial.print(accData[1]) / 16384.0;
        Serial.print(" Acc Z: ");
        Serial.println(accData[2]) / 16384.0;
        Serial.print("Gyr X: ");
        Serial.print(gyrData[0]) / 131.0;
        Serial.print(" Gyr Y: ");
        Serial.print(gyrData[1]) / 131.0;
        Serial.print(" Gyr Z: ");
        Serial.println(gyrData[2]) / 131.0;

        // Create a JSON array to hold the data
        StaticJsonDocument<JSON_ARRAY_SIZE(8)> doc;
        JsonArray array = doc.to<JsonArray>();

        // Fill the array with IMU data (first 6 slots) and 0 for the last 2 slots
        array.add(1); // Acc X
        array.add(2); // Acc Y
        array.add(3); // Acc Z
        array.add(4); // Gyr X
        array.add(5); // Gyr Y
        array.add(6); // Gyr Z
        array.add(7); // Placeholder
        array.add(8); // Placeholder

        // Determine the size needed for the MessagePack format
        size_t bufferSize = measureMsgPack(doc);

        // Allocate the buffer to store serialized MessagePack
        uint8_t payload[bufferSize];

        // Serialize the array into the MessagePack buffer
        serializeMsgPack(doc, payload, bufferSize);

        // Publish the MessagePack data
        client.publish(topic, payload, bufferSize);
    }

    // Maintain the MQTT connection
    client.loop();
}
