#include <Arduino.h>
#include <Wire.h>
#include <float.h>

#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "PID.h"
#include "imu.h"
#include "servo.h"

#define SDA_PIN 21
#define SCL_PIN 22

#define GEAR_RATIO 46.8
#define PULSES_PER_REV 11

#define ENCODERLEFT_PIN_A 2
#define ENCODERLEFT_PIN_B 4
#define ENCODERRIGHT_PIN_A 18
#define ENCODERRIGHT_PIN_B 19

#define MOTOR1_PIN_A 20
#define MOTOR1_PIN_B 21
#define MOTOR2_PIN_A 22
#define MOTOR2_PIN_B 23

#define MOTOR1_PWM 25
#define MOTOR2_PWM 26

#define MAX_SPEED 0.4  // m/s
#define SMOOTHING_FACTOR 0.9

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
  Motor(const uint8_t pin1, const uint8_t pin2, const uint8_t pwm_pin) : pin1(pin1), pin2(pin2), pwm_pin(pwm_pin) {
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

auto ssid = "Mina's Galaxy Note20 Ultra 5G";
auto password = "loli1414";

auto mqtt_broker = "mqtt.eclipseprojects.io";
auto topic = "Motion Commands";
constexpr int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
constexpr unsigned long interval = 20; //ms

static auto pidRight = PID(1, 0, 0, MAX_SPEED);
static auto pidLeft = PID(1, 1, 0, MAX_SPEED);

static auto motorRight = Motor(MOTOR1_PIN_A, MOTOR1_PIN_B, MOTOR1_PWM);
static auto motorLeft = Motor(MOTOR2_PIN_A, MOTOR2_PIN_B, MOTOR2_PWM);


static auto encoderLeft = Encoder(ENCODERLEFT_PIN_A, ENCODERLEFT_PIN_B);
static auto encoderRight = Encoder(ENCODERRIGHT_PIN_A, ENCODERRIGHT_PIN_B);

double softStart(const double target, const double current, const double factor) {
  return factor * current + (1 - factor) * target;
}

void isrRight() {
  encoderRight.pulse_count++;
  encoderRight.m_direction = (GPIO.in & (1 << ENCODERLEFT_PIN_B)) ? BACKWARDS : FORWARDS;
  encoderRight.m_position += encoderRight.m_direction;
}

void isrLeft() {
  encoderLeft.pulse_count++;
  encoderLeft.m_direction = (GPIO.in & (1 << ENCODERRIGHT_PIN_B)) ? BACKWARDS : FORWARDS;
  encoderLeft.m_position += encoderLeft.m_direction;
}

void Task1_readSensors_sendData(void *pvParameters) {
  IMU imu(SDA_PIN, SCL_PIN);

  StaticJsonDocument<JSON_ARRAY_SIZE(8)> doc;
  const auto array = doc.to<JsonArray>();

  while (true) {
    imu.fetchIMU();
    // imu.accData,imu.gyroData;(data of the imu that will be send to the nav)
    int32_t position1 = encoderLeft.m_position;
    int32_t position2 = encoderRight.m_position;

    array.add(imu.accData[0]); // Acc X
    array.add(imu.accData[1]); // Acc Y
    array.add(imu.accData[2]); // Acc Z
    array.add(imu.gyroData[0]); // Gyr X
    array.add(imu.gyroData[1]); // Gyr Y
    array.add(imu.gyroData[2]); // Gyr Z
    array.add(encoderRight.m_position); // leftEncoderPosition
    array.add(encoderLeft.m_position); // rightEncoderPosition

    const size_t bufferSize = measureMsgPack(doc);

    uint8_t payload[bufferSize];

    serializeMsgPack(doc, payload, bufferSize);

    client.publish(topic, payload, bufferSize);

    vTaskDelay(pdMS_TO_TICKS(15));
  }
}

// void Task2_fetchBrokerData(void* pvParameters) {
//   //
//   // double leftVel;
//   // double rightVel;
//   // double velArr[] = { leftVel, rightVel };
//   // while (1) {
//   //   if (xQueueSend(dataQueue, velArr, portMAX_DELAY) != pdTRUE) {
//   //     Serial.println("Error:failed to send the data to the queue");
//   //   }
//   //   //servo1 functions and data
//   //   //servo2 functions and data
//   // }
// }

void Task2_Fetch_and_Drive(void *pvParameters) {
  while (true) {
    double leftVel;
    double rightVel;

    //add mqtt

    const double rpmRight = encoderRight.rpm();
    const double rpmLeft = encoderLeft.rpm();

    const Direction directionRight = rightVel > 0 ? FORWARDS : BACKWARDS;
    const Direction directionLeft = leftVel > 0 ? FORWARDS : BACKWARDS;

    const double outputRight = softStart(pidRight.get_output(rightVel, rpmRight), rightVel,SMOOTHING_FACTOR);
    const double outputLeft = softStart(pidLeft.get_output(leftVel, rpmLeft), rpmLeft,SMOOTHING_FACTOR);
    motorRight.drive(directionRight, outputRight);
    motorLeft.drive(directionLeft, outputLeft);
  }
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
  // client.setCallback(callback);
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


  // dataQueue = xQueueCreate(20, sizeof(imuData));
  // if (dataQueue == NULL) {
  //   Serial.print("Error in initalizing queue");
  // }
  xTaskCreate(Task1_readSensors_sendData, "Task1_readData", 2048, NULL, 1, NULL);
  xTaskCreate(Task2_Fetch_and_Drive, "Task2_FetchDataFromBrokwer_PID_MotorDriver", 2048, NULL, 1,
              NULL);
  // xTaskCreate(Task3_PID_MotorDriver, "Task3_PID_MotorDriver", 2048, NULL, 1, NULL);
}
