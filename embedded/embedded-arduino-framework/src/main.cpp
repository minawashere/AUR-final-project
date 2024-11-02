#include <Arduino.h>
#include <Wire.h>
#include <float.h>

#include <WiFi.h>
#include <ArduinoJson.h>
#include <MsgPack.h>
#include <PubSubClient.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "PID.h"
#include "imu.h"
#include "servo.h"
#include "encoder.h"
#include "motor.h"

#define SDA_PIN 21
#define SCL_PIN 22

#define ENCODER_LEFT_PIN_A 4
#define ENCODER_LEFT_PIN_B 18
#define ENCODER_RIGHT_PIN_A 2
#define ENCODER_RIGHT_PIN_B 15

#define MOTOR_LEFT_PIN_A 12
#define MOTOR_LEFT_PIN_B 14
#define MOTOR_RIGHT_PIN_A 19
#define MOTOR_RIGHT_PIN_B 23

#define MOTOR_LEFT_PWM 27
#define MOTOR_RIGHT_PWM 13

#define MAX_SPEED 110  // rpm
#define SMOOTHING_FACTOR 0.9

#define SERVO_ELEV_PIN 26
#define SERVO_GRIPPER_PIN 25

#define SERVO_ELEV_CHANNEL 0
#define SERVO_GRIPPER_CHANNEL 1


auto ssid = "Mina's Galaxy Note20 Ultra 5G";
auto password = "loli1414";

auto mqtt_broker = "mqtt.eclipseprojects.io";
auto topic = "Motion Commands";
constexpr int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
constexpr unsigned long interval = 20; //ms

static auto pidRight = PID(1, 0.5, 0, 110);
static auto pidLeft = PID(1, 0.5, 0, 110);

static auto motorLeft = Motor(MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B, MOTOR_LEFT_PWM);
static auto motorRight = Motor(MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B, MOTOR_RIGHT_PWM);


double leftVel = 1;
double rightVel = 1;

int servoGripperData ;
int servoElevData;


static auto encoderLeft = Encoder(ENCODER_LEFT_PIN_A, ENCODER_LEFT_PIN_B);
static auto encoderRight = Encoder(ENCODER_RIGHT_PIN_A, ENCODER_RIGHT_PIN_B);

static auto servoElev=Servo(SERVO_ELEV_PIN,SERVO_ELEV_CHANNEL);
static auto servoGripper=Servo(SERVO_GRIPPER_PIN,SERVO_GRIPPER_CHANNEL);

double softStart(const double target, const double current, const double factor) {
  return factor * current + (1 - factor) * target;
}

void IRAM_ATTR isrRight() {
  encoderRight.pulse_count++;
  encoderRight.m_direction = (GPIO.in & (1 << ENCODER_RIGHT_PIN_B)) ? BACKWARDS : FORWARDS;
  encoderRight.m_position += encoderRight.m_direction;
  // Serial.println(encoderRight.m_position);
}

void IRAM_ATTR isrLeft() {
  encoderLeft.pulse_count++;
  encoderLeft.m_direction = (GPIO.in & (1 << ENCODER_LEFT_PIN_B)) ? BACKWARDS : FORWARDS;
  encoderLeft.m_position += encoderLeft.m_direction;
  // Serial.println(encoderLeft.m_position);
}


void Task1_readSensors_sendData(void *pvParameters) {
  IMU imu(SDA_PIN, SCL_PIN);
  while (true) {

    imu.requestImu(); //change to imu.readData()
    JsonDocument sensors;
    auto array = sensors.to<JsonArray>();

    /* USE DEDICATED MSGPACK LIBRARY */

    array.add(imu.accData[0]);
    array.add(imu.accData[1]);
    array.add(imu.accData[2]);
    array.add(imu.gyroData[0]);
    array.add(imu.gyroData[1]);
    array.add(imu.gyroData[2]);
    array.add(encoderRight.m_position);
    array.add(encoderLeft.m_position);

    const size_t bufferSize = measureMsgPack(sensors);

    uint8_t payload[bufferSize];

    serializeMsgPack(sensors, payload, bufferSize);

    client.publish(topic, payload, bufferSize);

    client.loop();

    vTaskDelay(1);
  }
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc;
  const DeserializationError error = deserializeMsgPack(doc, payload, length);



  if (error) {
    Serial.print(F("Failed to deserialize MessagePack: "));
    Serial.println(error.c_str());
    return;
  }

  const auto array = doc.as<JsonArray>();
  leftVel = array[0];
  rightVel = array[1];
  servoElevData = array[2];
  servoGripperData = array[3];

  if(servoElevData!=0)
  {
    static uint16_t angle=0;
    if(angle <= 180 || angle >= 0)
    {
      servoElev.write(angle);
      angle+= servoElevData;
    }
  }


  if(servoGripperData!=0)
  {
    static uint16_t angle=0;
    if(angle <= 180 || angle >= 0)
    {
      servoGripper.write(angle);
      angle+= servoGripperData;
    }
  }

  vTaskDelay(pdMS_TO_TICKS(1));
}


void Task2_Fetch_and_Drive(void *pvParameters) {
  while (true) {


    const double rpmRight = encoderRight.rpm();
    const double rpmLeft = encoderLeft.rpm();

    const
        Direction
        directionRight = rightVel > 0 ? FORWARDS : BACKWARDS;
    const
        Direction
        directionLeft = leftVel > 0 ? FORWARDS : BACKWARDS;

    const double outputRight = softStart(pidRight.get_output(rightVel * 95, rpmRight), rpmRight,SMOOTHING_FACTOR);
    const double outputLeft = softStart(pidLeft.get_output(leftVel * 95, rpmLeft), rpmLeft,SMOOTHING_FACTOR);
    motorRight.drive(directionRight, outputRight);
    motorLeft.drive(directionLeft, outputLeft);
    vTaskDelay(pdMS_TO_TICKS(1));

  }
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  servoElev.write(90);
  servoGripper.write(90);
  Serial.println("servo set");

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN_A), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN_A), isrRight, RISING);

  Serial.println("motor set");


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

  client.setCallback(mqttCallback);


  // dataQueue = xQueueCreate(20, sizeof(imuData));
  // if (dataQueue == NULL) {
  //   Serial.print("Error in initalizing queue");
  // }
  xTaskCreate(
    Task1_readSensors_sendData,
    "Task1_readData",
    2048,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    Task2_Fetch_and_Drive,
    "Task2_FetchDataFromBrokwer_PID_MotorDriver",
    2048,
    NULL,
    1,
    NULL
  );
  // xTaskCreate(Task3_PID_MotorDriver, "Task3_PID_MotorDriver", 2048, NULL, 1, NULL);
}

void loop() {
}

