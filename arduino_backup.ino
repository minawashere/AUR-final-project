/* Something we wrote 5 mins before our training competetion as our ESP32 fired */


const int motor1Pin1 = 9;   
const int motor1Pin2 = 8;   
const int motor1Enable = 10; 

const int motor2Pin1 = 7;   
const int motor2Pin2 = 6;   
const int motor2Enable = 11; 

void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor1Enable, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor2Enable, OUTPUT);


  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    lastCommand = command;
  }

  switch (lastCommand) {
    case 'w':
      moveForward();
      break;
    case 's':
      moveBackward();
      break;
    case 'a': 
      turnRight();
      break;
    case 'd':
      turnLeft();
      break;
    case 'x':
      stopMotors();
      break;
    default:  
      stopMotors();
      break;
  }
}

void moveForward() {
  digitalWrite(motor1Pin1, HIGH);  
  digitalWrite(motor1Pin2, LOW);
  analogWrite(motor1Enable, motorSpeed);

  digitalWrite(motor2Pin1, HIGH);  
  digitalWrite(motor2Pin2, LOW);
  analogWrite(motor2Enable, motorSpeed);
  delay(500);
}

void moveBackward() {
  digitalWrite(motor1Pin1, LOW); 
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(motor1Enable, motorSpeed);

  digitalWrite(motor2Pin1, LOW);   
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(motor2Enable, motorSpeed);
    delay(300);

}

void turnLeft() {
  digitalWrite(motor1Pin1, HIGH);  
  digitalWrite(motor1Pin2, LOW);
  analogWrite(motor1Enable, motorSpeed);

  digitalWrite(motor2Pin1, LOW);  
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(motor2Enable, motorSpeed);
    delay(300);

}

void turnRight() {
  digitalWrite(motor1Pin1, LOW);   
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(motor1Enable, motorSpeed);

  digitalWrite(motor2Pin1, HIGH);  
  digitalWrite(motor2Pin2, LOW);
  analogWrite(motor2Enable, motorSpeed);
    delay(300);

}

void stopMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(motor1Enable, 0);  

  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(motor2Enable, 0); 
    delay(300);

}
