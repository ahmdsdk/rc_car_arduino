#include <AFMotor.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define TRIG_PIN A0
#define ECHO_PIN A1
#define Tx A2
#define Rx A3
#define B_LIGHTS A4
#define F_LIGHTS A5
#define MODE_PIN 2
#define SERVO_PIN 10
#define MOTOR_SPEED 255
#define SPEED_FACTOR 2
#define MIN_DIST 45
#define LEFT_ANGLE 180
#define RIGHT_ANGLE 0
#define FORWARD_ANGLE 90

Servo servo;

AF_DCMotor motor1(1),
  motor2(2),
  motor3(3),
  motor4(4);

SoftwareSerial hc06(Tx, Rx);

int distanceF = 0,
  distanceL = 0,
  distanceR = 0,
  distance = 0,
  mode = 0;

char message = 0;

bool isTurnServo = true;

void setup() {
  setPins();
  setMotors();

  hc06.begin(9600);
  
  toggleLights();
}

void loop() {
  checkMode();
}

void setPins() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MODE_PIN, OUTPUT);
  pinMode(F_LIGHTS, OUTPUT);
  pinMode(B_LIGHTS, OUTPUT);
}

void setMotors() {
  motor1.setSpeed(MOTOR_SPEED);
  motor1.run(RELEASE);

  motor2.setSpeed(MOTOR_SPEED);
  motor2.run(RELEASE);

  motor3.setSpeed(MOTOR_SPEED);
  motor3.run(RELEASE);

  motor4.setSpeed(MOTOR_SPEED);
  motor4.run(RELEASE);

  servo.attach(SERVO_PIN);
}

void toggleLights() {
  digitalWrite(F_LIGHTS, HIGH);
  digitalWrite(B_LIGHTS, HIGH);
  digitalWrite(MODE_PIN, HIGH);
  delay(500);
  digitalWrite(F_LIGHTS, LOW);
  digitalWrite(B_LIGHTS, LOW);
  digitalWrite(MODE_PIN, LOW);
  delay(500);
  digitalWrite(F_LIGHTS, HIGH);
  digitalWrite(B_LIGHTS, HIGH);
  digitalWrite(MODE_PIN, HIGH);
  delay(500);
  digitalWrite(F_LIGHTS, LOW);
  digitalWrite(B_LIGHTS, LOW);
  digitalWrite(MODE_PIN, LOW);
  delay(500);
  digitalWrite(F_LIGHTS, HIGH);
  digitalWrite(B_LIGHTS, HIGH);
  digitalWrite(MODE_PIN, HIGH);
  delay(1000);
  digitalWrite(F_LIGHTS, HIGH);
  digitalWrite(B_LIGHTS, LOW);
  digitalWrite(MODE_PIN, LOW);
  
}

void checkMode() {
  if (mode == 0) {
    digitalWrite(MODE_PIN, HIGH);
    controllerMode();
  } else {
    digitalWrite(MODE_PIN, LOW);
    freeMode();
  }
}

void controllerMode() {
  if (hc06.available()) {
    message = hc06.read();
    if (message == '8') {
      drawEight();
    }
    stop();
    if (message == 'M') {
      changeMode();
    } else if (message == 'F') {
      moveForward();
    } else if (message == 'B') {
      moveBackward();
    } else if (message == 'R') {
      turnRight();
    } else if (message == 'L') {
      turnLeft();
    } else if (message == 'E') {
      forwardRight();
    } else if (message == 'C') {
      backwardRight();
    } else if (message == 'Q') {
      forwardLeft();
    } else if (message == 'Z') {
      backwardLeft();
    } else if (message == 'S') {
      stop();
    }
  }
}

void changeMode() {
  mode = (mode + 1) % 2;
  isTurnServo = !isTurnServo;
  delay(100);
  stop();
  checkMode();
}

void freeMode() {
  distanceF = 0;
  distanceL = 0;
  distanceR = 0;

  if (hc06.available()) {
    message = hc06.read();
    if (message == 'M') {
      stop();
      changeMode();
      return;
    }
  }

  // speed of sound 343 m/s = 0.0343 cm/μs
  distanceF = (readUltrasonicDuration(TRIG_PIN, ECHO_PIN) / 2) * 0.0343;
  delay(100);

  if (distanceF <= MIN_DIST) {
    delay(10);
    stop();
    delay(500);
    moveBackward();
    delay(200);
    stop();
    delay(200);

    servo.write(LEFT_ANGLE);
    delay(500);
    distanceL = (readUltrasonicDuration(TRIG_PIN, ECHO_PIN) / 2) * 0.0343;
    delay(100);

    servo.write(RIGHT_ANGLE);
    delay(500);
    distanceR = (readUltrasonicDuration(TRIG_PIN, ECHO_PIN) / 2) * 0.0343;
    delay(100);

    servo.write(FORWARD_ANGLE);
    if (distanceL <= distanceR) {
      delay(500);
      stop();
      delay(500);
      turnRight();
      delay(400);
      stop();
      delay(100);
    } else if (distanceL > distanceR) {
      delay(500);
      stop();
      delay(500);
      turnLeft();
      delay(400);
      stop();
      delay(100);
    }
  } else {
    moveForward();
  }
}

long readUltrasonicDuration(int triggerpin, int ECHO_PIN) {
  digitalWrite(triggerpin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerpin, LOW);

  return pulseIn(ECHO_PIN, HIGH);
}

void drawEight() {
  distanceF = (readUltrasonicDuration(TRIG_PIN, ECHO_PIN) / 2) * 0.0343;
  delay(100);

  if (distanceF <= MIN_DIST / 2) {
    digitalWrite(B_LIGHTS, HIGH);
    delay(500);
    digitalWrite(B_LIGHTS, LOW);
    return;
  }

  servo.write(LEFT_ANGLE);
  delay(500);
  distanceL = (readUltrasonicDuration(TRIG_PIN, ECHO_PIN) / 2) * 0.0343;
  delay(100);

  servo.write(RIGHT_ANGLE);
  delay(500);
  distanceR = (readUltrasonicDuration(TRIG_PIN, ECHO_PIN) / 2) * 0.0343;
  delay(100);

  turnRight();
  delay(700);
  stop();
  servo.write(FORWARD_ANGLE);
  delay(500);
  distanceF = (readUltrasonicDuration(TRIG_PIN, ECHO_PIN) / 2) * 0.0343;
  delay(100);

  turnRight();
  delay(700);
  stop();
  delay(1000);

  bool cancel = distanceF <= MIN_DIST / 2 || distanceR <= MIN_DIST || distanceL <= MIN_DIST;

  if (cancel) {
    digitalWrite(B_LIGHTS, HIGH);
    delay(500);
    digitalWrite(B_LIGHTS, LOW);
    return;
  }

  forwardRight();
  delay(2250);
  stop();
  delay(150);
  stop();
  forwardLeft();
  delay(2250);
  stop();
}

void moveForward() {
  if (isTurnServo) {
    servo.write(FORWARD_ANGLE);
  }

  motor1.setSpeed(MOTOR_SPEED);
  motor1.run(FORWARD);
  motor2.setSpeed(MOTOR_SPEED);
  motor2.run(FORWARD);
  motor3.setSpeed(MOTOR_SPEED);
  motor3.run(FORWARD);
  motor4.setSpeed(MOTOR_SPEED);
  motor4.run(FORWARD);

  digitalWrite(B_LIGHTS, LOW);
}

void moveBackward() {
  if (isTurnServo) {
    servo.write(FORWARD_ANGLE);
  }

  motor1.setSpeed(MOTOR_SPEED / SPEED_FACTOR);
  motor1.run(BACKWARD);
  motor2.setSpeed(MOTOR_SPEED / SPEED_FACTOR);
  motor2.run(BACKWARD);
  motor3.setSpeed(MOTOR_SPEED / SPEED_FACTOR);
  motor3.run(BACKWARD);
  motor4.setSpeed(MOTOR_SPEED / SPEED_FACTOR);
  motor4.run(BACKWARD);

  digitalWrite(B_LIGHTS, HIGH);
}

void turnLeft() {
  if (isTurnServo) {
    servo.write(LEFT_ANGLE);
  }

  motor1.setSpeed(MOTOR_SPEED);
  motor1.run(BACKWARD);
  motor2.setSpeed(MOTOR_SPEED);
  motor2.run(BACKWARD);
  motor3.setSpeed(MOTOR_SPEED);
  motor3.run(FORWARD);
  motor4.setSpeed(MOTOR_SPEED);
  motor4.run(FORWARD);

  digitalWrite(B_LIGHTS, LOW);
}

void turnRight() {
  if (isTurnServo) {
    servo.write(RIGHT_ANGLE);
  }

  motor1.setSpeed(MOTOR_SPEED);
  motor1.run(FORWARD);
  motor2.setSpeed(MOTOR_SPEED);
  motor2.run(FORWARD);
  motor3.setSpeed(MOTOR_SPEED);
  motor3.run(BACKWARD);
  motor4.setSpeed(MOTOR_SPEED);
  motor4.run(BACKWARD);

  digitalWrite(B_LIGHTS, LOW);
}

void forwardLeft() {
  if (isTurnServo) {
    servo.write(150);
  }

  motor1.setSpeed(0);
  motor1.run(RELEASE);
  motor2.setSpeed(0);
  motor2.run(RELEASE);
  motor3.setSpeed(MOTOR_SPEED);
  motor3.run(FORWARD);
  motor4.setSpeed(MOTOR_SPEED);
  motor4.run(FORWARD);

  digitalWrite(B_LIGHTS, LOW);
}

void forwardRight() {
  if (isTurnServo) {
    servo.write(30);
  }

  motor1.setSpeed(MOTOR_SPEED);
  motor1.run(FORWARD);
  motor2.setSpeed(MOTOR_SPEED);
  motor2.run(FORWARD);
  motor3.setSpeed(0);
  motor3.run(RELEASE);
  motor4.setSpeed(0);
  motor4.run(RELEASE);

  digitalWrite(B_LIGHTS, LOW);
}

void backwardLeft() {
  if (isTurnServo) {
    servo.write(150);
  }

  motor1.setSpeed(0);
  motor1.run(RELEASE);
  motor2.setSpeed(0);
  motor2.run(RELEASE);
  motor3.setSpeed((MOTOR_SPEED / SPEED_FACTOR) * 1.5);
  motor3.run(BACKWARD);
  motor4.setSpeed((MOTOR_SPEED / SPEED_FACTOR) * 1.5);
  motor4.run(BACKWARD);

  digitalWrite(B_LIGHTS, HIGH);
}

void backwardRight() {
  if (isTurnServo) {
    servo.write(30);
  }

  motor1.setSpeed((MOTOR_SPEED / SPEED_FACTOR) * 1.5);
  motor1.run(BACKWARD);
  motor2.setSpeed((MOTOR_SPEED / SPEED_FACTOR) * 1.5);
  motor2.run(BACKWARD);
  motor3.setSpeed(0);
  motor3.run(RELEASE);
  motor4.setSpeed(0);
  motor4.run(RELEASE);

  digitalWrite(B_LIGHTS, HIGH);
}

void stop() {
  if (isTurnServo) {
    servo.write(FORWARD_ANGLE);
  }

  motor1.setSpeed(0);
  motor1.run(RELEASE);
  motor2.setSpeed(0);
  motor2.run(RELEASE);
  motor3.setSpeed(0);
  motor3.run(RELEASE);
  motor4.setSpeed(0);
  motor4.run(RELEASE);

  digitalWrite(B_LIGHTS, LOW);
}  
