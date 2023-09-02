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

int btnInput = 0,
  distanceF = 0,
  distanceL = 0,
  distanceR = 0,
  distance = 0,
  mode = 0;

char direction = 0,
  oldDirection = 0;

bool isTurnServo = true;

void setup() {
  // put your setup code here, to run once:
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MODE_PIN, OUTPUT);
  pinMode(F_LIGHTS, OUTPUT);
  pinMode(B_LIGHTS, OUTPUT);

  motor1.setSpeed(MOTOR_SPEED);
  motor1.run(RELEASE);

  motor2.setSpeed(MOTOR_SPEED);
  motor2.run(RELEASE);

  motor3.setSpeed(MOTOR_SPEED);
  motor3.run(RELEASE);

  motor4.setSpeed(MOTOR_SPEED);
  motor4.run(RELEASE);

  servo.attach(SERVO_PIN);

  hc06.begin(9600);

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
  digitalWrite(F_LIGHTS, LOW);
  digitalWrite(B_LIGHTS, LOW);
  digitalWrite(MODE_PIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(F_LIGHTS, HIGH);

  if (mode == 0) {
    controllerMode();
    digitalWrite(MODE_PIN, HIGH);
  } else {
    digitalWrite(MODE_PIN, LOW);
    freeMode();
  }
}

void controllerMode() {
  if (hc06.available()) {
    direction = hc06.read();
    if (direction == '8') {
      drawEight();
    }
    stop();
    if (direction == 'M') {
      changeMode();
    } else if (direction == 'F') {
      moveForward();
    } else if (direction == 'B') {
      moveBackward();
    } else if (direction == 'R') {
      turnRight();
    } else if (direction == 'L') {
      turnLeft();
    } else if (direction == 'E') {
      forwardRight();
    } else if (direction == 'C') {
      backwardRight();
    } else if (direction == 'Q') {
      forwardLeft();
    } else if (direction == 'Z') {
      backwardLeft();
    } else if (direction == 'S') {
      stop();
    }
  }
}

void changeMode() {
  mode = (mode + 1) % 2;
  isTurnServo = !isTurnServo;
  delay(300);
  stop();
  delay(100);
}

void freeMode() {
  distanceF = 0;
  distanceL = 0;
  distanceR = 0;

  if (hc06.available()) {
    direction = hc06.read();
    if (direction == 'M') {
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

  servo.write(LEFT_ANGLE);
  delay(500);
  distanceL = (readUltrasonicDuration(TRIG_PIN, ECHO_PIN) / 2) * 0.0343;
  delay(100);

  servo.write(RIGHT_ANGLE);
  delay(500);
  distanceR = (readUltrasonicDuration(TRIG_PIN, ECHO_PIN) / 2) * 0.0343;
  delay(100);

  bool cancel = distanceF <= MIN_DIST || distanceR <= MIN_DIST || distanceL <= MIN_DIST;

  if (cancel) return;

  forwardLeft();
  delay(2300);
  stop();
  delay(150);
  stop();
  forwardRight();
  delay(2500);
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
