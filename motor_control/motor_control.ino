#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);

void setup() {
  AFMS.begin();
  Serial.begin(9600);
  myMotor->setSpeed(150);
  myMotor2->setSpeed(150);
}

void loop() {

  char cmd = Serial.read();
  if(cmd == 's'){
    myMotor->run(FORWARD);
    myMotor2->run(FORWARD);
    delay(5000);
    myMotor->setSpeed(0);
    myMotor2->setSpeed(0);
  }

}

void right() {
  myMotor->run(FORWARD);
  myMotor2->run(BACKWARD);
}

void left() {
  myMotor2->run(FORWARD);
  myMotor->run(BACKWARD);
}
