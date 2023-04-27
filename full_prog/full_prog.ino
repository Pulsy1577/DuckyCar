#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
int dirs = {1,0,1,0,1} //0 for left 1 for right
int heading = 0
void setup() {
  //Initialize everything

  //Set ekf
  
}

void loop() {
  //take readings
  //zero heading
  //send to EKF

  //start going forward
  //if heading is off then correct it

  //if an obstacle is in front:
    //turn right untill it doesnt see it, if it has turned 90 degrees then re center and try left
    //go forward, after a small delay the turn back to face "forward"

  //if it notices first path to right then take it
  //notices first path on left take 
  //more steps here

}

