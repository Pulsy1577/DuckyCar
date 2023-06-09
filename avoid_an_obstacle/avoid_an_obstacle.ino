// These must be defined before including TinyEKF.h
#define Nsta 2     // Two state values: pressure, temperature
#define Mobs 3     // Three measurements: baro pressure, baro temperature, LM35 temperature

#define LM35_PIN 0
#include <Wire.h> // for I2C communication
#include "ICM_20948.h"
#include "TinyEKF.h"
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the lArduinast bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1
const int sensor1_address = 8; // I2C address of sensor 1
const int sensor2_address = 9; // I2C address of sensor 2
const int sensor3_address = 10; // I2C address of sensor 3
const int trigPin1 = 3; //Front
const int echoPin1 = 2; //Front
const int trigPin2 = 5; //Left
const int echoPin2 = 4; //Left
const int trigPin3 = 7; //Right
const int echoPin3 = 6; //Right

ICM_20948_I2C myICM;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
float d1_meas = 100;
float d2_meas = 0;
float heading = 0;
float curr_dir = 0;

float lastDistVal[] = {0,0,0};

unsigned long sensor_timer;

void setup() {
  
  AFMS.begin();
  myMotor->setSpeed(90);
  myMotor2->setSpeed(90);
  // Initialize ultrasonic sensor pins

  initSensors();

}

void loop() {

  ICM_20948_I2C *sensor = &myICM;

  float z = sensor->gyrZ(); 

  float delta_s = (millis() - sensor_timer);
  heading += z * (delta_s/1000);
  sensor_timer = millis();

  // myMotor->run(FORWARD);
  // myMotor2->run(FORWARD);
  
  d1_meas = readUltrasonic(trigPin1, echoPin1);
  float d2_meas = readUltrasonic(trigPin2, echoPin2);
  float d3_meas = readUltrasonic(trigPin3, echoPin3);

  // Serial.println(d1_meas);
  // if ((d1_meas) < 6){
  //   turnLeft90();
  // }

  getIMU();

  // send the sensor data over i2c as an array of bytes
  uint8_t sensor_data[] = {static_cast<uint8_t>(d1_meas), static_cast<uint8_t>(d2_meas), static_cast<uint8_t>(d3_meas)};
  Wire.beginTransmission(8); // transmit to device with address 8
  Wire.write(sensor_data, sizeof(sensor_data)); // sends sensor data
  Wire.endTransmission(); // stop transmitting
  //z << d1_meas, d2_meas, d3_meas;
  //delay(2000);

  Serial.print("Curr dist ");
  Serial.println(d2_meas);
  Serial.print("last dist ");
  Serial.println(lastDistVal[1]);
  detectPath(d2_meas, lastDistVal[1]);

  lastDistVal[0] = d1_meas;
  lastDistVal[1] = d2_meas;
  lastDistVal[2] = d3_meas;

  

}

//End loop 

void getIMU(){
  if(myICM.dataReady()){
    myICM.getAGMT();

    ICM_20948_I2C *sensor = &myICM;
    float x = sensor->accX();
    float y = sensor->accY();
    float z = sensor->accZ();
    float gx = sensor->gyrX();
    float gy = sensor->gyrY();
    float gz = sensor->gyrZ();       
    //printData(x, y, z, gx, gy, gz);
  } 
}


void initSensors(){
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  sensor_timer = millis();
  myICM.begin(WIRE_PORT, AD0_VAL);
  Serial.begin(9600);  
}

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) / 58.2;
}

void stop(){
  Serial.println("Stoping");
  myMotor->setSpeed(0);
  myMotor2->setSpeed(0);
}

void turnRight(){
  Serial.println("Turning Right");
  myMotor->run(BACKWARD);
  myMotor2->run(FORWARD);
}

void turnLeft(){
  Serial.println("Turning Left");
  myMotor->run(FORWARD);
  myMotor2->run(BACKWARD);
}


int turnLeft90(){

  float target = heading - 90;

  myMotor->run(FORWARD);
  myMotor2->run(BACKWARD);

  while (heading >= target){
    ICM_20948_I2C *sensor = &myICM;
    float z = sensor->gyrZ(); 
    float delta_s = (millis() - sensor_timer);
    Serial.print("Delt s: ");
    Serial.println(delta_s);    
    heading += z * (delta_s/1000);
    sensor_timer = millis();
    Serial.println("Turning Left");
    Serial.print("Heading: ");
    Serial.println(heading);
    Serial.print("Target: ");
    Serial.println(target);
    delay(100);
  }
  return 0;
  
}

void correctAngle(){
  
}

int detectPath(float currDist,float lastDist) {
  if (currDist > 20 && lastDist > 20 ) {
    //Serial.println("Path detected");
    return 0;
  }
  return 1;
}


void avoidRight(){
  float meas = readUltrasonic(trigPin1, echoPin1);
  while (meas < 8){
    meas = readUltrasonic(trigPin1, echoPin1);
    turnRight();
  }

}


