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

float lastDistVal[] = {0,0,0}

void setup() {
  AFMS.begin();
  myMotor->setSpeed(150);
  myMotor2->setSpeed(150);
  // Initialize ultrasonic sensor pins
  initSensors();

}

void loop() {
  myMotor->run(FORWARD);
  myMotor2->run(FORWARD);

  // put your main code here, to run repeatedly:
  // Read ultrasonic sensor measurements

  //printSensorData(d1_meas);
  Serial.println((d1_meas));
  if ((d1_meas) < 6){
    turnRight();
  }
else {
  myMotor->run(FORWARD);
  myMotor2->run(FORWARD);
}  

  d1_meas = readUltrasonic(trigPin1, echoPin1);
  float d2_meas = readUltrasonic(trigPin2, echoPin2);
  float d3_meas = readUltrasonic(trigPin3, echoPin3);

  getIMU();
  
  // send the sensor data over i2c as an array of bytes
  uint8_t sensor_data[] = {static_cast<uint8_t>(d1_meas), static_cast<uint8_t>(d2_meas), static_cast<uint8_t>(d3_meas)};
  Wire.beginTransmission(8); // transmit to device with address 8
  Wire.write(sensor_data, sizeof(sensor_data)); // sends sensor data
  Wire.endTransmission(); // stop transmitting
  //z << d1_meas, d2_meas, d3_meas;
  //delay(2000);


  lastDistVal[0] = d1_meas
  lastDistVal[1] = d2_meas
  lastDistVal[2] = d3_meas
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

void printData(float x, float y, float z, float gx, float gy, float gz){
  // Serial.print("Ax: ");
  //   Serial.println(x);
  //   Serial.print("Ay: ");
  //   Serial.println(y);
  //   Serial.print("Az: ");
  //   Serial.println(z);
    Serial.print("Gx: ");
    Serial.println(gx);
    Serial.print("Gy: ");
    Serial.println(gy);
    Serial.print("Gz: ");
    Serial.println(gz);
    Serial.println("");
    delay(3000);
     
}
void printSensorData(float sensorData){
  Serial.print("Sensor: ");
  Serial.println(sensorData);
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

void avoidRight(){
  float meas = readUltrasonic(trigPin1, echoPin1);
  while (meas < 8){
    meas = readUltrasonic(trigPin1, echoPin1);
    turnRight();
  }
  delay(1000);
  myMotor->run(FORWARD);
  myMotor2->run(FORWARD);

}


