// These must be defined before including TinyEKF.h
#define Nsta 2     // Two state values: pressure, temperature
#define Mobs 6     // Three measurements: baro pressure, baro temperature, LM35 temperature

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
//const int sensor1_address = 8; // I2C address of sensor 1
//const int sensor2_address = 9; // I2C address of sensor 2
//const int sensor3_address = 10; // I2C address of sensor 3
const int trigPin1 = 3; //Front
const int echoPin1 = 2; //Front
const int trigPin2 = 5; //Left
const int echoPin2 = 4; //Left
const int trigPin3 = 7; //Right
const int echoPin3 = 6; //Right
ICM_20948_I2C myICM;
char ins[] = {'r', 'l', 'r', 'l', 'r'};
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
float d1_meas = 100;
//float heading = 0;

//float lastDistVal[] = {0,0,0};

unsigned long sensor_timer;

class Filter : public TinyEKF {
  public:
      Filter()
      {   
          // approximate the process noise                  
          this->setQ(0, 0, .0001);
          this->setQ(1, 1, .0001);
          // same for measurement noise
          this->setR(0, 0, .0001);
          this->setR(1, 1, .0001);
          this->setR(2, 2, .0001);
          this->setR(3, 3, .0001);
          this->setR(4, 4, .0001);
          this->setR(5, 5, .0001);
      }
  protected:
      void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
      {
      // Process model: x(k) = f(x(k-1))
      fx[0] = this->x[0]; //+ this->x[1]; // heading update
      fx[1] = this->x[1]; // angular velocity update

      // State transition matrix: F(k) = df(x(k))/dx(k-1)
      F[0][0] = 1; 
      F[1][1] = 1;

      // Observation model: z(k) = h(x(k))
      hx[0] = this->x[0]; // accelerationX measurement
      hx[1] = this->x[1]; // accelerationY measurement
      hx[2] = this->x[2]; // accelerationZ measurement
      hx[3] = this->x[3]; // gyroscopeX measurement
      hx[4] = this->x[4]; // gyroscopeY measurement
      hx[5] = this->x[5]; // gyroscopeZ measurement

      // Observation matrix: H(k) = dh(x(k))/dx(k)
      H[0][0] = 1;
      H[1][1] = 1;
      H[2][2] = 1;
      H[3][3] = 1;
      H[4][4] = 1;
      H[5][5] = 1;
      }
};

Filter ekf;
void setup() {
  
  AFMS.begin();
  rightMotor->setSpeed(150);
  leftMotor->setSpeed(150);
  // Initialize ultrasonic sensor pins

  initSensors();
  ekf.setX(0, 0);

}

void loop() {
  // put your main code here, to run repeatedly:
  // Read ultrasonic sensor measurements

  //printSensorData(d1_meas);
  //Serial.println((d1_meas));
  //obstacle avoidance
  //move();

  // getIMU();
  myICM.getAGMT();

  ICM_20948_I2C *sensor = &myICM;
  double x = sensor->accX();
  double y = sensor->accY();
  double z = sensor->accZ();
  double gx = sensor->gyrX();
  double gy = sensor->gyrY();
  double gz = sensor->gyrZ();

  double sh[6] = {x, y, z, gx, gy, gz};

  ekf.step(sh);

  Serial.print(sh[0]);
  Serial.print(" accX | ");
  Serial.print(sh[1]);
  Serial.print(" accY | ");
  Serial.print(sh[2]);
  Serial.print(" accZ | ");
  Serial.print(sh[3]);
  Serial.print(" gX | ");
  Serial.print(sh[4]);
  Serial.print(" gY | ");
  Serial.print(sh[5]);
  Serial.println(" gZ | ");
  Serial.print(ekf.getX(0));
  Serial.print(" heading | ");
  Serial.print(ekf.getX(1));
  Serial.print(" angular velocity | ");
  Serial.println("");
  Serial.println("");

}

//End loop 

void getIMU(){
  if(myICM.dataReady()){
    myICM.getAGMT();

    ICM_20948_I2C *sensor = &myICM;
    double x = sensor->accX();
    double y = sensor->accY();
    double z = sensor->accZ();
    double gx = sensor->gyrX();
    double gy = sensor->gyrY();
    double gz = sensor->gyrZ();

    // Report measured and predicte/fused values
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
int i = 0;
double frontSensor = 100;
void move(){
  forward();
  frontSensor = readUltrasonic(trigPin1, echoPin1);
  //Serial.println(frontSensor);
  if(frontSensor < 10){
    Serial.println(frontSensor);
    Serial.println("Detected");
    //stop();
    turnedRight90();
    /*if(checkRight() && ins[i] == 'r'){
      turnedRight90();
      Serial.println("Supposedly turned right");
    }else if(checkLeft() && ins[i] == 'l'){
      turnLeft90();
    }*/
  }
}
void stop(){
  Serial.println("Stoping");
  rightMotor->setSpeed(0);
  leftMotor->setSpeed(0);
}
void forward(){
  rightMotor->run(FORWARD);
  leftMotor->run(FORWARD);
}
void turnRight(){
  Serial.println("Turning Right");
  rightMotor->run(BACKWARD);
  leftMotor->run(FORWARD);
}
void turnLeft(){
  Serial.println("Turning Left");
  rightMotor->run(FORWARD);
  leftMotor->run(BACKWARD);
}
bool checkRight(){
  if(readUltrasonic(trigPin3, echoPin3) > 10){
    return true;
  }
  else{
    return false;
  }
}

bool checkLeft(){
  if(readUltrasonic(trigPin2, echoPin2) > 10){
    return true;
  }
  else{
    return false;
  }
}

int turnRight90(){
  float heading = 0;
  float target = 0 - 90;
  turnRight();

  while (heading > target){
    ICM_20948_I2C *sensor = &myICM;
    float z = sensor->gyrZ(); 
    float delta_s = (millis() - sensor_timer);
    Serial.print("Delt s: ");
    Serial.println(delta_s);    
    heading += z * (delta_s/1000);
    sensor_timer = millis();
    Serial.println("Turning Right");
    Serial.print("Heading: ");
    Serial.println(heading);
    Serial.print("Target: ");
    Serial.println(target);
    delay(100);
  }
  //forward();
  return 0;
  
}
bool turnedRight90(){
  float heading = 0;
  float target = heading - 90;
  turnRight();

  while (heading > target){
    ICM_20948_I2C *sensor = &myICM;
    float z = sensor->gyrZ(); 
    float delta_s = (millis() - sensor_timer);
    Serial.print("Delta s: ");
    Serial.println(delta_s);    
    heading += z * (delta_s/1000);
    sensor_timer = millis();
    Serial.println("Turning Right");
    Serial.print("Heading: ");
    Serial.println(heading);
    Serial.print("Target: ");
    Serial.println(target);
    delay(100);
  }
  
  // Stop the car
  stop();

  // Reset the heading to zero degrees
  heading = 0;

  return true;
}
int turnLeft90(){
  float heading = 0;
  float target = heading - 90;
  turnLeft();

  while (heading + 10 > target && heading - 10 > target){
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
  forward();
  return 0;
  
}
void avoidRight(){
  float meas = readUltrasonic(trigPin1, echoPin1);
  while (meas < 10){
    meas = readUltrasonic(trigPin1, echoPin1);
    turnRight();
  }
}