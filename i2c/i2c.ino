// These must be defined before including TinyEKF.h
#define Nsta 5     // 5 state values: position in 3d (x, y, z), velocity in 3d (x, y,z), acceleration in 3d, orientation in 3d (pitch, roll, yaw), sensor bias
#define Mobs 4     // 4 measurements: 3 distance measurements, one measurement of acceleration and gryoscope data

#define LM35_PIN 0
#include <Wire.h> // for I2C communication
#include "ICM_20948.h"
#include "TinyEKF.h"
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the lArduinast bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1
const int sensor1_address = 8; // I2C address of sensor 1
const int sensor2_address = 9; // I2C address of sensor 2
const int sensor3_address = 10; // I2C address of sensor 3
const int trigPin1 = 3;
const int echoPin1 = 2;
const int trigPin2 = 5;
const int echoPin2 = 4;
const int trigPin3 = 7;
const int echoPin3 = 6;
float x, y, z, gx, gy, gz;
//double z[Mobs];
ICM_20948_I2C myICM;
class Filter : public TinyEKF {
    private:
      double dt; // time step
      double P[Nsta][Nsta]; // state covariance matrix
    public:
        Filter()
        {            
            // We set the initial state to zero
            this->x[0] = 0; // position x
            this->x[1] = 0; // position y
            this->x[2] = 0; // position z
            this->x[3] = 0; // velocity x
            this->x[4] = 0; // velocity y
            this->x[5] = 0; // velocity z
            this->x[6] = 0; // acceleration x
            this->x[7] = 0; // acceleration y
            this->x[8] = 0; // acceleration z
            this->x[9] = 0; // orientation pitch
            this->x[10] = 0; // orientation roll
            this->x[11] = 0; // orientation yaw
            this->x[12] = 0; // sensor bias

            // We set the initial state covariance to a high value to indicate high uncertainty
            for (int i = 0; i < Nsta; i++) {
              for (int j = 0; j < Nsta; j++) {
                if (i == j) {
                  this->P[i][j] = 100.0;
                } else {
                  this->P[i][j] = 0.0;
                }
              }
            }

            // We set the process noise to a small value to indicate low uncertainty
            this->setQ(0, 0, 0.01);
            this->setQ(1, 1, 0.01);
            this->setQ(2, 2, 0.01);
            this->setQ(3, 3, 0.01);
            this->setQ(4, 4, 0.01);
            this->setQ(5, 5, 0.01);
            this->setQ(6, 6, 0.001);
            this->setQ(7, 7, 0.001);
            this->setQ(8, 8, 0.001);
            this->setQ(9, 9, 0.001);
            this->setQ(10, 10, 0.001);
            this->setQ(11, 11, 0.001);
            this->setQ(12, 12, 0.001);

            // Same for measurement noise
            this->setR(0, 0, 0.1);
            this->setR(1, 1, 0.1);
            this->setR(2, 2, 0.1);
            this->setR(3, 3, 0.1);
        }
    protected:
    double z[Mobs];
    void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
    {
        // Process model is f(x) = x
        fx[0] = this->x[0] + this->x[3] * this->dt; // x position
        fx[1] = this->x[1] + this->x[4] * this->dt; // y position
        fx[2] = this->x[2] + this->x[5] * this->dt; // z position
        fx[3] = this->x[3] + this->x[6] * this->dt; // x velocity
        fx[4] = this->x[4] + this->x[7] * this->dt; // y velocity
        fx[5] = this->x[5] + this->x[8] * this->dt; // z velocity
        fx[6] = this->x[6]; // x acceleration
        fx[7] = this->x[7]; // y acceleration
        fx[8] = this->x[8]; // z acceleration
        fx[9] = this->x[9] + this->x[12] * this->dt; // pitch
        fx[10] = this->x[10] + this->x[13] * this->dt; // roll
        fx[11] = this->x[11] + this->x[14] * this->dt; // yaw
        fx[12] = this->x[12]; // pitch rate bias
        fx[13] = this->x[13]; // roll rate bias
        fx[14] = this->x[14]; // yaw rate bias

        // State transition Jacobian matrix
        F[0][0] = 1;
        F[0][3] = this->dt;
        F[1][1] = 1;
        F[1][4] = this->dt;
        F[2][2] = 1;
        F[2][5] = this->dt;
        F[3][3] = 1;
        F[3][6] = this->dt;
        F[4][4] = 1;
        F[4][7] = this->dt;
        F[5][5] = 1;
        F[5][8] = this->dt;
        F[6][6] = 1;
        F[7][7] = 1;
        F[8][8] = 1;
        F[9][9] = 1;
        F[9][12] = this->dt;
        F[10][10] = 1;
        F[10][13] = this->dt;
        F[11][11] = 1;
        F[11][14] = this->dt;
        F[12][12] = 1;
        F[13][13] = 1;
        F[14][14] = 1;

        // Measurement function hx = Hx
        hx[0] = this->z[0]; // sensor 1 distance measurement
        hx[1] = this->z[1]; // sensor 2 distance measurement
        hx[2] = this->z[2]; // sensor 3 distance measurement 
      }
};
Filter ekf;
void setup() {
  // put your setup code here, to run once:
  // Initialize ultrasonic sensor pins
  initSensors();
}

void loop() {
  // put your main code here, to run repeatedly:
  // Read ultrasonic sensor measurements
  float d1_meas = readUltrasonic(trigPin1, echoPin1);
  float d2_meas = readUltrasonic(trigPin2, echoPin2);
  float d3_meas = readUltrasonic(trigPin3, echoPin3); 
  getIMU();
  // Send these measurements to the EKF
  double z[3] = {d1_meas, d2_meas, d3_meas};
  ekf.step(z);
  Serial.print(z[0]);
  Serial.print(" ");
  Serial.print(z[1]);
  Serial.print(" ");
  Serial.print(z[2]);
  Serial.print(" ");
  Serial.print(ekf.getX(0));
  Serial.print(" ");
  Serial.println(ekf.getX(1));
  // send the sensor data over i2c as an array of bytes
  uint8_t sensor_data[] = {static_cast<uint8_t>(d1_meas), static_cast<uint8_t>(d2_meas), static_cast<uint8_t>(d3_meas)};
  Wire.beginTransmission(8); // transmit to device with address 8
  Wire.write(sensor_data, sizeof(sensor_data)); // sends sensor data
  Wire.endTransmission(); // stop transmitting
  //z << d1_meas, d2_meas, d3_meas;
  //delay(2000);
}
void getIMU(){
  if(myICM.dataReady()){
    myICM.getAGMT();

    ICM_20948_I2C *sensor = &myICM;
    x = sensor->accX();
    y = sensor->accY();
    z = sensor->accZ();
    gx = sensor->gyrX();
    gy = sensor->gyrY();
    gz = sensor->gyrZ();       
    printData(x, y, z, gx, gy, gz);
  } 
}
void printData(float x, float y, float z, float gx, float gy, float gz){
  Serial.print("Ax: ");
    Serial.println(x);
    Serial.print("Ay: ");
    Serial.println(y);
    Serial.print("Az: ");
    Serial.println(z);
    Serial.print("Gx: ");
    Serial.println(gx);
    Serial.print("Gy: ");
    Serial.println(gy);
    Serial.print("Gz: ");
    Serial.println(gz); 
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