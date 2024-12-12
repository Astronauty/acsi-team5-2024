#include <stdio.h>
#include "madgwickFilter.h"
#include <Wire.h>
#include "GY521.h"

const int MPU = 0x68; 
GY521 sensor(MPU);
// KalmanFilter filter;

#define DELTA_T 0.005f

float ax, ay, az, gx, gy, gz;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  if (!sensor.begin()) {
    Serial.println("Failed to initialize GY521 sensor!");
    while (1);
  }

  if (!sensor.calibrate(250)) {
    Serial.println("Calibration failed!");
    while (1);
  }
}

void loop() {
  if (sensor.read() != GY521_OK) {
    Serial.println("Failed to read sensor data!");
    delay(1000);
    return;
  }

  ax = 9.81* sensor.getAccelX();
  ay = 9.81* sensor.getAccelY();
  az = 9.81* sensor.getAccelZ();
  gx = sensor.getGyroX();
  gy = sensor.getGyroY();
  gz = sensor.getGyroZ();

  // Serial.print("Accelerometer: ");
  // Serial.print("X = "); Serial.print(ax);
  // Serial.print(" | Y = "); Serial.print(ay);
  // Serial.print(" | Z = "); Serial.println(az);

  // Serial.print("Gyroscope: ");
  // Serial.print("X = "); Serial.print(gx);
  // Serial.print(" | Y = "); Serial.print(gy);
  // Serial.print(" | Z = "); Serial.println(gz);
  // Serial.println(" ");


  // flat upright position to resolve the graident descent problem
  float roll = 0.0, pitch = 0.0, yaw = 0.0;

  imu_filter(ax, ay, az, gx, gy, gz);
  eulerAngles(q_est, &roll, &pitch, &yaw);
  Serial.print(", roll: ");
  Serial.print(roll);
  Serial.print(", pitch: ");
  Serial.print(pitch);
  Serial.print(", yaw: ");
  Serial.println(yaw);
  
  // }  
    

  delay(5);
}


// #include <Wire.h>
// #include "GY521.h"
// #include "Balanced.h"

// const int MPU = 0x68; 
// GY521 sensor(MPU);
// Mpu6050 mpu(sensor);
// Balanced balanced(mpu);

// void setup() {
//   Wire.begin();
//   Serial.begin(9600);
//   mpu.init();
//   Serial.print("Calibrating IMU...");
//   mpu.calibrateIMU(500, 500);
// }

// void loop() {
//   balanced.Get_EncoderSpeed();
//   // mpu.DataProcessing();
//   mpu.readIMU();
//   balanced.PD_VerticalRing();
//   // balanced.PI_SpeedRing();
//   // balanced.PI_SteeringRing();
//   balanced.Total_Control();

//   delay(5);
// }