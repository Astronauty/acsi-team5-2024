// #include <Wire.h>
// #include "GY521.h"

// const int MPU = 0x68; 
// GY521 sensor(MPU);
// // KalmanFilter filter;


// void setup() {
//   Wire.begin();
//   Serial.begin(9600);

//   if (!sensor.begin()) {
//     Serial.println("Failed to initialize GY521 sensor!");
//     while (1);
//   }

//   if (!sensor.calibrate(100)) {
//     Serial.println("Calibration failed!");
//     while (1);
//   }
// }

// void loop() {
//   if (sensor.read() != GY521_OK) {
//     Serial.println("Failed to read sensor data!");
//     delay(1000);
//     return;
//   }
//   Serial.print("Accelerometer: ");
//   Serial.print("X = "); Serial.print(sensor.getAccelX());
//   Serial.print(" | Y = "); Serial.print(sensor.getAccelY());
//   Serial.print(" | Z = "); Serial.println(sensor.getAccelZ());

//   Serial.print("Gyroscope: ");
//   Serial.print("X = "); Serial.print(sensor.getGyroX());
//   Serial.print(" | Y = "); Serial.print(sensor.getGyroY());
//   Serial.print(" | Z = "); Serial.println(sensor.getGyroZ());
//   Serial.println(" ");

//   delay(333);
// }


#include <Wire.h>
#include "GY521.h"
#include "Balanced.h"

const int MPU = 0x68; 
GY521 sensor(MPU);
Mpu6050 mpu(sensor);
Balanced balanced(mpu);

void setup() {
  Wire.begin();
  Serial.begin(9600);
  mpu.init();
  Serial.print("Calibrating IMU...");
  mpu.calibrateIMU(500, 500);
}


void loop() {
  balanced.Get_EncoderSpeed();
  // mpu.DataProcessing();
  mpu.readIMU();
  balanced.PD_VerticalRing();
  // balanced.PI_SpeedRing();
  // balanced.PI_SteringRing();
  balanced.Total_Control();

  delay(3);
}