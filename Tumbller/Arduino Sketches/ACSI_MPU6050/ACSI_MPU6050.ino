// #include <Wire.h>  
#include "MPU6050.h"
#include "Balanced.h"
// #include "KalmanFilter.h"

// const int MPUaddr = 0x68; // unused relic from GY521
MPU6050 mpu6050; 
MyMPU mympu(mpu6050);
Balanced balanced(mympu);


void setup() {
  // Wire.begin();
  Serial.begin(9600);
  delay(100);
  Serial.println("Initialization");
}

void loop() {
  // int16_t ax, ay, az, gx, gy, gz;
  // mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // // Convert raw accelerometer values to m/s²
  // float ax_mps2 = (ax / 16384.0) * 9.81;
  // float ay_mps2 = (ay / 16384.0) * 9.81;
  // float az_mps2 = (az / 16384.0) * 9.81;

  // Serial.print("Accelerometer: ");
  // Serial.print("X = "); Serial.print(ax_mps2);
  // Serial.print(" m/s² | Y = "); Serial.print(ay_mps2);
  // Serial.print(" m/s² | Z = "); Serial.println(az_mps2);
  // Serial.println(" m/s²");

  // Serial.print("Gyroscope: ");
  // Serial.print("X = "); Serial.print(gx);
  // Serial.print(" | Y = "); Serial.print(gy);
  // Serial.print(" | Z = "); Serial.println(gz);
  // Serial.println(" ");

  delay(333);


  // balanced.Get_EncoderSpeed();
  // mpu.DataProcessing();
  // mpu.readIMU();
  // balanced.PD_VerticalRing();
  // balanced.Total_Control();

  // delay(5);
}