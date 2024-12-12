#include "Balanced.h"
#include <Wire.h>
#include "Motor.h"
#include <Arduino.h>
#include "KalmanFilter.h"

KalmanFilter kalmanfilter;
Motor motor;

Balanced::Balanced(Mpu6050& mpu):mpu6050(mpu)
{
  kp_balance = 15, kd_balance = 0;  
  kp_speed = 10, ki_speed = 0.26;
  kp_turn = 2.5, kd_turn = 0.5;
}

void Balanced::Total_Control()
{
  pwm_left = balance_control_output - speed_control_output - rotation_control_output;
  pwm_right = balance_control_output - speed_control_output + rotation_control_output;
  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);
  
  while(EXCESSIVE_ANGLE_TILT || PICKED_UP)
  { 
    // mpu6050.DataProcessing();
    mpu6050.readIMU();
    motor.Stop();
  }
  // Serial.println("Total Control"); // this is running
  
  (pwm_left < 0) ?  (motor.Control(AIN1,1,PWMA_LEFT,-pwm_left)):
                    (motor.Control(AIN1,0,PWMA_LEFT,pwm_left));
  
  (pwm_right < 0) ? (motor.Control(BIN1,1,PWMB_RIGHT,-pwm_right)): 
                    (motor.Control(BIN1,0,PWMB_RIGHT,pwm_right));
}

void Balanced::Get_EncoderSpeed()
{
  encoder_left_pulse_num_speed += pwm_left < 0 ? (-Motor::encoder_count_left_a) : 
                                                  Motor::encoder_count_left_a;
  encoder_right_pulse_num_speed += pwm_right < 0 ? (-Motor::encoder_count_right_a) :
                                                  Motor::encoder_count_right_a;
  Motor::encoder_count_left_a=0;
  Motor::encoder_count_right_a=0;
}

void Balanced::Motion_Control(Direction direction)
{
  switch(direction)
  {
    case STOP:
                  Stop();break;
    case FORWARD:
                  Forward(40);break;
    case BACK:
                  Back(40);break;
    case LEFT:
                  Left(50);break;
    case RIGHT:
                  Right(50);break;
    default:      
                  Stop();break;
  }
}

void Balanced::Stop()
{
  setting_car_speed = 0;
  setting_turn_speed = 0;
}

void Balanced::Forward(int speed)
{
  setting_car_speed = speed;
  setting_turn_speed = 0;
}

void Balanced::Back(int speed)
{
  setting_car_speed = -speed;
  setting_turn_speed = 0;
}

void Balanced::Left(int speed)
{
  setting_car_speed = 0;
  setting_turn_speed = speed;
}

void Balanced::Right(int speed)
{
  setting_car_speed = 0;
  setting_turn_speed = -speed;
}

void Balanced::PI_SpeedRing()
{
   double car_speed=(encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
   encoder_left_pulse_num_speed = 0;
   encoder_right_pulse_num_speed = 0;
   speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
   speed_filter_old = speed_filter;
   car_speed_integeral += speed_filter;
   car_speed_integeral += -setting_car_speed; 
   car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);

   speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
}

void Balanced::PD_VerticalRing()
{
  float angle = kalmanfilter.angle;
  // mpu.
  float contribution_p = kp_balance * (angle - 0);
  float contribution_d = kd_balance * (kalmanfilter.Gyro_x - 0);

  // Serial.print("P: ");
  // Serial.print(contribution_p);
  // Serial.print("\t");
  // Serial.print("D: ");
  // Serial.print(contribution_d);
  // Serial.print("\t");
  
  balance_control_output= contribution_p +  contribution_d;
}

void Balanced::PI_SteeringRing()
{  
   rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;
}

Mpu6050::Mpu6050(GY521& sensor): sensor(sensor)
{
  dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
  // dt = 0.005, Q_angle = 0.015, Q_gyro = 0.075, R_angle = 0.05, C_0 = 1, K1 = 0.05; // LSM6DS3
}

void Mpu6050::init()
{
  Serial.println("Initializing IMU...");
  if (!sensor.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }   
  Serial.println("IMU initialized");
  if (!sensor.calibrate(100)) {
    Serial.println("Calibration failed!");
    while (1);
  }
  Serial.println("IMU calibrated");
  lastTime = micros();
}

void Mpu6050::PrintData()
{
  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
  Serial.print(",");
  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.println(gz);
}

void Mpu6050::DataProcessing()
{ 
  if (sensor.read() != GY521_OK) {
    Serial.println("Failed to read sensor data!");
    return;
  }
  ax = sensor.getAccelX();
  ay = sensor.getAccelY();
  az = sensor.getAccelZ();
  gx = sensor.getGyroX();
  gy = sensor.getGyroY();
  gz = sensor.getGyroZ();
  #define CONVERT_gTO_MS2 9.80665f
  ax *= CONVERT_gTO_MS2;
  ay *= CONVERT_gTO_MS2;
  az *= CONVERT_gTO_MS2;

  // kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  
  // Serial.println(" ");
  // Serial.print("Accelerometer: ");
  // Serial.print("X = "); Serial.print(ax);
  // Serial.print(" | Y = "); Serial.print(ay);
  // Serial.print(" | Z = "); Serial.println(az);

  // Serial.print("Gyroscope: ");
  // Serial.print("X = "); Serial.print(gx);
  // Serial.print(" | Y = "); Serial.print(gy);
  // Serial.print(" | Z = "); Serial.println(gz);

  Serial.println(" ");
  Serial.print("Angle6: ");
  Serial.print(kalmanfilter.angle6);
  Serial.print("\t");
  Serial.print("KF Angle: ");
  Serial.print(kalmanfilter.angle);
  Serial.print("\t");
  Serial.print("KF GyroX: ");
  Serial.print(kalmanfilter.Gyro_x);
  Serial.print("\t");
  Serial.print("gx: ");
  Serial.println (gx);
}

void Mpu6050::calibrateIMU(unsigned long delayMillis, unsigned long calibrationMillis)
{
  Serial.println("Begin calibration");
  int calibrationCount = 0;

  delay(delayMillis);
  Serial.println("waited for delayMillis");

  float sumX = 0, sumY = 0, sumZ = 0;
  Serial.println("calling startTime");
  unsigned long startTime = millis();
  Serial.println(startTime);
  while (millis() < startTime + calibrationMillis) {
    if (readIMU()) {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      calibrationCount++;
    }
  }

  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;
  Serial.print("Calibration done");
}

bool Mpu6050::readIMU()
{
  DataProcessing(); // ax, ay, az, gx, gy, gz
  doCalculations(); // this does the actual computation, we want complementaryRoll = angle
  // pass in the complementaryRoll and gx to kalmanfilter.Angle_New
  kalmanfilter.Angle_New(complementaryRoll, gx, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  unsigned long currentTime = micros();
  lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
  lastTime = currentTime;
  return true;
}


// // complementaryRoll determines vertical angle, replace kalmanfilter.angle
void Mpu6050::doCalculations() // replaces DataProcessing
{
  accRoll = atan2(ay, az) * 180 / M_PI;
  accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;

  float lastFrequency = (float) 1000000.0 / lastInterval;
  gyroRoll = gyroRoll + (gx / lastFrequency);
  gyroPitch = gyroPitch + (gy / lastFrequency);
  gyroYaw = gyroYaw + (gz / lastFrequency);

  gyroCorrectedRoll = gyroCorrectedRoll + ((gx - gyroDriftX) / lastFrequency);
  gyroCorrectedPitch = gyroCorrectedPitch + ((gy - gyroDriftY) / lastFrequency);
  gyroCorrectedYaw = gyroCorrectedYaw + ((gz - gyroDriftZ) / lastFrequency);

  complementaryRoll = complementaryRoll + ((gx - gyroDriftX) / lastFrequency);
  complementaryPitch = complementaryPitch + ((gy - gyroDriftY) / lastFrequency);
  complementaryYaw = complementaryYaw + ((gz - gyroDriftZ) / lastFrequency);

  // complementaryRoll = 0.98 * complementaryRoll + 0.02 * accRoll;
  // complementaryPitch = 0.98 * complementaryPitch + 0.02 * accPitch;
  complementaryRoll = 0.3 * complementaryRoll + 0.7 * accRoll;
  complementaryPitch = 0.3 * complementaryPitch + 0.7 * accPitch;
  
}