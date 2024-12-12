#include "Balanced.h"
#include <Wire.h>
#include "Motor.h"
#include <Arduino.h>
// #include "MPU6050_6Axis_MotionApps20.h"
#include "KalmanFilter.h"
#include <Arduino_LSM6DS3.h>

KalmanFilter kalmanfilter;
Motor motor;
// void Timer2::init(int time)
// {
//   MsTimer2::set(time,interrupt);
//   MsTimer2::start();
// }

// static void Timer2::interrupt()
// { 
//   sei();//enable the global interrupt // noInterrupts();
//   Balanced.Get_EncoderSpeed();
//   MyMPU.DataProcessing();
//   Balanced.PD_VerticalRing();
//   Balanced.interrupt_cnt++;
//   if(B b alanced.interrupt_cnt > 8)
//   {
//     Balanced.interrupt_cnt=0;
//     Balanced.PI_SpeedRing();
//     Balanced.PI_SteeringRing();
//    }
//   Balanced.Total_Control();
// }

Balanced::Balanced(MyMPU& mpu):mympu(mpu)
{
  // kp_balance = 55, kd_balance = 0.75;
  // tested:
  // (25, 1.5)  
  // (155,5)
  // (455,5) - large oscillations grow in magnitude
  // (55,5)
  // (85,5)
  // (55,25)
  // (85, 150)
  // (85, 350) - reduces overshoot, compensates a bit too much, opposite K_p slow
  // (185, 170) - quickly cuts off overshoot, but does nothing after first rebound
  // when kd_balance is >40, there is steady state bias towards moving backward
  kp_balance = 50, kd_balance = 1;  
  kp_speed = 10, ki_speed = 0.26;
  kp_turn = 2.5, kd_turn = 0.5;
}

void Balanced::Total_Control()
{
  pwm_left = balance_control_output - speed_control_output - rotation_control_output;//Superposition of Vertical Velocity Steering Ring
  pwm_right = balance_control_output - speed_control_output + rotation_control_output;//Superposition of Vertical Velocity Steering Ring
  
  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);
  
   while(EXCESSIVE_ANGLE_TILT || PICKED_UP)
  { 
    mympu.DataProcessing();
    // mpu6050.readIMU();
    motor.Stop();
  }
  
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
  // float angle_dot = kalmanfilter.Gyro_x; // default
  // angle_dot = (kalmanfilter.angle - prev_angle)/(millis() - delta_t);
  // prev_angle = kalmanfilter.angle;
  // delta_t = millis();
  float angle = kalmanfilter.angle;
  // angle = mpu6050.complementaryRoll;
  balance_control_output= kp_balance * (angle - 0) +  kd_balance * (kalmanfilter.Gyro_x - 0);
  // balance_control_output= kp_balance * (kalmanfilter.angle6 - 0) + kd_balance * (kalmanfilter.Gyro_x - 0);
}

void Balanced::PI_SteeringRing()
{  
   rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;////control with Z-axis gyroscope
}


MyMPU::MyMPU(MPU6050& mpu6050) : mpu6050(mpu6050) // Update constructor
{
  // dt is time b/t samples, so 104 Hz or 0.0096 sec. (original: 0.005)
  // process covariance goes up Q, to trust sensor more
  // measurement covariance has to go down R, 
  // dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
  // factor of 10 for Q_angle, Q_ gyro, R_angle, and K makes it faster
  // K is measure of ratio of using new sensor, but YiOrderFilter is not used
  dt = 0.005, Q_angle = 0.01, Q_gyro = 0.05, R_angle = 0.05, C_0 = 1, K1 = 0.05;
  Serial.println("MPU6050 initialization");
  mpu6050.initialize();
  if (!mpu6050.testConnection()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);
  }
  Serial.println("MPU6050 initialized successfully");
}

void MyMPU::init()
{
  // do
}

void MyMPU::PrintData()
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

// Kalman filter implementation
void MyMPU::DataProcessing()
{ 
  int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
  mpu6050.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
  
  // Convert rraw_aw raw_accelerometer vraw_alues to m/s²
  float ax = (raw_ax / 16384.0) * 9.81;
  float ay = (raw_ay / 16384.0) * 9.81;
  float az = (raw_az / 16384.0) * 9.81;

  float gx = (raw_gx - (-400.0)) / 131.0;
  float gy = (raw_gy - (80)) / 131.0;
  float gz = (raw_gz - (-210)) / 131.0;

  // kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);// Obtaining Angle by Kalman Filter
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);// Obtaining Angle by Kalman Filter
  Serial.print("Angle6: ");
  Serial.print(kalmanfilter.angle6);
  Serial.print("\t");
  Serial.print("KF Angle: ");
  Serial.println(kalmanfilter.angle);
}


// void MyMPU::calibrateIMU(unsigned long delayMillis, unsigned long calibrationMillis)
// {
//   Serial.println("Begin calibration");
//   int calibrationCount = 0;

//   delay(delayMillis);
//   Serial.println("waited for delayMillis");

//   float sumX = 0, sumY = 0, sumZ = 0;
//   Serial.println("calling startTime");
//   unsigned long startTime = millis();
//   Serial.println(startTime);
//   while (millis() < startTime + calibrationMillis) {
//     if (readIMU()) {
//       sumX += gx;
//       sumY += gy;
//       sumZ += gz;
//       calibrationCount++;
//     }
//   }

//   if (calibrationCount == 0) {
//     Serial.println("Failed to calibrate");
//   }

//   gyroDriftX = sumX / calibrationCount;
//   gyroDriftY = sumY / calibrationCount;
//   gyroDriftZ = sumZ / calibrationCount;
//   Serial.print("Calibration done");
// }

// bool MyMPU::readIMU()
// {
//   Serial.println("Reading IMU"); 
//   // if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
//   Serial.println("IMU data available");
//   IMU.readAcceleration(ax, ay, az);
//   Serial.println("IMU data available");
//   IMU.readGyroscope(gx, gy, gz);
//   Serial.println("IMU data available");
//   #define CONVERT_gTO_MS2 9.80665f
//   ax *= CONVERT_gTO_MS2;
//   ay *= CONVERT_gTO_MS2;
//   az *= CONVERT_gTO_MS2;
//   doCalculations();
//   unsigned long currentTime = micros();
//   lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
//   lastTime = currentTime;
//   return true;
//   return false;
// }

// // complementaryRoll determines vertical angle, replace kalmanfilter.angle
// void MyMPU::doCalculations() // replaces DataProcessing
// {
//   accRoll = atan2(-ay, az) * 180 / M_PI;
//   accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;

//   float lastFrequency = (float) 1000000.0 / lastInterval;
//   gyroRoll = gyroRoll + (gx / lastFrequency);
//   gyroPitch = gyroPitch + (gy / lastFrequency);
//   gyroYaw = gyroYaw + (gz / lastFrequency);

//   gyroCorrectedRoll = gyroCorrectedRoll + ((gx - gyroDriftX) / lastFrequency);
//   gyroCorrectedPitch = gyroCorrectedPitch + ((gy - gyroDriftY) / lastFrequency);
//   gyroCorrectedYaw = gyroCorrectedYaw + ((gz - gyroDriftZ) / lastFrequency);

//   complementaryRoll = complementaryRoll + ((gx - gyroDriftX) / lastFrequency);
//   complementaryPitch = complementaryPitch + ((gy - gyroDriftY) / lastFrequency);
//   complementaryYaw = complementaryYaw + ((gz - gyroDriftZ) / lastFrequency);

//   complementaryRoll = 0.98 * complementaryRoll + 0.02 * accRoll;
//   complementaryPitch = 0.98 * complementaryPitch + 0.02 * accPitch;
// }