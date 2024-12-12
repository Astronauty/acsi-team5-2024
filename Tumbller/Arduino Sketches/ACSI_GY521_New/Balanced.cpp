#include "Balanced.h"
#include <Wire.h>
#include "Motor.h"
#include <Arduino.h>
#include "KalmanFilter.h"
#include "MsTimer2.h"

KalmanFilter kalmanfilter;
Motor motor;

Balanced::Balanced(MPU6050& mpu):mpu6050(mpu)
{
  Serial.println("Building Balanced object");
  kp_balance = 55, kd_balance = 0.75;  
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
    // mpu6050.DataProcessing();
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
  float angle = kalmanfilter.angle;
  float setAngle = 0;
  float setAngleDot = -1.0;
  float contribution_p = kp_balance * (angle - setAngle);
  float contribution_d = kd_balance * (kalmanfilter.Gyro_x - setAngleDot); // change this

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

