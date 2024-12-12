#ifndef _BALANCED_h
#define _BALANCED_h

// #include "MsTimer2.h"
#include <Arduino.h>
#include "KalmanFilter.h"
// #include "SparkFunLSM6DS3.h"

enum Direction
{
  FORWARD,
  BACK,
  LEFT,
  RIGHT,
  STOP,
};

class Mpu6050
{
  public:
        void init();
        void DataProcessing(); // readIMU
        void PrintData();
        // void calibrateIMU(unsigned long delayMillis, unsigned long calibrationMillis);
        // bool readIMU();
        // void doCalculations();
        Mpu6050();

  public:
        float ax, ay, az, gx, gy, gz;
        float dt, Q_angle, Q_gyro, R_angle, C_0, K1;
        float gyroDriftX, gyroDriftY, gyroDriftZ;
        float gyroRoll, gyroPitch, gyroYaw;
        float gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw;
        float accRoll, accPitch, accYaw;
        float complementaryRoll, complementaryPitch, complementaryYaw;
        unsigned long lastTime;
        unsigned long lastInterval;
        // LSM6DS3 myIMU; // SparkFun
};

class Balanced
{
  public:
          Balanced(Mpu6050& mpu);
          void Get_EncoderSpeed();
          void PD_VerticalRing();
          void PI_SpeedRing();
          void PI_SteeringRing();
          void Total_Control();

          void Motion_Control(Direction direction);
          void Stop();
          void Forward(int speed);
          void Back(int speed);
          void Left(int speed);
          void Right(int speed);
  
/*Speed value*/
          double pwm_left;
          double pwm_right;
          int encoder_left_pulse_num_speed;
          int encoder_right_pulse_num_speed;
/*Cnt*/
          int interrupt_cnt;

/*PID parameter*/
         /*PD_VerticalRing*/
          double kp_balance, kd_balance;
         /*PI_SpeedRing*/
          double kp_speed, ki_speed;
         /*PI_SteeringRing*/
          double kp_turn, kd_turn;

          double speed_filter;
          double speed_filter_old;
          double car_speed_integeral;
          double balance_control_output;
          double speed_control_output;
          double rotation_control_output;
          int setting_turn_speed;
          int setting_car_speed;
          float delta_t; 
          float prev_angle;
          
   private:
          Mpu6050& mpu6050; // Add this line
   #define ANGLE_MIN -32 // -27
   #define ANGLE_MAX 32 // 27
   #define EXCESSIVE_ANGLE_TILT (kalmanfilter.angle < ANGLE_MIN || ANGLE_MAX < kalmanfilter.angle)
   #define PICKED_UP 0 //(kalmanfilter.angle6 < -10 || 22 < kalmanfilter.angle6)
};

// class Timer2
// {
//   public:
//           void init(int time);
//           static void interrupt();
//   private:       
//           #define TIMER 5
// };


#endif
