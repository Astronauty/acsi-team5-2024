// Manual spinning begin
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include "Motor.h"
#include "Balanced.h"
#include "KalmanFilter.h"
#include "SAMDTimerInterrupt.h"

Mpu6050 mpu6050;
KalmanFilter kalmanfilter;
Balanced balanced();
Motor motor;
// MKRFOX 1200 (SAMS21) timer test - generate 100KHz square wave on pin 6
// from g https://github.com/khoih-prog/SAMD_TimerInterrupt

#if !(defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_SAMD_MKR1000) || defined(ARDUINO_SAMD_MKRWIFI1010) \
      || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_SAMD_MKRFox1200) || defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310) \
      || defined(ARDUINO_SAMD_MKRGSM1400) || defined(ARDUINO_SAMD_MKRNB1500) || defined(ARDUINO_SAMD_MKRVIDOR4000) \
      || defined(ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS) || defined(__SAMD51__) || defined(__SAMD51J20A__) \
      || defined(__SAMD51J19A__) || defined(__SAMD51G19A__) || defined(__SAMD51P19A__) \
      || defined(__SAMD21E15A__) || defined(__SAMD21E16A__) || defined(__SAMD21E17A__) || defined(__SAMD21E18A__) \
      || defined(__SAMD21G15A__) || defined(__SAMD21G16A__) || defined(__SAMD21G17A__) || defined(__SAMD21G18A__) \
      || defined(__SAMD21J15A__) || defined(__SAMD21J16A__) || defined(__SAMD21J17A__) || defined(__SAMD21J18A__))
#error This code is designed to run on SAMD21/SAMD51 platform! Please check your Tools->Board setting.
#endif

/////////////////////////////////////////////////////////////////

// These define's must be placed at the beginning before #include "SAMDTimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
// Don't define TIMER_INTERRUPT_DEBUG > 2. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG 0
#define _TIMERINTERRUPT_LOGLEVEL_ 0

// Select only one to be true for SAMD21. Must must be placed at the beginning before #include "SAMDTimerInterrupt.h"
#define USING_TIMER_TC3 true   // Only TC3 can be used for SAMD51
#define USING_TIMER_TC4 false  // Not to use with Servo library
#define USING_TIMER_TC5 false
#define USING_TIMER_TCC false
#define USING_TIMER_TCC1 false
#define USING_TIMER_TCC2 false  // Don't use this, can crash on some boards


// this is just for output checking
#define SQUARE_WAVE_PIN 6  // pin to output square wave // PWMB_RIGHT

#define TIMER_INTERVAL_US 5000 // timer interval in uSec, 9616µSec = 104Hz

#if (TIMER_INTERRUPT_USING_SAMD21)

#if USING_TIMER_TC3
#define SELECTED_TIMER TIMER_TC3
#elif USING_TIMER_TC4
#define SELECTED_TIMER TIMER_TC4
#elif USING_TIMER_TC5
#define SELECTED_TIMER TIMER_TC5
#elif USING_TIMER_TCC
#define SELECTED_TIMER TIMER_TCC
#elif USING_TIMER_TCC1
#define SELECTED_TIMER TIMER_TCC1
#elif USING_TIMER_TCC2
#define SELECTED_TIMER TIMER_TCC
#else
#error You have to select 1 Timer
#endif

#else

#if !(USING_TIMER_TC3)
#error You must select TC3 for SAMD51
#endif

#define SELECTED_TIMER TIMER_TC3

#endif

// Init selected SAMD timer
SAMDTimer ITimer(SELECTED_TIMER);
int counter = 0;
int sensor_freq = 0;



// Balanced::Balanced()
// {
//   // kp_balance = 55, kd_balance = 0.75;
//   // tested:
//   // (25, 1.5)  
//   // (155,5)
//   // (455,5) - large oscillations grow in magnitude
//   // (55,5)
//   kp_balance = 55, kd_balance = 1.5; 
//   kp_speed = 10, ki_speed = 0.26;
//   kp_turn = 2.5, kd_turn = 0.5;
// }

// void Balanced::Total_Control()
// {
//   pwm_left = balance_control_output - speed_control_output - rotation_control_output;//Superposition of Vertical Velocity Steering Ring
//   pwm_right = balance_control_output - speed_control_output + rotation_control_output;//Superposition of Vertical Velocity Steering Ring
//   pwm_left = -pwm_left; // onboard IMU is inverted
//   pwm_right = -pwm_right;

//   pwm_left = constrain(pwm_left, -255, 255);
//   pwm_right = constrain(pwm_right, -255, 255);

//    while(EXCESSIVE_ANGLE_TILT || PICKED_UP)
//   { 
//     mpu6050.DataProcessing();
//     motor.Stop();
//   }
  
//   (pwm_left < 0) ?  (motor.Control(AIN1,1,PWMA_LEFT,-pwm_left)):
//                     (motor.Control(AIN1,0,PWMA_LEFT,pwm_left));
  
//   (pwm_right < 0) ? (motor.Control(BIN1,1,PWMB_RIGHT,-pwm_right)): 
//                     (motor.Control(BIN1,0,PWMB_RIGHT,pwm_right));
// }

// void Balanced::Get_EncoderSpeed()
// {
//   encoder_left_pulse_num_speed += pwm_left < 0 ? (-Motor::encoder_count_left_a) : 
//                                                   Motor::encoder_count_left_a;
//   encoder_right_pulse_num_speed += pwm_right < 0 ? (-Motor::encoder_count_right_a) :
//                                                   Motor::encoder_count_right_a;
//   Motor::encoder_count_left_a=0;
//   Motor::encoder_count_right_a=0;
// }

// void Balanced::Motion_Control(Direction direction)
// {
//   switch(direction)
//   {
//     case STOP:
//                   Stop();break;
//     case FORWARD:
//                   Forward(40);break;
//     case BACK:
//                   Back(40);break;
//     case LEFT:
//                   Left(50);break;
//     case RIGHT:
//                   Right(50);break;
//     default:      
//                   Stop();break;
//   }
// }

// void Balanced::Stop()
// {
//   setting_car_speed = 0;
//   setting_turn_speed = 0;
// }

// void Balanced::Forward(int speed)
// {
//   setting_car_speed = speed;
//   setting_turn_speed = 0;
// }

// void Balanced::Back(int speed)
// {
//   setting_car_speed = -speed;
//   setting_turn_speed = 0;
// }

// void Balanced::Left(int speed)
// {
//   setting_car_speed = 0;
//   setting_turn_speed = speed;
// }

// void Balanced::Right(int speed)
// {
//   setting_car_speed = 0;
//   setting_turn_speed = -speed;
// }

// void Balanced::PI_SpeedRing()
// {
//    double car_speed=(encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
//    encoder_left_pulse_num_speed = 0;
//    encoder_right_pulse_num_speed = 0;
//    speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
//    speed_filter_old = speed_filter;
//    car_speed_integeral += speed_filter;
//    car_speed_integeral += -setting_car_speed; 
//    car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);

//    speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
// }

// void Balanced::PD_VerticalRing()
// {
//   balance_control_output= kp_balance * (kalmanfilter.angle - 0) + kd_balance * (kalmanfilter.Gyro_x - 0);
// }

// void Balanced::PI_SteeringRing()
// {  
//    rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;////control with Z-axis gyroscope
// }


// Mpu6050::Mpu6050() // : myIMU(SPI_MODE, 10)  // SparkFun  
// {
//   // dt is time b/t samples, so 104 Hz or 0.0096 sec. (original: 0.005)
//   dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
// }

// void Mpu6050::init()
// {
//    Serial.println("Initializing IMU...");
//    if (!IMU.begin()) {
//     Serial.println("Failed to initialize IMU!");
//     while (1);
//    }   
//    Serial.print("Accelerometer sample rate = ");
//    Serial.println(IMU.accelerationSampleRate());
//    Serial.print("Gyro sample rate = ");
//    Serial.println(IMU.gyroscopeSampleRate());
//    Serial.println("IMU initialized");
//  }

// void Mpu6050::PrintData()
// {
//   Serial.print(ax);
//   Serial.print(",");
//   Serial.print(ay);
//   Serial.print(",");
//   Serial.print(az);
//   Serial.print(",");
//   Serial.print(gx);
//   Serial.print(",");
//   Serial.print(gy);
//   Serial.print(",");
//   Serial.println(gz);
// }

// void Mpu6050::DataProcessing()
// { 
//   IMU.readAcceleration(ax, ay, az);
//   IMU.readGyroscope(gx, gy, gz);
//   float scale = 1;
//   ax *= 9.81/scale;
//   ay *= 9.81/scale;
//   az *= 9.81/scale;
//   kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);// Obtaining Angle by Kalman Filter
// }










// timer interrupt routine
// make sure that you set a long enough interval for the timer to run
// code runs 208 times in 2 seconds
void TimerHandler() {
  counter++;
  balanced.Get_EncoderSpeed();
  mpu6050.DataProcessing();
  balanced.PD_VerticalRing();
  balanced.Total_Control();
}

// can run at 500hz iff the robot is in +- 27 degrees
void TimerHandler2() {
  counter++;  // count number of executions per period
  // interrupts(); // might not be needed
  balanced.Get_EncoderSpeed();
  mpu6050.DataProcessing();
  balanced.PD_VerticalRing();
  balanced.interrupt_cnt++;
  if (balanced.interrupt_cnt > 4)  // orig: 8 @ 5ms, try 4 @ 9.6ms period
  {
    balanced.interrupt_cnt = 0;
    balanced.PI_SpeedRing();
    balanced.PI_SteeringRing();
  }
  balanced.Total_Control();
}


void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  delay(100);
  Serial.print(F("\nStarting TimerInterruptTest on "));
  Serial.println(BOARD_NAME);
  Serial.println(SAMD_TIMER_INTERRUPT_VERSION);
  Serial.print(F("CPU Frequency = "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));

  // Interval in microseconds
  if (ITimer.attachInterruptInterval(TIMER_INTERVAL_US, TimerHandler)) {
    Serial.print(F("Starting ITimer OK, Interval (us) = "));
    Serial.println(TIMER_INTERVAL_US);
  } else Serial.println(F("Can't set ITimer. Select another freq. or timer"));

  Motor.Pin_init();
  Serial.println("Motor pins initialized");
  Motor.Encoder_init();
  Serial.println("Encoders initialized");
  mpu6050.init();
  Serial.println("Setup complete");
  delay(100);
  // mpu6050.PrintData();
}

long timer = millis();

// printing data occurs at 28Hz
void loop() {
  // every second print interupt counter
  mpu6050.PrintData(); // nesting is slow
  if (millis() - timer > 2000) {
    Serial.println(kalmanfilter.angle)
    timer = millis();
    Serial.print("Number of Balance Executions: ");
    Serial.println(counter);  // number of executions of interrupt routine
    Serial.print("Number of Read Executions: ");
    Serial.println(counter+sensor_freq);  // number of executions of interrupt routine
    counter = 0; 
    sensor_freq = 0;
  };
  sensor_freq++;
}