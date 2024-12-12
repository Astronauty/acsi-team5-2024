#ifndef MOTOR_HEAD
#define MOTOR_HEAD

class Motor
{
  public:
          Motor();
          
          void Pin_init();
          /*Measuring_speed*/
          static void Encoder_init(); // Change to static

        //  void (Motor::*MOVE[4])(int speed); 
          void Control(int AIN1_value,int BIN1_value,int PWM_pin,int speed);
          
          void Stop();
          void FwdCW(int speed, float ratio);
          void BackCW(int speed, float ratio);
          void FwdCCW(int speed, float ratio);
          void BackCCW(int speed, float ratio);
          void Forward(int speed);
          void Back(int speed);
          void Left(int speed);
          void Right(int speed);       
          static unsigned long encoder_count_right_a;
          static unsigned long encoder_count_left_a;
          
  private:
          /*Motor pin*/
          #define AIN1 7
          #define PWMA_LEFT 5
          #define BIN1 12
          #define PWMB_RIGHT 6
          #define STBY_PIN 8
          
          /*Encoder measuring speed  pin*/
          #define ENCODER_LEFT_A_PIN 2
          #define ENCODER_RIGHT_A_PIN 4

};

#endif