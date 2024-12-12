#include <Arduino.h>
#include "Motor.h"

//Getting Right Wheel Speed.
static void EncoderCountRightA();
unsigned long Motor::encoder_count_right_a;
static void EncoderCountRightA()
{
  Motor::encoder_count_right_a++;
}

//Getting Left Wheel Speed.
static void EncoderCountLeftA();
unsigned long Motor::encoder_count_left_a;
static void EncoderCountLeftA()
{
  Motor::encoder_count_left_a++;
}


void Motor::Pin_init()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
}

void Motor::Encoder_init() {
  pinMode(ENCODER_LEFT_A_PIN, INPUT);
  pinMode(ENCODER_RIGHT_A_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), []() {
    encoder_count_left_a++;
  }, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), []() {
    encoder_count_right_a++;
  }, RISING);
}

Motor::Motor()
{
  // MOVE[0] = &Motor::Forward;
  // MOVE[1] = &Motor::Back;
  // MOVE[2] = &Motor::Left;
  // MOVE[3] = &Motor::Right;
}

void Motor::Stop()
{
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void Motor::FwdCW(int speed, float ratio)
{ 
  digitalWrite(AIN1, 0);
  digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed/ratio);
}

void Motor::BackCCW(int speed, float ratio)
{
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed/ratio);
}

void Motor::BackCW(int speed, float ratio)
{
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed/ratio);
  analogWrite(PWMB_RIGHT, speed);
}

void Motor::FwdCCW(int speed, float ratio)
{
  digitalWrite(AIN1, 0);
  digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, speed/ratio);
  analogWrite(PWMB_RIGHT, speed);
}

void Motor::Forward(int speed)
{
  digitalWrite(AIN1, 0);
  digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, 6+speed);
}

void Motor::Back(int speed)
{
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, 9+speed); // 5% of 180
}


void Motor::Left(int speed)
{
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, 0);
}

void Motor::Right(int speed)
{
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT,0);
  analogWrite(PWMB_RIGHT,speed);
}

void Motor::Control(int PIN,int PIN_value,int PWM_pin,int speed)
{
  digitalWrite(PIN, PIN_value);
  analogWrite(PWM_pin,speed);
}