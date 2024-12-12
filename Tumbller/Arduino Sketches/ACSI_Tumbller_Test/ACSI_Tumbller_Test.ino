// Manual spinning begin
#include <Arduino.h>
#include "Motor.h"
#include <ArduinoBLE.h>
// #include "SparkFunLSM6DS3.h"
#include "Balanced.h"

Mpu6050 mpu6050;
Balanced balanced(mpu6050);
Motor Motor;

/* Keep commented out
// void setup() 
// {
//   Serial.begin(115200);
//   Serial.println("Initializing...");
  
//   Motor.Pin_init();
//   Serial.println("Motor pins initialized");
  
//   Motor.Encoder_init();
//   Serial.println("Encoders initialized");
  
//   Serial.println("Setup complete");
//   delay(100);
// }

// void loop() 
// {
//   Serial.println("Forward");
//   Motor.Forward(128); // 50% speed
//   delay(3000); // 3 seconds

//   Serial.println("Back");
//   Motor.Back(128); // 50% speed
//   delay(3000); // 3 seconds

//   Serial.println("Left");
//   Motor.Left(128); // 50% speed
//   delay(3000); // 3 seconds

//   Serial.println("Right");
//   Motor.Right(128); // 50% speed
//   delay(3000); // 3 seconds

//   Motor.Stop();
//   delay(3000); // 3 seconds stop
// }
Keep commented end */

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
#include "SAMDTimerInterrupt.h"


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
// SAMDTimer 
int counter = 0;
int sensor_freq = 0;

// timer interrupt routine
// make sure that you set a long enough interval for the timer to run
// code runs 208 times in 2 seconds

  // timer interrupt toggles pin SQUARE_WAVE_PIN
  // interrupts();
  // digitalWrite(SQUARE_WAVE_PIN, !digitalRead(SQUARE_WAVE_PIN));
  // Motor.Forward(128);




void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  delay(100);
  /* Specs */
  // Serial.print(F("\nStarting TimerInterruptTest on "));
  // Serial.println(BOARD_NAME);
  // Serial.println(SAMD_TIMER_INTERRUPT_VERSION);
  // Serial.print(F("CPU Frequency = "));
  // Serial.print(F_CPU / 1000000);
  // Serial.println(F(" MHz"));

  // Interval in microseconds
  if (ITimer.attachInterruptInterval(TIMER_INTERVAL_US, TimerHandler)) {
    Serial.print(F("Starting ITimer OK, Interval (us) = "));
    Serial.println(TIMER_INTERVAL_US);
  } else Serial.println(F("Can't set ITimer. Select another freq. or timer"));
  
  /* Wire library for I2C*/
  // Wire.begin(); // use SPI not I2C
  // Serial.println("\nI2C Scanner");
  // for (byte address = 1; address < 127; address++) {
  //   Wire.beginTransmission(address);
  //   byte error = Wire.endTransmission();

  //   if (error == 0) {
  //     Serial.print("I2C device found at address 0x");
  //     if (address < 16) Serial.print("0");
  //     Serial.print(address, HEX);
  //     Serial.println(" !");
  //   } else if (error == 4) {
  //     Serial.print("Unknown error at address 0x");
  //     if (address < 16) Serial.print("0");
  //     Serial.println(address, HEX);
  //   }
  // }

  Motor.Pin_init();
  Serial.println("Motor pins initialized");
  Motor.Encoder_init();
  Serial.println("Encoders initialized");
  mpu6050.init(); // if not using KF or balanced
  Serial.println("Setup complete");
  delay(100);
  // mpu6050.PrintData();

}


unsigned long delay_handle = 0;
// This takes <1.2 ms to run fully
void TimerHandler() {
  // counter++;
  delay_handle = micros();
  // Serial.print("Read encoder: ");
  // Serial.println(micros() - delay_handle);
  balanced.Get_EncoderSpeed();
  // Serial.print("Read IMU: ");
  // mpu6050.readIMU(); // DRO
  // mpu6050.DataProcessing(); // KF
  // Serial.println(micros() - delay_handle);
  // Serial.print("Vertical computation: ");
  balanced.PD_VerticalRing();
  // Serial.println(micros() - delay_handle);
  // Serial.print("Send motor PWM: ");
  balanced.Total_Control(); 
  // Serial.println(micros() - delay_handle);
}



// can run at 500hz iff the robot is in +- 27 degreesT
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

long timer = millis();

// printing data occurs at 28Hz
void loop() {
  // every second print interupt counter
  // Serial.println("Loop executing");
  // delay(10);f
  // balanced.Get_EncoderSpeed();
  // mpu6050.DataProcessing();
  // balanced.PD_VerticalRing();
  // balanced.Total_Control();
  // mpu6050.PrintData(); // nesting is slow
  if (millis() - timer > 2000) {
    timer = millis();
    // Serial.print("Number of Balance Executions: ");
    // Serial.println(counter);  // number of executions of interrupt routine
    // Serial.print("Number of Read Executions: ");
    // Serial.println(counter+sensor_freq);  // number of executions of interrupt routine
    counter = 0; 
    sensor_freq = 0;
    // Motor.Stop();
  };
  sensor_freq++;
}