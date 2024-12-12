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


/****************************************************************************************************************************
  ISR_Timers_Array_Simple.ino
  For Arduino and Adadruit AVR 328(P) and 32u4 boards
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/TimerInterrupt
  Licensed under MIT license

  Now we can use these new 16 ISR-based timers, while consuming only 1 hardware Timer.
  Their independently-selected, maximum interval is practically unlimited (limited only by unsigned long miliseconds)
  The accuracy is nearly perfect compared to software timers. The most important feature is they're ISR-based timers
  Therefore, their executions are not blocked by bad-behaving functions / tasks.
  This important feature is absolutely necessary for mission-critical tasks.

  Notes:
  Special design is necessary to share data between interrupt code and the rest of your program.
  Variables usually need to be "volatile" types. Volatile tells the compiler to avoid optimizations that assume
  variable can not spontaneously change. Because your function may change variables while your program is using them,
  the compiler needs this hint. But volatile alone is often not enough.
  When accessing shared variables, usually interrupts must be disabled. Even with volatile,
  if the interrupt changes a multi-byte variable between a sequence of instructions, it can be read incorrectly.
  If your data is multiple variables, such as an array and a count, usually interrupts need to be disabled
  or the entire sequence of your code which accesses the data.
*****************************************************************************************************************************/

// These define's must be placed at the beginning before #include "TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#if ( defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)  || \
        defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI) ||    defined(ARDUINO_AVR_ETHERNET) || \
        defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_BT)   || defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_PRO)      || \
        defined(ARDUINO_AVR_NG) || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_FEATHER328P) || \
        defined(ARDUINO_AVR_METRO) || defined(ARDUINO_AVR_PROTRINKET5) || defined(ARDUINO_AVR_PROTRINKET3) || defined(ARDUINO_AVR_PROTRINKET5FTDI) || \
        defined(ARDUINO_AVR_PROTRINKET3FTDI) )
  #define USE_TIMER_1     true
  #warning Using Timer1
#else          
  #define USE_TIMER_3     true
  #warning Using Timer3
#endif

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "TimerInterrupt.h"

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "ISR_Timer.h"

// #include <TimerInterrupt.hpp>         //https://github.com/khoih-prog/TimerInterrupt
// #include <ISR_Timer.hpp>              //https://github.com/khoih-prog/TimerInterrupt

#include <SimpleTimer.h>              // https://github.com/schinken/SimpleTimer
#include <Wire.h>
#include "GY521.h"
#include "Balanced.h"
const int MPU = 0x68; 
GY521 sensor(MPU);
Mpu6050 mpu(sensor);
Balanced balanced(mpu);

ISR_Timer ISR_timer;

// You have to use longer time here if having problem because Arduino AVR clock is low, 16MHz => lower accuracy.
// Tested OK with 1ms when not much load => higher accuracy.
#define TIMER_INTERVAL_MS            1L

void TimerHandler()
{
  ISR_timer.run();
}

void balanceBot()
{
  // Serial.println("get Encoder");
  // // balanced.Get_EncoderSpeed(); // only if doing speed control
  // // mpu.DataProcessing();
  // // mpu.readIMU();
  // Serial.println("balancing");
  // balanced.PD_VerticalRing();
  // // balanced.PI_SpeedRing();
  // // balanced.PI_SteeringRing();
  // balanced.Total_Control();
}

void doingSomething5s()
{
  // Serial.print("5s : ");
}

/////////////////////////////////////////////////

// #define SIMPLE_TIMER_MS        2000L

// // Init SimpleTimer
// SimpleTimer simpleTimer;

// // Here is software Timer, you can do somewhat fancy stuffs without many issues.
// // But always avoid
// // 1. Long delay() it just doing nothing and pain-without-gain wasting CPU power.Plan and design your code / strategy ahead
// // 2. Very long "do", "while", "for" loops without predetermined exit time.
// void simpleTimerDoingSomething2s()
// {
//   static unsigned long previousMillis = startMillis;

//   unsigned long currMillis = millis();

//   Serial.print(F("SimpleTimer : programmed ")); Serial.print(SIMPLE_TIMER_MS);
//   Serial.print(F("ms, current time ms : ")); Serial.print(currMillis);
//   Serial.print(F(", Delta ms : ")); Serial.println(currMillis - previousMillis);

//   Serial.print(F("Timer2s actual : ")); Serial.println(deltaMillis2s);
//   Serial.print(F("Timer5s actual : ")); Serial.println(deltaMillis5s);
  
//   previousMillis = currMillis;
// }

////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  while (!Serial);

  Serial.print(F("\nStarting ISR_Timers_Array_Simple on "));
  Serial.println(BOARD_TYPE);
  Serial.println(TIMER_INTERRUPT_VERSION);
  // Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));

  // Timer0 is used for micros(), millis(), delay(), etc and can't be used
  // Select Timer 1-2 for UNO, 1-5 for MEGA, 1,3,4 for 16u4/32u4
  // Timer 2 is 8-bit timer, only for higher frequency
  // Timer 4 of 16u4 and 32u4 is 8/10-bit timer, only for higher frequency
  
#if USE_TIMER_1

  ITimer1.init();

  // Using ATmega328 used in UNO => 16MHz CPU clock ,

  if (ITimer1.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler))
  {
    Serial.print(F("Starting  ITimer1 OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));
    
#elif USE_TIMER_3

  ITimer3.init();

  if (ITimer3.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler))
  {
    Serial.print(F("Starting  ITimer3 OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer3. Select another freq. or timer"));

#endif


  // You need this timer for non-critical tasks. Avoid abusing ISR if not absolutely necessary.
  // simpleTimer.setInterval(SIMPLE_TIMER_MS, simpleTimerDoingSomething2s);
  Wire.begin();
  mpu.init();
  Serial.print("Calibrating IMU...");
  delay(5000);
  ISR_timer.setInterval(5L, balanceBot);
  // ISR_timer.setInterval(5000L, doingSomething5s);
  // mpu.calibrateIMU(500, 500);
  // MsTimer2::set(5, timerFunction);
  // MsTimer2::start();
}

#define BLOCKING_TIME_MS      10000L

void loop()
{
  // This unadvised blocking task is used to demonstrate the blocking effects onto the execution and accuracy to Software timer
  // You see the time elapse of ISR_Timer still accurate, whereas very unaccurate for Software Timer
  // The time elapse for 2000ms software timer now becomes 3000ms (BLOCKING_TIME_MS)
  // While that of ISR_Timer is still prefect.
  // delay(BLOCKING_TIME_MS);

  // You need this Software timer for non-critical tasks. Avoid abusing ISR if not absolutely necessary
  // You don't need to and never call ISR_Timer.run() here in the loop(). It's already handled by ISR timer.
  // simpleTimer.run();
  Serial.println("get Encoder");
  balanced.Get_EncoderSpeed(); // only if doing speed control
  // mpu.DataProcessing();
  // mpu.readIMU();
  Serial.println("balancing");
  balanced.PD_VerticalRing();
  // balanced.PI_SpeedRing();
  // balanced.PI_SteeringRing();
  balanced.Total_Control();
  // delay(3);
}