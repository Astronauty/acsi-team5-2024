// Manual spinning begin
#include <Arduino.h>
#include "Motor.h"

Motor Motor;

//* Keep commented out


void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  delay(100);
  // Serial.print(F("\nStarting TimerInterruptTest on "));
  // Serial.println(BOARD_NAME);
  // Serial.println(SAMD_TIMER_INTERRUPT_VERSION);
  // Serial.print(F("CPU Frequency = "));
  // Serial.print(F_CPU / 1000000);
  // Serial.println(F(" MHz"));

  // Interval in microseconds
  // if (ITimer.attachInterruptInterval(TIMER_INTERVAL_US, TimerHandler)) {
  //   Serial.print(F("Starting ITimer OK, Interval (us) = "));
  //   // Serial.println(TIMER_INTERVAL_US);
  // } else Serial.println(F("Can't set ITimer. Select another freq. or timer"));

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
  Serial.println("Setup complete");
  delay(100);
  // mpu6050.PrintData();
}

long timer = millis();

// Lemniscate
int duration = 0;
unsigned long routine = 5750;
unsigned long lastTime = 0;
int state = 0;

// void loop() {
//   // every second pri(); // nesting is slow
//   // Serial.print("Left: ");
//   // Serial.print(Motor.encoder_count_left_a);
//   // Serial.print("\t");
//   // Serial.print("Right: ");
//   // Serial.println(Motor.encoder_count_right_a);
//   Serial.println(state);
//   if (millis() - lastTime > duration) {
//     lastTime = millis();
//     switch (state) {
//       case 0:
//         Motor.FwdCW(130, 2);
//         duration = routine; // turn more than move
//         break;
//       case 1:
//         Motor.FwdCCW(130, 2);
//         duration = routine;
//         break;
//     }
//     state = (state + 1) % 2; // Cycle through the states
//   }
// }


// // 1m circle 
// void loop() {
//   // every second pri(); // nesting is slow
//   Serial.print("Left: ");
//   Serial.print(Motor.encoder_count_left_a);
//   Serial.print("\t");
//   Serial.print("Right: ");
//   Serial.println(Motor.encoder_count_right_a);
//   if (millis() - lastTime > duration) {
//     lastTime = millis();
//     switch (state) {
//       case 0:
//         Motor.BackCW(100, 2);
//         duration = 200; // turn more than move
//         break;
//       case 1:
//         // Motor.Forward(100);
//         duration = 1500;
//         break;
//     }
//     state = (state + 1) % 2; // Cycle through the states
//   }
// }


void loop() {
  // every second pri(); // nesting is slow
  Serial.print("Left: ");
  Serial.print(Motor.encoder_count_left_a);
  Serial.print("\t");
  Serial.print("Right: ");
  Serial.println(Motor.encoder_count_right_a);
  if (millis() - lastTime > duration) {
    lastTime = millis();
    switch (state) {
      case 0:
        Motor.Back(120);
        duration = 3400; // turn more than move
        break;
      case 1:
        // Motor.Forward(100);
        Motor.Back(10);
        duration = 500;
        break;
      case 2:
        // Motor.Forward(100);
        Motor.Forward(50);
        duration = 300;
        break;
      case 3:
        Motor.Forward(120);
        duration = 3400; // turn more than move
        break;
      case 4:
        // Motor.Forward(100);
        Motor.Forward(10);
        duration = 500;
        break;
      case 5:
        // Motor.Forward(100);
        Motor.Back(50);
        duration = 300;
        break;
    }
    state = (state + 1) % 6; // Cycle through the states
  }
}