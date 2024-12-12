#include <Arduino.h>
#include <ArduinoBLE.h>
#include <SPI.h>
#include <Wire.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoBLE.h>
// #include <ArduinoJson.h>
// #include <PubSubClient.h>
// #include <WiFiNINA.h>
#include "MPU6050.h"

float x, x_dot, y, y_dot, theta, theta_dot;


struct Acc_senseData{
  float acc_x = 0.0F;
  float acc_y = 0.0F;
  float acc_z = 0.0F;
};

struct Gyr_senseData
{
  float gyr_x = 0.0F;
  float gyr_y = 0.0F;
  float gyr_z = 0.0F;
};

static char payload[256];
static Acc_senseData acc_data;
static Gyr_senseData gyr_data;
// StaticJsonDocument<256> doc;

// static unsigned long last_interval_ms = 0;
// int16_t ax, ay, az, gx, gy, gz; // MPU6050
float ax, ay, az, gx, gy, gz;
unsigned long lastSentTime = 0;
int pause = 500; //500


#define CONVERT_gTO_MS2 9.80665f
#define FREQUENCY_HZ 10 //104
#define INTERVAL_MS (1000 / (FREQUENCY_HZ + 1))

// Define a custom BLE Service and Characteristic
BLEService myService("180C");  // Replace with your chosen UUID for the service
BLEStringCharacteristic myCharacteristic("2A56", BLERead | BLENotify, 48);  // Max length of 50 for data string

MPU6050 mpu; //Instantiate a MPU6050 object with the object name MPU.
unsigned long last_interval_ms = 0;


void readIMU()
{
  // MPU6050 mpu_obj;
  // mpu_obj.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  // ax *= 9.81;
  // ay *= 9.81;
  // az *= 9.81;
}

void printIMU()
{
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(gx);
  Serial.print(',');
  Serial.print(gy);
  Serial.print(',');
  Serial.print(gz);
  Serial.print('\n');
}


void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting Bluetooth® Low Energy failed!");
    while (1);
  }

  // mpu.initialize(); //Initialization MPU6050
  // Serial.print("Initialized IMU\n");  
  // delay(2);

  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Set BLE properties
  BLE.setLocalName("Nano33IoT");
  BLE.setAdvertisedService(myService);
  myService.addCharacteristic(myCharacteristic);
  BLE.addService(myService);
  BLE.advertise();

  Serial.println("BLE device is now advertising!");
}


void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    while (central.connected()) {
      if (myCharacteristic.written()) {
        uint8_t data[48];
        myCharacteristic.readValue(data, 48);
        memcpy(&x, data, 8);
        memcpy(&x_dot, data + 8, 8);
        memcpy(&y, data + 16, 8);
        memcpy(&y_dot, data + 24, 8);
        memcpy(&theta, data + 32, 8);
        memcpy(&theta_dot, data + 40, 8);

        // Print the received state variables
        Serial.print("Received state: ");
        Serial.print("x: "); Serial.print(x);
        Serial.print(", x_dot: "); Serial.print(x_dot);
        Serial.print(", y: "); Serial.print(y);
        Serial.print(", y_dot: "); Serial.print(y_dot);
        Serial.print(", theta: "); Serial.print(theta);
        Serial.print(", theta_dot: "); Serial.println(theta_dot);
      }
    }

    Serial.println("Disconnected from central.");
  }
}



//// Direct Readout of Accelerometer to log data
// void loop() {
//   // Listen for BLE connections
//   BLEDevice central = BLE.central();

//   // Check if it's time to read and send sensor data
//   if (millis() > last_interval_ms + INTERVAL_MS) {
//     last_interval_ms = millis();
//     unsigned long currentTime = micros();
//     unsigned long latency = INTERVAL_MS; //currentTime - lastSentTime - pause*1000; // Calculate latency since last packet
//     // Prepare message with timestamp
//     String timeData = "Hello from Nano33IoT! Latency: " + String(latency) + "µs";
//     myCharacteristic.writeValue(timeData);  // Write data to characteristic
//     Serial.print("Bluetooth sent data: ");
//     Serial.println(timeData);
//     lastSentTime = currentTime;  // Update the last sent time


//     // Read accelerometer and gyroscope data
//     readIMU(); // MPU 6050 test
//     // IMU.readAcceleration(ax, ay, az);
//     // IMU.readGyroscope(gx, gy, gz);

//     // Convert accelerometer data to m/s² and format it as six strings
//     // az detects pitch angle
//     // gx detects pitch rate-
//     int len = 1;
//     String values[len];
//     values[0] = String(az * CONVERT_gTO_MS2, 2) + " m/s²" + "\t" + "GYR_X: " + String(gx, 2) + " °/s" + "\t" + "GYR_Y: " + String(gy, 2) + " °/s" + "\t" + "GYR_Z: " + String(gz, 2) + " °/s" ;
//     // values[1] = String(ay * CONVERT_gTO_MS2, 2) + " m/s²" + "\t";
//     // values[0] = String(az * CONVERT_gTO_MS2, 2) + " m/s²" + "\t";
//     // values[3] = String(gx, 2) + " °/s" + "\t";
//     // values[4] = String(gy, 2) + " °/s" + "\t";
//     // values[5] = String(gz, 2) + " °/s" + "\t";
//     // "ACC_X: " + 
//     // "ACC_Y: " + 
//     // "ACC_Z: " + 
//     // "GYR_X: " + 
//     // "GYR_Y: " + 
//     // "GYR_Z: " + 
//     // Print each value with a status bar
    
//     for (int i = 0; i < len; i++) {
//         Serial.print(values[i]);
    
//         // Extract the numeric value from the string
//         float value = values[i].substring(values[i].indexOf(":") + 2).toFloat();
    
//         // Calculate the position in the status bar
//         int pos = map(value, -480, 480, 0, 10);
    
//         // Print the status bar
//         // for (int j = 0; j < 10; j++) {
//         //     if (j == 5) {
//         //         Serial.print("|");  // Middle point
//         //     } else if (j == pos) {
//         //         Serial.print("*");  // Value position
//         //     } else {
//         //         Serial.print("-");
//         //     }
//         // }
//         // Serial.println();
//     }
//     Serial.println();
//   }

//   // Check for BLE connection
//   if (central) {
//     Serial.print("Connected to central: ");
//     Serial.println(central.address());

//     // Keep sending data while connected
//     while (central.connected()) {
//       delay(pause);  // Control the update rate
//     }

//     Serial.println("Disconnected from central.");
//   }
// }
