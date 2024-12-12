#include "KalmanFilter.h"

void KalmanFilter::Yiorderfilter(float angle_m, float gyro_m,float dt,float K1) {
  angle6 = K1 * angle_m + (1 - K1) * (angle6 + gyro_m * dt);
}

void KalmanFilter::Kalman_Filter(double angle_m, double gyro_m,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0) {
  // float check_time = micros(); // takes 0.16 ms to run
  angle += (gyro_m - q_bias) * dt;
  angle_err = angle_m - angle;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err;
  // angle -= 0.2; // OH
  // if (angle < 0.25 || angle > -0.25) { 
  //   angle = 0;
  // }
  q_bias += K_1 * angle_err;
  angle_dot = gyro_m - q_bias;
  // Serial.println(micros() - check_time);
}
// void KalmanFilter::Angle(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1) {
void KalmanFilter::Angle(float ax, float ay, float az, float gx, float gy, float gz, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1) {
  float Angle = atan2(-ay , az) * 57.3; // atan2(ay, az) on standard Nano 
  // Serial.print("ay = "); // OH 
  // Serial.print(ay); //OH
  // Serial.print("\t"); //OH
  // Serial.print("az = "); //OH
  // Serial.println(az); // OH
  Gyro_x = (gx - 128.1) / 131; // subtract drift, divide
  Kalman_Filter(Angle, Gyro_x, dt, Q_angle, Q_gyro, R_angle, C_0); // check 1
  // angle = Angle; // OH test shows it jumping from 0 to +-6.34; the culprit is precision not MEMS inertia or KF lag
  // Yiorderfilter(Angle, Gyro_x, dt, K1); // check 2
  // Yiorderfilter(angle, Gyro_x, dt, K1); // check 3 - not a good idea
  Gyro_z = -gz / 131;
}
