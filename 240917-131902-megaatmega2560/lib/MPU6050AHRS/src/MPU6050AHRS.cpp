#include "MPU6050AHRS.h"

MPU6050AHRS::MPU6050AHRS(uint8_t address)
  : MPU_addr(address) {
  // Initialize calibration data (adjust these values based on your sensor calibration)
  A_cal[0] = 265.0;
  A_cal[1] = -80.0;
  A_cal[2] = -700.0;  // Offsets
  A_cal[3] = 0.994;
  A_cal[4] = 1.000;
  A_cal[5] = 1.014;  // Scale factors

  G_off[0] = -499.5;
  G_off[1] = -17.7;
  G_off[2] = -82.0;

  // Initialize quaternion
  q[0] = 1.0;
  q[1] = 0.0;
  q[2] = 0.0;
  q[3] = 0.0;

  // Mahony filter parameters
  Kp = 30.0;
  Ki = 0.0;
  integralFBx = integralFBy = integralFBz = 0.0;

  lastUpdate = micros();
  deltat = 0.0;

  yaw = pitch = roll = 0.0;

  // Calibration flags
  calibrated = false;
  cal_count = 0;
  gsum[0] = gsum[1] = gsum[2] = 0;
}

bool MPU6050AHRS::begin() {
  Wire.begin();
  // Initialize MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up the MPU-6050
  if (Wire.endTransmission(true) != 0) {
    return false;  // Failed to initialize
  }
  return true;
}

void MPU6050AHRS::calibrateGyro(bool verbose, int num_samples) {
  long gx_off = 0, gy_off = 0, gz_off = 0;
  int16_t gx, gy, gz;

  for (int i = 0; i < num_samples; ++i) {
    readSensorData();
    gx_off += gx;
    gy_off += gy;
    gz_off += gz;
    delay(5);  // Small delay to allow sensor to stabilize
  }

  G_off[0] = gx_off / num_samples;
  G_off[1] = gy_off / num_samples;
  G_off[2] = gz_off / num_samples;

  if (verbose) {
    Serial.print(F("Gyro calibration complete:\n"));
    Serial.print(F("G_off[0]: "));
    Serial.println(G_off[0]);
    Serial.print(F("G_off[1]: "));
    Serial.println(G_off[1]);
    Serial.print(F("G_off[2]: "));
    Serial.println(G_off[2]);
  }

  calibrated = true;
}

void MPU6050AHRS::readSensorData() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  // Request 14 registers

  ax_raw = (Wire.read() << 8) | Wire.read();
  ay_raw = (Wire.read() << 8) | Wire.read();
  az_raw = (Wire.read() << 8) | Wire.read();
  temp_raw = (Wire.read() << 8) | Wire.read();
  gx_raw = (Wire.read() << 8) | Wire.read();
  gy_raw = (Wire.read() << 8) | Wire.read();
  gz_raw = (Wire.read() << 8) | Wire.read();
}

void MPU6050AHRS::updateFilter() {
  unsigned long now = micros();
  deltat = (now - lastUpdate) * 1.0e-6f;  // Convert to seconds
  lastUpdate = now;

  // Apply calibration to accelerometer data
  Axyz[0] = (static_cast<float>(ax_raw) - A_cal[0]) * A_cal[3];
  Axyz[1] = (static_cast<float>(ay_raw) - A_cal[1]) * A_cal[4];
  Axyz[2] = (static_cast<float>(az_raw) - A_cal[2]) * A_cal[5];

  // Apply calibration to gyro data
  const float gscale = ((250.0f / 32768.0f) * DEG_TO_RAD);  // Convert to radians/s
  Gxyz[0] = (static_cast<float>(gx_raw) - G_off[0]) * gscale;
  Gxyz[1] = (static_cast<float>(gy_raw) - G_off[1]) * gscale;
  Gxyz[2] = (static_cast<float>(gz_raw) - G_off[2]) * gscale;

  // Update Mahony filter
  updateMahonyFilter(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);
}

void MPU6050AHRS::computeAngles() {
  // Compute Tait-Bryan angles
  roll = atan2(q[0] * q[1] + q[2] * q[3], 0.5f - q[1] * q[1] - q[2] * q[2]);
  pitch = asin(2.0f * (q[0] * q[2] - q[1] * q[3]));
  yaw = -atan2(q[1] * q[2] + q[0] * q[3], 0.5f - q[2] * q[2] - q[3] * q[3]);

  // Convert to degrees
  yaw *= RAD_TO_DEG;
  if (yaw < 0) yaw += 360.0f;
  pitch *= RAD_TO_DEG;
  roll *= RAD_TO_DEG;
}

float MPU6050AHRS::getYaw() {
  return yaw;
}

float MPU6050AHRS::getPitch() {
  return pitch;
}

float MPU6050AHRS::getRoll() {
  return roll;
}

bool MPU6050AHRS::isCalibrated() {
  return calibrated;
}

float MPU6050AHRS::getAccelX() {
  return Axyz[0];
}

float MPU6050AHRS::getAccelY() {
  return Axyz[1];
}

float MPU6050AHRS::getAccelZ() {
  return Axyz[2];
}

void MPU6050AHRS::updateMahonyFilter(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;
  float qa, qb, qc;

  // Normalize accelerometer measurement
  float norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return;  // Avoid division by zero
  recipNorm = 1.0f / norm;
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;

  // Estimated direction of gravity
  vx = q[1] * q[3] - q[0] * q[2];
  vy = q[0] * q[1] + q[2] * q[3];
  vz = q[0] * q[0] - 0.5f + q[3] * q[3];

  // Error is cross product between estimated and measured direction of gravity
  ex = (ay * vz - az * vy);
  ey = (az * vx - ax * vz);
  ez = (ax * vy - ay * vx);

  // Apply integral feedback if Ki > 0
  if (Ki > 0.0f) {
    integralFBx += Ki * ex * deltat;
    integralFBy += Ki * ey * deltat;
    integralFBz += Ki * ez * deltat;
    gx += integralFBx;  // Apply integral feedback
    gy += integralFBy;
    gz += integralFBz;
  }

  // Apply proportional feedback
  gx += Kp * ex;
  gy += Kp * ey;
  gz += Kp * ez;

  // Integrate rate of change of quaternion
  gx *= (0.5f * deltat);  // Pre-multiply common factors
  gy *= (0.5f * deltat);
  gz *= (0.5f * deltat);
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // Normalize quaternion
  norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  recipNorm = 1.0f / norm;
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;
}
