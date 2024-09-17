#ifndef MPU6050AHRS_H
#define MPU6050AHRS_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050AHRS {
public:
    MPU6050AHRS(uint8_t address = 0x68); // Default I2C address (change if needed)
    bool begin();
    void calibrateGyro(bool verbose = false, int num_samples = 500);
    void readSensorData();
    void updateFilter();
    void computeAngles();
    float getYaw();
    float getPitch();
    float getRoll();
    bool isCalibrated();

private:
    void updateMahonyFilter(float ax, float ay, float az, float gx, float gy, float gz, float deltat);

    // Sensor address
    uint8_t MPU_addr;

    // Calibration data
    float A_cal[6];
    float G_off[3];

    // Raw sensor data
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    int16_t temp_raw;

    // Scaled sensor data
    float Axyz[3];
    float Gxyz[3];

    // Quaternion components
    float q[4];

    // Mahony filter parameters
    float Kp;
    float Ki;
    float integralFBx, integralFBy, integralFBz;

    // Timing variables
    unsigned long lastUpdate;
    float deltat;

    // Euler angles
    float yaw, pitch, roll;

    // Calibration flags
    bool calibrated;
    unsigned int cal_count;
    long gsum[3];
};

#endif // MPU6050AHRS_H
