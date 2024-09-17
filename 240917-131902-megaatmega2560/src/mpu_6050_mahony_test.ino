#include <Wire.h>
#include "MPU6050AHRS.h"

// Create an instance of the class
MPU6050AHRS mpu(0x69);

void setup() {
    Serial.begin(9600);
    while (!Serial); // Wait for Serial Monitor to open

    if (!mpu.begin()) {
        Serial.println("Failed to initialize MPU-6050!");
        Serial.println("Maybe you could run i2c_scanner example to find the address.");
        while (1); // Halt if initialization failed
    }

    Serial.println("MPU-6050 Initialized. Starting gyro calibration...");
}

unsigned long currentTime = 0;

void loop() {
    currentTime = millis();

    if (!mpu.isCalibrated()) {
        mpu.calibrateGyro();
        return;
    }

    mpu.readSensorData();
    mpu.updateFilter();
    mpu.computeAngles();

    serialPrintTask(currentTime);
}

// Serial Print Task
void serialPrintTask(unsigned long ct) {
    static unsigned long lastTime = 0;
    const unsigned long rate = 200;

    if (ct - lastTime < rate)
        return;
    
    lastTime = ct;

    Serial.print("Yaw: ");
    Serial.print(mpu.getYaw(), 1);
    Serial.print("°, Pitch: ");
    Serial.print(mpu.getPitch(), 1);
    Serial.print("°, Roll: ");
    Serial.print(mpu.getRoll(), 1);
    Serial.println("°");
}

