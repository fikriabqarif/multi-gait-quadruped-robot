#ifndef IMU_SAFETY_H
#define IMU_SAFETY_H

#include <Wire.h>
#include <Arduino.h>

// MPU6050 I2C address (AD0 pin low = 0x68, high = 0x69)
#define MPU6050_ADDRESS 0x68

// MPU6050 register addresses
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_GYRO_XOUT_H 0x43

// Default safety thresholds (in degrees)
#define DEFAULT_ROLL_THRESHOLD 45.0
#define DEFAULT_PITCH_THRESHOLD 45.0

// Filter constants for complementary filter
#define ALPHA 0.98  // Gyro weight in complementary filter

class IMUSafety {
public:
    IMUSafety();
    
    // Initialization
    bool init();
    void calibrate();  // Calibrate gyro offsets (robot should be stationary)
    
    // Data reading
    void update();  // Call this regularly (e.g., every loop iteration)
    float getRoll();   // Roll angle in degrees
    float getPitch();  // Pitch angle in degrees
    float getYaw();    // Yaw angle in degrees (drifts without magnetometer)
    
    // Raw sensor data
    void getAccel(int16_t& x, int16_t& y, int16_t& z);
    void getGyro(int16_t& x, int16_t& y, int16_t& z);
    float getTemperature();
    
    // Safety monitoring
    void setRollThreshold(float threshold);
    void setPitchThreshold(float threshold);
    bool isRollExceeded();
    bool isPitchExceeded();
    bool isTiltExceeded();  // Either roll or pitch exceeded
    
    // Utility
    bool isInitialized();
    
private:
    // I2C communication
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    int16_t read16BitRegister(uint8_t reg);
    
    // Sensor data processing
    void readRawData();
    void calculateAngles();
    
    // Raw sensor data
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    
    // Calculated angles (degrees)
    float roll, pitch, yaw;
    
    // Gyro calibration offsets
    int16_t gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    
    // Safety thresholds
    float rollThreshold;
    float pitchThreshold;
    
    // State
    bool initialized;
    bool calibrated;
    unsigned long lastUpdateTime;
    float dt;  // Time delta in seconds
};

#endif // IMU_SAFETY_H

