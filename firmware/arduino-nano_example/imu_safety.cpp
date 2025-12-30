#include "imu_safety.h"

IMUSafety::IMUSafety() 
    : roll(0.0), pitch(0.0), yaw(0.0),
      gyroOffsetX(0), gyroOffsetY(0), gyroOffsetZ(0),
      rollThreshold(DEFAULT_ROLL_THRESHOLD),
      pitchThreshold(DEFAULT_PITCH_THRESHOLD),
      initialized(false), calibrated(false),
      lastUpdateTime(0), dt(0.0) {
}

bool IMUSafety::init() {
    Wire.begin();
    
    // Wake up MPU6050 (clear sleep bit)
    writeRegister(MPU6050_PWR_MGMT_1, 0x00);
    delay(100);
    
    // Set sample rate divider (1kHz / (1 + divider))
    writeRegister(MPU6050_SMPLRT_DIV, 0x07);  // 125Hz sample rate
    
    // Configure DLPF (Digital Low Pass Filter)
    writeRegister(MPU6050_CONFIG, 0x06);  // 5Hz DLPF
    
    // Configure gyroscope range (±250°/s)
    writeRegister(MPU6050_GYRO_CONFIG, 0x00);
    
    // Configure accelerometer range (±2g)
    writeRegister(MPU6050_ACCEL_CONFIG, 0x00);
    
    delay(100);
    
    // Verify communication by reading WHO_AM_I (should be 0x68)
    uint8_t whoAmI = readRegister(0x75);
    if (whoAmI != 0x68) {
        return false;  // Communication failed
    }
    
    initialized = true;
    lastUpdateTime = millis();
    return true;
}

void IMUSafety::calibrate() {
    if (!initialized) return;
    
    // Collect samples for calibration (robot should be stationary)
    const int numSamples = 100;
    long sumX = 0, sumY = 0, sumZ = 0;
    
    for (int i = 0; i < numSamples; i++) {
        readRawData();
        sumX += gyroX;
        sumY += gyroY;
        sumZ += gyroZ;
        delay(10);
    }
    
    gyroOffsetX = sumX / numSamples;
    gyroOffsetY = sumY / numSamples;
    gyroOffsetZ = sumZ / numSamples;
    
    calibrated = true;
    
    // Reset angles to zero
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
}

void IMUSafety::update() {
    if (!initialized) return;
    
    // Calculate time delta
    unsigned long currentTime = millis();
    dt = (currentTime - lastUpdateTime) / 1000.0;  // Convert to seconds
    if (dt > 0.1) dt = 0.1;  // Cap at 100ms to prevent large jumps
    lastUpdateTime = currentTime;
    
    // Read raw sensor data
    readRawData();
    
    // Calculate angles
    calculateAngles();
}

void IMUSafety::readRawData() {
    // Read accelerometer data
    accelX = read16BitRegister(MPU6050_ACCEL_XOUT_H);
    accelY = read16BitRegister(MPU6050_ACCEL_XOUT_H + 2);
    accelZ = read16BitRegister(MPU6050_ACCEL_XOUT_H + 4);
    
    // Read gyroscope data
    gyroX = read16BitRegister(MPU6050_GYRO_XOUT_H);
    gyroY = read16BitRegister(MPU6050_GYRO_XOUT_H + 2);
    gyroZ = read16BitRegister(MPU6050_GYRO_XOUT_H + 4);
    
    // Apply gyro offsets if calibrated
    if (calibrated) {
        gyroX -= gyroOffsetX;
        gyroY -= gyroOffsetY;
        gyroZ -= gyroOffsetZ;
    }
}

void IMUSafety::calculateAngles() {
    // Calculate roll and pitch from accelerometer
    // Using atan2 for better accuracy
    float accelRoll = atan2(accelY, accelZ) * 180.0 / PI;
    float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
    
    // Convert gyro rates to degrees per second
    // Gyro range is ±250°/s, so 131 LSB/(°/s) for ±250°/s range
    float gyroRollRate = gyroX / 131.0;
    float gyroPitchRate = gyroY / 131.0;
    float gyroYawRate = gyroZ / 131.0;
    
    // Complementary filter: combine accelerometer and gyroscope
    // Alpha determines trust in gyro vs accelerometer
    roll = ALPHA * (roll + gyroRollRate * dt) + (1.0 - ALPHA) * accelRoll;
    pitch = ALPHA * (pitch + gyroPitchRate * dt) + (1.0 - ALPHA) * accelPitch;
    yaw = yaw + gyroYawRate * dt;  // Yaw drifts without magnetometer
}

float IMUSafety::getRoll() {
    return roll;
}

float IMUSafety::getPitch() {
    return pitch;
}

float IMUSafety::getYaw() {
    return yaw;
}

void IMUSafety::getAccel(int16_t& x, int16_t& y, int16_t& z) {
    x = accelX;
    y = accelY;
    z = accelZ;
}

void IMUSafety::getGyro(int16_t& x, int16_t& y, int16_t& z) {
    x = gyroX;
    y = gyroY;
    z = gyroZ;
}

float IMUSafety::getTemperature() {
    int16_t temp = read16BitRegister(MPU6050_TEMP_OUT_H);
    return temp / 340.0 + 36.53;  // Formula from MPU6050 datasheet
}

void IMUSafety::setRollThreshold(float threshold) {
    rollThreshold = threshold;
}

void IMUSafety::setPitchThreshold(float threshold) {
    pitchThreshold = threshold;
}

bool IMUSafety::isRollExceeded() {
    return abs(roll) > rollThreshold;
}

bool IMUSafety::isPitchExceeded() {
    return abs(pitch) > pitchThreshold;
}

bool IMUSafety::isTiltExceeded() {
    return isRollExceeded() || isPitchExceeded();
}

bool IMUSafety::isInitialized() {
    return initialized;
}

void IMUSafety::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t IMUSafety::readRegister(uint8_t reg) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

int16_t IMUSafety::read16BitRegister(uint8_t reg) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)2);
    if (Wire.available() >= 2) {
        uint8_t high = Wire.read();
        uint8_t low = Wire.read();
        return (int16_t)((high << 8) | low);
    }
    return 0;
}

