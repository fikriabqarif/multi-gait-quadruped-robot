#include "servo_controller.h"

ServoController::ServoController() : initialized(false) {
    // Initialize calibration data with defaults
    for (int i = 0; i < NUM_SERVOS; i++) {
        calibrations[i].minPulse = DEFAULT_MIN_PULSE;
        calibrations[i].maxPulse = DEFAULT_MAX_PULSE;
        calibrations[i].centerPulse = DEFAULT_CENTER_PULSE;
    }
}

bool ServoController::init() {
    Wire.begin();
    
    // Reset PCA9685
    writeRegister(PCA9685_MODE1, 0x00);
    delay(10);
    
    // Set frequency to 50Hz for servos
    setFrequency(SERVO_FREQ);
    
    // Wake up PCA9685 (clear sleep bit)
    uint8_t mode1 = readRegister(PCA9685_MODE1);
    writeRegister(PCA9685_MODE1, mode1 & ~0x10);  // Clear sleep bit
    delay(10);
    
    // Disable all outputs initially
    disableAll();
    
    initialized = true;
    return true;
}

void ServoController::setFrequency(uint8_t freq) {
    // Calculate prescale value for desired frequency
    // prescale = round(25MHz / (4096 * frequency)) - 1
    float prescaleval = 25000000.0;
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5);
    
    // Put PCA9685 to sleep
    uint8_t oldmode = readRegister(PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10;  // Set sleep bit
    writeRegister(PCA9685_MODE1, newmode);
    
    // Set prescale
    writeRegister(PCA9685_PRESCALE, prescale);
    
    // Wake up
    writeRegister(PCA9685_MODE1, oldmode);
    delay(5);
    
    // Enable auto-increment
    writeRegister(PCA9685_MODE1, oldmode | 0x80);
}

void ServoController::setAngle(uint8_t channel, float angle) {
    if (channel >= NUM_SERVOS) return;
    
    // Clamp angle to 0-180 degrees
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Convert angle to pulse width using calibration
    ServoCalibration& cal = calibrations[channel];
    float pulseWidth = cal.minPulse + (angle / 180.0) * (cal.maxPulse - cal.minPulse);
    
    setPulseWidth(channel, (uint16_t)pulseWidth);
}

void ServoController::setPulseWidth(uint8_t channel, uint16_t pulseWidth) {
    if (channel >= NUM_SERVOS) return;
    
    // Clamp pulse width to calibration limits
    ServoCalibration& cal = calibrations[channel];
    if (pulseWidth < cal.minPulse) pulseWidth = cal.minPulse;
    if (pulseWidth > cal.maxPulse) pulseWidth = cal.maxPulse;
    
    // Convert microseconds to 12-bit PWM value (0-4095)
    // At 50Hz, period is 20ms = 20000us
    // PWM value = (pulseWidth / 20000) * 4096
    float pwmValue = (pulseWidth / 20000.0) * 4096.0;
    uint16_t pwm = (uint16_t)pwmValue;
    
    // Set PWM: on at 0, off at calculated value
    setPWM(channel, 0, pwm);
}

void ServoController::setServoLimits(uint8_t channel, uint16_t minPulse, uint16_t maxPulse) {
    if (channel >= NUM_SERVOS) return;
    calibrations[channel].minPulse = minPulse;
    calibrations[channel].maxPulse = maxPulse;
}

void ServoController::setServoCenter(uint8_t channel, uint16_t centerPulse) {
    if (channel >= NUM_SERVOS) return;
    calibrations[channel].centerPulse = centerPulse;
}

void ServoController::getServoLimits(uint8_t channel, uint16_t& minPulse, uint16_t& maxPulse) {
    if (channel >= NUM_SERVOS) {
        minPulse = 0;
        maxPulse = 0;
        return;
    }
    minPulse = calibrations[channel].minPulse;
    maxPulse = calibrations[channel].maxPulse;
}

uint8_t ServoController::getChannel(uint8_t leg, uint8_t joint) {
    if (leg >= LEGS_PER_ROBOT || joint >= JOINTS_PER_LEG) return 255;
    return leg * JOINTS_PER_LEG + joint;
}

void ServoController::setLegJointAngle(uint8_t leg, uint8_t joint, float angle) {
    uint8_t channel = getChannel(leg, joint);
    if (channel < NUM_SERVOS) {
        setAngle(channel, angle);
    }
}

void ServoController::setLegJointPulse(uint8_t leg, uint8_t joint, uint16_t pulseWidth) {
    uint8_t channel = getChannel(leg, joint);
    if (channel < NUM_SERVOS) {
        setPulseWidth(channel, pulseWidth);
    }
}

void ServoController::setAllServos(float angle) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        setAngle(i, angle);
    }
}

void ServoController::disableAll() {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        setPWM(i, 0, 0);
    }
}

void ServoController::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(PCA9685_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t ServoController::readRegister(uint8_t reg) {
    Wire.beginTransmission(PCA9685_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(PCA9685_ADDRESS, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

void ServoController::setPWM(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    
    Wire.beginTransmission(PCA9685_ADDRESS);
    Wire.write(reg);
    Wire.write(on & 0xFF);
    Wire.write(on >> 8);
    Wire.write(off & 0xFF);
    Wire.write(off >> 8);
    Wire.endTransmission();
}

