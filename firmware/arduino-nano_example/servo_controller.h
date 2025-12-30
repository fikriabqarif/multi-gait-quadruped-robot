#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Wire.h>
#include <Arduino.h>

// PCA9685 I2C address (default 0x40, can be changed with address jumpers)
#define PCA9685_ADDRESS 0x40

// PCA9685 register addresses
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06

// Servo configuration
#define NUM_SERVOS 12
#define SERVO_FREQ 50  // 50Hz for standard servos
#define DEFAULT_MIN_PULSE 500   // 0.5ms in microseconds
#define DEFAULT_MAX_PULSE 2500  // 2.5ms in microseconds
#define DEFAULT_CENTER_PULSE 1500  // 1.5ms center position

// Leg and joint definitions
#define LEGS_PER_ROBOT 4
#define JOINTS_PER_LEG 3

// Leg indices
#define LEG_FRONT_LEFT 0
#define LEG_FRONT_RIGHT 1
#define LEG_BACK_LEFT 2
#define LEG_BACK_RIGHT 3

// Joint indices (per leg)
#define JOINT_HIP 0
#define JOINT_UPPER_LEG 1
#define JOINT_LOWER_LEG 2

class ServoController {
public:
    ServoController();
    
    // Initialization
    bool init();
    void setFrequency(uint8_t freq);
    
    // Servo control
    void setAngle(uint8_t channel, float angle);  // Angle in degrees (0-180)
    void setPulseWidth(uint8_t channel, uint16_t pulseWidth);  // Pulse width in microseconds
    
    // Calibration
    void setServoLimits(uint8_t channel, uint16_t minPulse, uint16_t maxPulse);
    void setServoCenter(uint8_t channel, uint16_t centerPulse);
    void getServoLimits(uint8_t channel, uint16_t& minPulse, uint16_t& maxPulse);
    
    // Convenience functions for leg/joint addressing
    void setLegJointAngle(uint8_t leg, uint8_t joint, float angle);
    void setLegJointPulse(uint8_t leg, uint8_t joint, uint16_t pulseWidth);
    
    // Utility
    uint8_t getChannel(uint8_t leg, uint8_t joint);
    void setAllServos(float angle);  // Set all servos to same angle (for testing)
    void disableAll();  // Set all servos to 0 (off)
    
private:
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void setPWM(uint8_t channel, uint16_t on, uint16_t off);
    
    struct ServoCalibration {
        uint16_t minPulse;
        uint16_t maxPulse;
        uint16_t centerPulse;
    };
    
    ServoCalibration calibrations[NUM_SERVOS];
    bool initialized;
};

#endif // SERVO_CONTROLLER_H

