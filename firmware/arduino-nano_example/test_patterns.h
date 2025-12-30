#ifndef TEST_PATTERNS_H
#define TEST_PATTERNS_H

#include "servo_controller.h"
#include <Arduino.h>

class TestPatterns {
public:
    TestPatterns(ServoController* servos);
    
    // Test patterns
    void allServoSweep();  // Sweep all servos from 0 to 180 degrees
    void legLiftTest(uint8_t leg);  // Lift one leg
    void waveTest();  // Wave motion with front legs
    void standTest();  // Stand up from crouched position
    void individualServoTest(uint8_t channel);  // Test individual servo
    
    // Utility
    void setAllServos(float angle);
    void centerAllServos();
    
private:
    ServoController* servos;
};

#endif // TEST_PATTERNS_H

