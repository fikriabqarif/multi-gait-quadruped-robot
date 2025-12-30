#ifndef ROLL_GAIT_H
#define ROLL_GAIT_H

#include "../servo_controller.h"
#include "../leg_kinematics.h"
#include <Arduino.h>

// Rolling gait parameters
#define DEFAULT_ROLL_RADIUS 100.0   // mm - approximate body radius
#define DEFAULT_ROLL_SPEED 0.5      // 0.0 to 1.0
#define DEFAULT_ROLL_DURATION 2000  // ms per full rotation

class RollGait {
public:
    RollGait(ServoController* servos, LegKinematics* kinematics);
    
    // Initialization
    void init();
    
    // Gait control
    void start(float speed = 0.5, float direction = 0.0);
    void stop();
    void update();  // Call regularly in main loop
    
    // Configuration
    void setRollRadius(float radius);
    void setRollDuration(uint16_t duration);
    void setSpeed(float speed);
    void setDirection(float direction);  // 0 = forward, 90 = right, etc.
    
    // Status
    bool isActive();
    void reset();
    
private:
    ServoController* servos;
    LegKinematics* kinematics;
    
    // Gait parameters
    float rollRadius;
    uint16_t rollDuration;
    float currentSpeed;
    float currentDirection;
    
    // Gait state
    bool active;
    unsigned long cycleStartTime;
    float cycleProgress;  // 0.0 to 1.0 (one full rotation)
    
    // Rolling sequence: legs move in sequence to create rolling motion
    // Each leg goes through: retract -> extend -> retract cycle
    struct LegRollState {
        float phase;  // 0.0 to 1.0, offset for each leg
        bool extended;
    };
    LegRollState legStates[4];
    
    // Helper functions
    void updateCycle();
    void calculateLegPositions();
    void applyLegPositions();
    Point3D getRollPosition(uint8_t leg, float progress);
    float getLegPhase(uint8_t leg);
};

#endif // ROLL_GAIT_H

