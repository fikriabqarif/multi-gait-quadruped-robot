#ifndef WALK_GAIT_H
#define WALK_GAIT_H

#include "../servo_controller.h"
#include "../leg_kinematics.h"
#include <Arduino.h>

// Gait parameters
#define DEFAULT_STEP_HEIGHT 30.0    // mm
#define DEFAULT_STRIDE_LENGTH 40.0  // mm
#define DEFAULT_STEP_DURATION 1000  // ms per step cycle

class WalkGait {
public:
    WalkGait(ServoController* servos, LegKinematics* kinematics);
    
    // Initialization
    void init();
    
    // Gait control
    void start(float speed = 0.5, float direction = 0.0);
    void stop();
    void update();  // Call regularly in main loop
    
    // Configuration
    void setStepHeight(float height);
    void setStrideLength(float length);
    void setStepDuration(uint16_t duration);
    void setSpeed(float speed);  // 0.0 to 1.0
    void setDirection(float direction);  // -180 to 180 degrees
    
    // Status
    bool isActive();
    void reset();
    
private:
    ServoController* servos;
    LegKinematics* kinematics;
    
    // Gait parameters
    float stepHeight;
    float strideLength;
    uint16_t stepDuration;
    float currentSpeed;
    float currentDirection;
    
    // Gait state
    bool active;
    unsigned long cycleStartTime;
    float cycleProgress;  // 0.0 to 1.0
    
    // Trot gait: diagonal leg pairs
    // Phase 0: Front-left + Back-right in swing, others in stance
    // Phase 1: Front-right + Back-left in swing, others in stance
    uint8_t currentPhase;
    
    // Leg positions
    struct LegState {
        bool inSwing;
        float x, y, z;  // Current foot position in leg frame
    };
    LegState legStates[4];
    
    // Helper functions
    void updateCycle();
    void calculateLegPositions();
    void applyLegPositions();
    Point3D getStancePosition(uint8_t leg);
    Point3D getSwingPosition(uint8_t leg, float progress);
    float easeInOut(float t);  // Easing function for smooth motion
};

#endif // WALK_GAIT_H

