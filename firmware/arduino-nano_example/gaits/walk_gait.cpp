#include "walk_gait.h"

WalkGait::WalkGait(ServoController* servos, LegKinematics* kinematics)
    : servos(servos), kinematics(kinematics),
      stepHeight(DEFAULT_STEP_HEIGHT),
      strideLength(DEFAULT_STRIDE_LENGTH),
      stepDuration(DEFAULT_STEP_DURATION),
      currentSpeed(0.5),
      currentDirection(0.0),
      active(false),
      cycleStartTime(0),
      cycleProgress(0.0),
      currentPhase(0) {
    for (int i = 0; i < 4; i++) {
        legStates[i].inSwing = false;
        legStates[i].x = 0;
        legStates[i].y = 0;
        legStates[i].z = 60.0;  // Default stance height
    }
}

void WalkGait::init() {
    // Set initial stance positions
    for (uint8_t leg = 0; leg < 4; leg++) {
        Point3D stancePos = getStancePosition(leg);
        legStates[leg].x = stancePos.x;
        legStates[leg].y = stancePos.y;
        legStates[leg].z = stancePos.z;
        
        // Apply initial position
        JointAngles angles;
        if (kinematics->inverseKinematics(stancePos, angles)) {
            servos->setLegJointAngle(leg, JOINT_HIP, angles.hip);
            servos->setLegJointAngle(leg, JOINT_UPPER_LEG, angles.upperLeg);
            servos->setLegJointAngle(leg, JOINT_LOWER_LEG, angles.lowerLeg);
        }
    }
}

void WalkGait::start(float speed, float direction) {
    setSpeed(speed);
    setDirection(direction);
    active = true;
    cycleStartTime = millis();
    cycleProgress = 0.0;
    currentPhase = 0;
}

void WalkGait::stop() {
    active = false;
    // Return to neutral stance
    init();
}

void WalkGait::update() {
    if (!active) return;
    
    updateCycle();
    calculateLegPositions();
    applyLegPositions();
}

void WalkGait::updateCycle() {
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - cycleStartTime;
    
    // Adjust duration based on speed (faster = shorter duration)
    uint16_t adjustedDuration = stepDuration / (currentSpeed + 0.1);  // Avoid division by zero
    
    cycleProgress = (float)elapsed / adjustedDuration;
    
    // When cycle completes, switch phase
    if (cycleProgress >= 1.0) {
        cycleProgress = 0.0;
        cycleStartTime = currentTime;
        currentPhase = (currentPhase + 1) % 2;
        
        // Update which legs are in swing
        if (currentPhase == 0) {
            // Front-left and back-right in swing
            legStates[LEG_FRONT_LEFT].inSwing = true;
            legStates[LEG_BACK_RIGHT].inSwing = true;
            legStates[LEG_FRONT_RIGHT].inSwing = false;
            legStates[LEG_BACK_LEFT].inSwing = false;
        } else {
            // Front-right and back-left in swing
            legStates[LEG_FRONT_LEFT].inSwing = false;
            legStates[LEG_BACK_RIGHT].inSwing = false;
            legStates[LEG_FRONT_RIGHT].inSwing = true;
            legStates[LEG_BACK_LEFT].inSwing = true;
        }
    }
}

void WalkGait::calculateLegPositions() {
    for (uint8_t leg = 0; leg < 4; leg++) {
        if (legStates[leg].inSwing) {
            // Leg is in swing phase
            Point3D swingPos = getSwingPosition(leg, cycleProgress);
            legStates[leg].x = swingPos.x;
            legStates[leg].y = swingPos.y;
            legStates[leg].z = swingPos.z;
        } else {
            // Leg is in stance phase - move backward to propel body forward
            float backwardProgress = cycleProgress;
            Point3D stancePos = getStancePosition(leg);
            
            // Move foot backward during stance (body moves forward)
            float backwardOffset = strideLength * backwardProgress;
            legStates[leg].x = stancePos.x - backwardOffset;
            legStates[leg].y = stancePos.y;
            legStates[leg].z = stancePos.z;
        }
    }
}

void WalkGait::applyLegPositions() {
    for (uint8_t leg = 0; leg < 4; leg++) {
        Point3D target;
        target.x = legStates[leg].x;
        target.y = legStates[leg].y;
        target.z = legStates[leg].z;
        
        // Apply direction offset (rotate around z-axis)
        if (currentDirection != 0.0) {
            float angleRad = currentDirection * PI / 180.0;
            float newX = target.x * cos(angleRad) - target.y * sin(angleRad);
            float newY = target.x * sin(angleRad) + target.y * cos(angleRad);
            target.x = newX;
            target.y = newY;
        }
        
        JointAngles angles;
        if (kinematics->inverseKinematics(target, angles)) {
            servos->setLegJointAngle(leg, JOINT_HIP, angles.hip);
            servos->setLegJointAngle(leg, JOINT_UPPER_LEG, angles.upperLeg);
            servos->setLegJointAngle(leg, JOINT_LOWER_LEG, angles.lowerLeg);
        }
    }
}

Point3D WalkGait::getStancePosition(uint8_t leg) {
    Point3D pos;
    pos.x = 0;  // Centered under leg
    pos.y = 0;
    pos.z = 60.0;  // Stance height (adjust based on robot)
    return pos;
}

Point3D WalkGait::getSwingPosition(uint8_t leg, float progress) {
    Point3D pos = getStancePosition(leg);
    
    // Move foot forward during swing
    float forwardOffset = strideLength * progress;
    pos.x = forwardOffset;
    
    // Lift foot during swing (parabolic trajectory)
    float easedProgress = easeInOut(progress);
    pos.z = getStancePosition(leg).z + stepHeight * sin(easedProgress * PI);
    
    return pos;
}

float WalkGait::easeInOut(float t) {
    // Smooth easing function
    return t < 0.5 ? 2 * t * t : 1 - pow(-2 * t + 2, 2) / 2;
}

void WalkGait::setStepHeight(float height) {
    stepHeight = height;
}

void WalkGait::setStrideLength(float length) {
    strideLength = length;
}

void WalkGait::setStepDuration(uint16_t duration) {
    stepDuration = duration;
}

void WalkGait::setSpeed(float speed) {
    if (speed < 0.0) speed = 0.0;
    if (speed > 1.0) speed = 1.0;
    currentSpeed = speed;
}

void WalkGait::setDirection(float direction) {
    while (direction > 180.0) direction -= 360.0;
    while (direction < -180.0) direction += 360.0;
    currentDirection = direction;
}

bool WalkGait::isActive() {
    return active;
}

void WalkGait::reset() {
    stop();
    init();
}

