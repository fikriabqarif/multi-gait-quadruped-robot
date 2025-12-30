#include "roll_gait.h"

RollGait::RollGait(ServoController* servos, LegKinematics* kinematics)
    : servos(servos), kinematics(kinematics),
      rollRadius(DEFAULT_ROLL_RADIUS),
      rollDuration(DEFAULT_ROLL_DURATION),
      currentSpeed(0.5),
      currentDirection(0.0),
      active(false),
      cycleStartTime(0),
      cycleProgress(0.0) {
    // Initialize leg phases (staggered for rolling motion)
    legStates[LEG_FRONT_LEFT].phase = 0.0;
    legStates[LEG_FRONT_RIGHT].phase = 0.25;
    legStates[LEG_BACK_RIGHT].phase = 0.5;
    legStates[LEG_BACK_LEFT].phase = 0.75;
    
    for (int i = 0; i < 4; i++) {
        legStates[i].extended = false;
    }
}

void RollGait::init() {
    // Set initial retracted position
    for (uint8_t leg = 0; leg < 4; leg++) {
        Point3D pos;
        pos.x = 0;
        pos.y = 0;
        pos.z = rollRadius;  // Retracted position
        
        JointAngles angles;
        if (kinematics->inverseKinematics(pos, angles)) {
            servos->setLegJointAngle(leg, JOINT_HIP, angles.hip);
            servos->setLegJointAngle(leg, JOINT_UPPER_LEG, angles.upperLeg);
            servos->setLegJointAngle(leg, JOINT_LOWER_LEG, angles.lowerLeg);
        }
    }
}

void RollGait::start(float speed, float direction) {
    setSpeed(speed);
    setDirection(direction);
    active = true;
    cycleStartTime = millis();
    cycleProgress = 0.0;
}

void RollGait::stop() {
    active = false;
    init();  // Return to retracted position
}

void RollGait::update() {
    if (!active) return;
    
    updateCycle();
    calculateLegPositions();
    applyLegPositions();
}

void RollGait::updateCycle() {
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - cycleStartTime;
    
    // Adjust duration based on speed
    uint16_t adjustedDuration = rollDuration / (currentSpeed + 0.1);
    
    cycleProgress = (float)elapsed / adjustedDuration;
    
    // Wrap around for continuous rolling
    if (cycleProgress >= 1.0) {
        cycleProgress = cycleProgress - (int)cycleProgress;
        cycleStartTime = currentTime - (elapsed % adjustedDuration);
    }
}

void RollGait::calculateLegPositions() {
    // Each leg follows a rolling trajectory based on its phase
    for (uint8_t leg = 0; leg < 4; leg++) {
        float legProgress = cycleProgress + legStates[leg].phase;
        if (legProgress >= 1.0) legProgress -= 1.0;
        
        legStates[leg].extended = (legProgress > 0.25 && legProgress < 0.75);
    }
}

void RollGait::applyLegPositions() {
    for (uint8_t leg = 0; leg < 4; leg++) {
        float legProgress = cycleProgress + legStates[leg].phase;
        if (legProgress >= 1.0) legProgress -= 1.0;
        
        Point3D target = getRollPosition(leg, legProgress);
        
        JointAngles angles;
        if (kinematics->inverseKinematics(target, angles)) {
            servos->setLegJointAngle(leg, JOINT_HIP, angles.hip);
            servos->setLegJointAngle(leg, JOINT_UPPER_LEG, angles.upperLeg);
            servos->setLegJointAngle(leg, JOINT_LOWER_LEG, angles.lowerLeg);
        }
    }
}

Point3D RollGait::getRollPosition(uint8_t leg, float progress) {
    Point3D pos;
    
    // Rolling motion: leg extends outward in a circular/elliptical pattern
    // Progress: 0.0 = retracted, 0.5 = fully extended, 1.0 = retracted again
    
    // Calculate extension distance (0 to rollRadius and back)
    float extension;
    if (progress < 0.5) {
        extension = rollRadius * (1.0 - cos(progress * 2.0 * PI));
    } else {
        extension = rollRadius * (1.0 - cos((1.0 - progress) * 2.0 * PI));
    }
    
    // Determine direction based on leg and current direction
    float baseAngle = getLegPhase(leg) * 2.0 * PI;
    float directionRad = currentDirection * PI / 180.0;
    float angle = baseAngle + directionRad;
    
    // Position leg in x-y plane (horizontal)
    pos.x = extension * cos(angle);
    pos.y = extension * sin(angle);
    
    // Z position: retracted legs are higher, extended legs are lower
    // This creates the rolling effect
    pos.z = rollRadius - extension * 0.5;  // Adjust factor for desired effect
    
    return pos;
}

float RollGait::getLegPhase(uint8_t leg) {
    return legStates[leg].phase;
}

void RollGait::setRollRadius(float radius) {
    rollRadius = radius;
}

void RollGait::setRollDuration(uint16_t duration) {
    rollDuration = duration;
}

void RollGait::setSpeed(float speed) {
    if (speed < 0.0) speed = 0.0;
    if (speed > 1.0) speed = 1.0;
    currentSpeed = speed;
}

void RollGait::setDirection(float direction) {
    while (direction > 360.0) direction -= 360.0;
    while (direction < 0.0) direction += 360.0;
    currentDirection = direction;
}

bool RollGait::isActive() {
    return active;
}

void RollGait::reset() {
    stop();
    init();
}

