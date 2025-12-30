#include "safety_manager.h"

SafetyManager::SafetyManager(IMUSafety* imu, BatteryMonitor* battery)
    : imu(imu), battery(battery),
      emergencyStopFlag(false),
      currentState(SAFETY_OK) {
}

void SafetyManager::init() {
    emergencyStopFlag = false;
    currentState = SAFETY_OK;
}

void SafetyManager::update() {
    // Update IMU and battery readings
    if (imu) {
        imu->update();
    }
    if (battery) {
        battery->update();
    }
    
    // Evaluate overall safety state
    evaluateState();
}

void SafetyManager::evaluateState() {
    bool imuOK = checkIMU();
    bool batteryOK = checkBattery();
    bool emergencyOK = !emergencyStopFlag;
    
    // Determine safety state
    if (!emergencyOK) {
        currentState = SAFETY_EMERGENCY;
    } else if (!imuOK || !batteryOK) {
        currentState = SAFETY_CRITICAL;
    } else if (battery->isLowBattery()) {
        currentState = SAFETY_WARNING;
    } else {
        currentState = SAFETY_OK;
    }
}

SafetyState SafetyManager::getState() {
    return currentState;
}

bool SafetyManager::checkIMU() {
    if (!imu || !imu->isInitialized()) {
        return true;  // If IMU not available, don't fail safety check
    }
    return !imu->isTiltExceeded();
}

bool SafetyManager::checkBattery() {
    if (!battery) {
        return true;  // If battery monitor not available, don't fail safety check
    }
    return battery->isBatteryOK();
}

bool SafetyManager::checkEmergencyStop() {
    return !emergencyStopFlag;
}

void SafetyManager::setEmergencyStop(bool state) {
    emergencyStopFlag = state;
    if (state) {
        currentState = SAFETY_EMERGENCY;
    } else {
        evaluateState();  // Re-evaluate after clearing
    }
}

bool SafetyManager::isEmergencyStop() {
    return emergencyStopFlag;
}

void SafetyManager::clearEmergencyStop() {
    // Only clear if conditions are safe
    if (checkIMU() && checkBattery()) {
        emergencyStopFlag = false;
        evaluateState();
    }
}

bool SafetyManager::isSafeToOperate() {
    return (currentState == SAFETY_OK || currentState == SAFETY_WARNING) && 
           !emergencyStopFlag;
}

const char* SafetyManager::getStateString() {
    switch (currentState) {
        case SAFETY_OK:
            return "OK";
        case SAFETY_WARNING:
            return "WARNING";
        case SAFETY_CRITICAL:
            return "CRITICAL";
        case SAFETY_EMERGENCY:
            return "EMERGENCY";
        default:
            return "UNKNOWN";
    }
}

