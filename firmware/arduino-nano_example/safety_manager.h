#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include "imu_safety.h"
#include "battery_monitor.h"
#include "uart_protocol.h"

// Safety states
enum SafetyState {
    SAFETY_OK = 0,
    SAFETY_WARNING = 1,
    SAFETY_CRITICAL = 2,
    SAFETY_EMERGENCY = 3
};

class SafetyManager {
public:
    SafetyManager(IMUSafety* imu, BatteryMonitor* battery);
    
    // Initialization
    void init();
    
    // Monitoring
    void update();  // Call regularly to check all safety conditions
    SafetyState getState();
    
    // Individual checks
    bool checkIMU();
    bool checkBattery();
    bool checkEmergencyStop();
    
    // Emergency stop control
    void setEmergencyStop(bool state);
    bool isEmergencyStop();
    void clearEmergencyStop();  // Only clears if conditions are safe
    
    // Status
    bool isSafeToOperate();
    const char* getStateString();
    
private:
    IMUSafety* imu;
    BatteryMonitor* battery;
    bool emergencyStopFlag;
    SafetyState currentState;
    
    void evaluateState();
};

#endif // SAFETY_MANAGER_H

