#include "servo_controller.h"
#include "uart_protocol.h"
#include "imu_safety.h"
#include "battery_monitor.h"
#include "safety_manager.h"
#include "leg_kinematics.h"
#include "gaits/walk_gait.h"
#include "gaits/roll_gait.h"

// Pin definitions
#define BATTERY_MONITOR_PIN A0
#define EMERGENCY_STOP_PIN 2  // Digital pin for emergency stop switch (NC)

// System objects
ServoController servos;
UARTProtocol uart;
IMUSafety imu;
BatteryMonitor battery(BATTERY_MONITOR_PIN);
SafetyManager safety(&imu, &battery);
LegKinematics kinematics;
WalkGait walkGait(&servos, &kinematics);
RollGait rollGait(&servos, &kinematics);

// System state
RobotState currentState = STATE_IDLE;
unsigned long systemStartTime;
unsigned long lastStatusUpdate;
const unsigned long STATUS_UPDATE_INTERVAL = 100;  // ms

// Emergency stop
bool emergencyStopHardware = false;

void setup() {
    // Initialize serial for debugging (separate from UART to ESP32)
    Serial.begin(115200);
    delay(1000);
    Serial.println("Quadruped Robot - Initializing...");
    
    systemStartTime = millis();
    lastStatusUpdate = millis();
    
    // Initialize emergency stop pin
    pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
    
    // Initialize subsystems
    Serial.println("Initializing servo controller...");
    if (!servos.init()) {
        Serial.println("ERROR: Servo controller initialization failed!");
        while(1) delay(1000);  // Halt on critical failure
    }
    
    Serial.println("Initializing UART protocol...");
    uart.init();
    
    Serial.println("Initializing IMU...");
    if (!imu.init()) {
        Serial.println("WARNING: IMU initialization failed - continuing without IMU");
    } else {
        Serial.println("Calibrating IMU (keep robot stationary)...");
        delay(2000);  // Give time for IMU to stabilize
        imu.calibrate();
        Serial.println("IMU calibrated");
    }
    
    Serial.println("Initializing battery monitor...");
    battery.init();
    
    Serial.println("Initializing safety manager...");
    safety.init();
    
    Serial.println("Initializing gaits...");
    walkGait.init();
    rollGait.init();
    
    // Set initial servo positions (neutral stance)
    servos.setAllServos(90.0);  // Center position
    delay(500);
    
    Serial.println("Initialization complete!");
    Serial.println("Waiting for commands from ESP32...");
}

void loop() {
    // Update safety systems
    safety.update();
    
    // Check hardware emergency stop
    emergencyStopHardware = !digitalRead(EMERGENCY_STOP_PIN);  // NC switch, LOW when pressed
    
    if (emergencyStopHardware) {
        safety.setEmergencyStop(true);
        handleEmergencyStop();
    }
    
    // Check safety conditions
    if (!safety.isSafeToOperate() && currentState != STATE_EMERGENCY_STOP) {
        handleEmergencyStop();
    }
    
    // Process UART commands
    processUARTCommands();
    
    // Update active gait
    updateGait();
    
    // Send status updates periodically
    if (millis() - lastStatusUpdate >= STATUS_UPDATE_INTERVAL) {
        sendStatusUpdate();
        lastStatusUpdate = millis();
    }
    
    // Small delay to prevent overwhelming the system
    delay(10);
}

void processUARTCommands() {
    if (!uart.available()) return;
    
    // Check for different message types
    GaitCommand gaitCmd;
    ServoControl servoCtrl;
    
    if (uart.receiveGaitCommand(gaitCmd)) {
        handleGaitCommand(gaitCmd);
        uart.sendAck();
    } else if (uart.receiveServoControl(servoCtrl)) {
        handleServoControl(servoCtrl);
        uart.sendAck();
    } else if (uart.receiveEmergencyStop()) {
        handleEmergencyStop();
        uart.sendAck();
    } else if (uart.receiveStatusRequest()) {
        sendStatusUpdate();
    }
}

void handleGaitCommand(const GaitCommand& cmd) {
    // Stop current gait
    walkGait.stop();
    rollGait.stop();
    
    // Check safety before starting new gait
    if (!safety.isSafeToOperate()) {
        currentState = STATE_EMERGENCY_STOP;
        return;
    }
    
    switch (cmd.gait) {
        case GAIT_IDLE:
        case GAIT_STOP:
            currentState = STATE_IDLE;
            walkGait.stop();
            rollGait.stop();
            // Return to neutral stance
            servos.setAllServos(90.0);
            break;
            
        case GAIT_WALK:
            currentState = STATE_WALKING;
            walkGait.start(cmd.speed, cmd.direction);
            break;
            
        case GAIT_ROLL:
            currentState = STATE_ROLLING;
            rollGait.start(cmd.speed, cmd.direction);
            break;
            
        case GAIT_CLIMB:
            // Climbing gait not yet implemented
            currentState = STATE_IDLE;
            Serial.println("Climbing gait not implemented");
            break;
            
        default:
            currentState = STATE_IDLE;
            break;
    }
}

void handleServoControl(const ServoControl& ctrl) {
    // Manual servo control (for calibration/testing)
    if (currentState == STATE_IDLE || currentState == STATE_EMERGENCY_STOP) {
        servos.setLegJointAngle(ctrl.leg, ctrl.joint, ctrl.angle);
    }
}

void handleEmergencyStop() {
    currentState = STATE_EMERGENCY_STOP;
    safety.setEmergencyStop(true);
    
    // Stop all gaits
    walkGait.stop();
    rollGait.stop();
    
    // Disable all servos
    servos.disableAll();
    
    Serial.println("EMERGENCY STOP ACTIVATED");
}

void updateGait() {
    if (currentState == STATE_WALKING) {
        walkGait.update();
    } else if (currentState == STATE_ROLLING) {
        rollGait.update();
    }
}

void sendStatusUpdate() {
    StatusResponse status;
    status.state = currentState;
    status.batteryVoltage = battery.getVoltage();
    status.roll = imu.getRoll();
    status.pitch = imu.getPitch();
    status.emergencyStop = safety.isEmergencyStop() || emergencyStopHardware;
    status.uptime = millis() - systemStartTime;
    
    uart.sendStatusResponse(status);
}

