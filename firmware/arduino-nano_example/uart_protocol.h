#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <Arduino.h>

// UART communication settings
#define UART_BAUD_RATE 115200
#define UART_BUFFER_SIZE 128
#define UART_TIMEOUT_MS 100

// Message types
enum MessageType {
    MSG_GAIT_COMMAND = 0x01,
    MSG_SERVO_CONTROL = 0x02,
    MSG_EMERGENCY_STOP = 0x03,
    MSG_STATUS_REQUEST = 0x04,
    MSG_STATUS_RESPONSE = 0x05,
    MSG_CALIBRATION = 0x06,
    MSG_ACK = 0x07,
    MSG_ERROR = 0x08
};

// Gait types
enum GaitType {
    GAIT_IDLE = 0x00,
    GAIT_WALK = 0x01,
    GAIT_ROLL = 0x02,
    GAIT_CLIMB = 0x03,
    GAIT_STOP = 0x04
};

// Robot states
enum RobotState {
    STATE_IDLE = 0x00,
    STATE_WALKING = 0x01,
    STATE_ROLLING = 0x02,
    STATE_CLIMBING = 0x03,
    STATE_EMERGENCY_STOP = 0x04,
    STATE_ERROR = 0x05
};

// Message structure
struct GaitCommand {
    GaitType gait;
    float speed;      // 0.0 to 1.0
    float direction;  // -180 to 180 degrees
};

struct ServoControl {
    uint8_t leg;      // 0-3
    uint8_t joint;    // 0-2
    float angle;      // 0-180 degrees
};

struct StatusResponse {
    RobotState state;
    float batteryVoltage;
    float roll;       // IMU roll angle
    float pitch;      // IMU pitch angle
    bool emergencyStop;
    uint32_t uptime;  // milliseconds
};

class UARTProtocol {
public:
    UARTProtocol();
    
    // Initialization
    void init();
    
    // Message sending
    bool sendGaitCommand(GaitType gait, float speed = 0.5, float direction = 0.0);
    bool sendServoControl(uint8_t leg, uint8_t joint, float angle);
    bool sendEmergencyStop();
    bool sendStatusRequest();
    bool sendStatusResponse(const StatusResponse& status);
    bool sendAck();
    bool sendError(uint8_t errorCode);
    
    // Message receiving
    bool available();
    bool receiveGaitCommand(GaitCommand& cmd);
    bool receiveServoControl(ServoControl& ctrl);
    bool receiveEmergencyStop();
    bool receiveStatusRequest();
    bool receiveStatusResponse(StatusResponse& status);
    bool receiveAck();
    bool receiveError(uint8_t& errorCode);
    
    // Utility
    MessageType peekMessageType();
    void flush();
    
private:
    // Message encoding/decoding
    uint8_t calculateChecksum(const uint8_t* data, uint8_t length);
    bool validateChecksum(const uint8_t* data, uint8_t length);
    
    // Low-level send/receive
    bool sendMessage(MessageType type, const uint8_t* payload, uint8_t payloadLen);
    bool receiveMessage(MessageType& type, uint8_t* payload, uint8_t& payloadLen);
    
    uint8_t rxBuffer[UART_BUFFER_SIZE];
    uint8_t txBuffer[UART_BUFFER_SIZE];
};

#endif // UART_PROTOCOL_H

