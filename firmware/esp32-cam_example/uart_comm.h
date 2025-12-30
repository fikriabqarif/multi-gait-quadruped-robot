#ifndef UART_COMM_H
#define UART_COMM_H

#include <HardwareSerial.h>
#include <Arduino.h>

// UART settings (must match Arduino side)
#define UART_BAUD_RATE 115200
#define UART_RX_PIN 16  // ESP32-CAM UART RX pin
#define UART_TX_PIN 17  // ESP32-CAM UART TX pin
#define UART_BUFFER_SIZE 128
#define UART_TIMEOUT_MS 100

// Message types (must match Arduino protocol)
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

// Message structures
struct GaitCommand {
    GaitType gait;
    float speed;
    float direction;
};

struct StatusResponse {
    RobotState state;
    float batteryVoltage;
    float roll;
    float pitch;
    bool emergencyStop;
    uint32_t uptime;
};

class UARTComm {
public:
    UARTComm();
    
    // Initialization
    void init();
    
    // Message sending
    bool sendGaitCommand(GaitType gait, float speed = 0.5, float direction = 0.0);
    bool sendServoControl(uint8_t leg, uint8_t joint, float angle);
    bool sendEmergencyStop();
    bool sendStatusRequest();
    bool sendAck();
    
    // Message receiving
    bool available();
    bool receiveStatusResponse(StatusResponse& status);
    bool receiveAck();
    bool receiveError(uint8_t& errorCode);
    
    // Utility
    void flush();
    
private:
    HardwareSerial* uartSerial;
    
    // Message encoding/decoding
    uint8_t calculateChecksum(const uint8_t* data, uint8_t length);
    bool validateChecksum(const uint8_t* data, uint8_t length);
    bool sendMessage(MessageType type, const uint8_t* payload, uint8_t payloadLen);
    bool receiveMessage(MessageType& type, uint8_t* payload, uint8_t& payloadLen);
    
    uint8_t rxBuffer[UART_BUFFER_SIZE];
    uint8_t txBuffer[UART_BUFFER_SIZE];
};

#endif // UART_COMM_H

