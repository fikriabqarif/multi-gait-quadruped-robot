#include "uart_protocol.h"

UARTProtocol::UARTProtocol() {
    // Constructor
}

void UARTProtocol::init() {
    Serial.begin(UART_BAUD_RATE);
    Serial.setTimeout(UART_TIMEOUT_MS);
}

uint8_t UARTProtocol::calculateChecksum(const uint8_t* data, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

bool UARTProtocol::validateChecksum(const uint8_t* data, uint8_t length) {
    if (length < 1) return false;
    uint8_t receivedChecksum = data[length - 1];
    uint8_t calculatedChecksum = calculateChecksum(data, length - 1);
    return receivedChecksum == calculatedChecksum;
}

bool UARTProtocol::sendMessage(MessageType type, const uint8_t* payload, uint8_t payloadLen) {
    // Message format: [START_BYTE][TYPE][LENGTH][PAYLOAD...][CHECKSUM]
    // START_BYTE = 0xAA
    const uint8_t START_BYTE = 0xAA;
    const uint8_t totalLen = 3 + payloadLen + 1; // START + TYPE + LEN + PAYLOAD + CHECKSUM
    
    if (totalLen > UART_BUFFER_SIZE) return false;
    
    txBuffer[0] = START_BYTE;
    txBuffer[1] = (uint8_t)type;
    txBuffer[2] = payloadLen;
    
    // Copy payload
    for (uint8_t i = 0; i < payloadLen; i++) {
        txBuffer[3 + i] = payload[i];
    }
    
    // Calculate and append checksum
    uint8_t checksum = calculateChecksum(txBuffer, 3 + payloadLen);
    txBuffer[3 + payloadLen] = checksum;
    
    // Send message
    Serial.write(txBuffer, totalLen);
    return true;
}

bool UARTProtocol::receiveMessage(MessageType& type, uint8_t* payload, uint8_t& payloadLen) {
    const uint8_t START_BYTE = 0xAA;
    
    // Wait for start byte
    if (!Serial.available()) return false;
    
    uint8_t startByte = Serial.read();
    if (startByte != START_BYTE) {
        // Try to find start byte in buffer
        while (Serial.available() > 0 && Serial.peek() != START_BYTE) {
            Serial.read(); // Discard
        }
        if (!Serial.available()) return false;
        startByte = Serial.read();
    }
    
    // Read type and length
    if (Serial.available() < 2) return false;
    type = (MessageType)Serial.read();
    payloadLen = Serial.read();
    
    if (payloadLen > UART_BUFFER_SIZE - 4) return false; // Too large
    
    // Read payload + checksum
    uint8_t bytesToRead = payloadLen + 1;
    uint8_t bytesRead = 0;
    unsigned long startTime = millis();
    
    while (bytesRead < bytesToRead) {
        if (Serial.available()) {
            rxBuffer[bytesRead++] = Serial.read();
        }
        if (millis() - startTime > UART_TIMEOUT_MS) {
            return false; // Timeout
        }
    }
    
    // Validate checksum
    if (!validateChecksum(rxBuffer, bytesToRead)) {
        return false;
    }
    
    // Copy payload (excluding checksum)
    for (uint8_t i = 0; i < payloadLen; i++) {
        payload[i] = rxBuffer[i];
    }
    
    return true;
}

bool UARTProtocol::sendGaitCommand(GaitType gait, float speed, float direction) {
    uint8_t payload[5];
    payload[0] = (uint8_t)gait;
    
    // Pack speed (0.0-1.0) as uint8_t (0-255)
    payload[1] = (uint8_t)(speed * 255.0);
    
    // Pack direction (-180 to 180) as int16_t
    int16_t dirInt = (int16_t)(direction * 100.0); // Store as hundredths
    payload[2] = (dirInt >> 8) & 0xFF;
    payload[3] = dirInt & 0xFF;
    
    return sendMessage(MSG_GAIT_COMMAND, payload, 4);
}

bool UARTProtocol::sendServoControl(uint8_t leg, uint8_t joint, float angle) {
    uint8_t payload[3];
    payload[0] = leg;
    payload[1] = joint;
    // Pack angle (0-180) as uint8_t (0-180 directly)
    payload[2] = (uint8_t)angle;
    
    return sendMessage(MSG_SERVO_CONTROL, payload, 3);
}

bool UARTProtocol::sendEmergencyStop() {
    return sendMessage(MSG_EMERGENCY_STOP, nullptr, 0);
}

bool UARTProtocol::sendStatusRequest() {
    return sendMessage(MSG_STATUS_REQUEST, nullptr, 0);
}

bool UARTProtocol::sendStatusResponse(const StatusResponse& status) {
    uint8_t payload[16];
    payload[0] = (uint8_t)status.state;
    
    // Pack battery voltage (0-10V) as uint16_t (0-10000 for mV)
    uint16_t voltage = (uint16_t)(status.batteryVoltage * 1000.0);
    payload[1] = (voltage >> 8) & 0xFF;
    payload[2] = voltage & 0xFF;
    
    // Pack roll and pitch (-90 to 90 degrees) as int16_t (hundredths)
    int16_t rollInt = (int16_t)(status.roll * 100.0);
    payload[3] = (rollInt >> 8) & 0xFF;
    payload[4] = rollInt & 0xFF;
    
    int16_t pitchInt = (int16_t)(status.pitch * 100.0);
    payload[5] = (pitchInt >> 8) & 0xFF;
    payload[6] = pitchInt & 0xFF;
    
    payload[7] = status.emergencyStop ? 1 : 0;
    
    // Pack uptime as uint32_t
    payload[8] = (status.uptime >> 24) & 0xFF;
    payload[9] = (status.uptime >> 16) & 0xFF;
    payload[10] = (status.uptime >> 8) & 0xFF;
    payload[11] = status.uptime & 0xFF;
    
    return sendMessage(MSG_STATUS_RESPONSE, payload, 12);
}

bool UARTProtocol::sendAck() {
    return sendMessage(MSG_ACK, nullptr, 0);
}

bool UARTProtocol::sendError(uint8_t errorCode) {
    uint8_t payload[1];
    payload[0] = errorCode;
    return sendMessage(MSG_ERROR, payload, 1);
}

bool UARTProtocol::available() {
    return Serial.available() > 0;
}

bool UARTProtocol::receiveGaitCommand(GaitCommand& cmd) {
    MessageType type;
    uint8_t payload[4];
    uint8_t payloadLen;
    
    if (!receiveMessage(type, payload, payloadLen)) return false;
    if (type != MSG_GAIT_COMMAND || payloadLen != 4) return false;
    
    cmd.gait = (GaitType)payload[0];
    cmd.speed = payload[1] / 255.0;
    
    int16_t dirInt = (payload[2] << 8) | payload[3];
    cmd.direction = dirInt / 100.0;
    
    return true;
}

bool UARTProtocol::receiveServoControl(ServoControl& ctrl) {
    MessageType type;
    uint8_t payload[3];
    uint8_t payloadLen;
    
    if (!receiveMessage(type, payload, payloadLen)) return false;
    if (type != MSG_SERVO_CONTROL || payloadLen != 3) return false;
    
    ctrl.leg = payload[0];
    ctrl.joint = payload[1];
    ctrl.angle = payload[2];
    
    return true;
}

bool UARTProtocol::receiveEmergencyStop() {
    MessageType type;
    uint8_t payload[1];
    uint8_t payloadLen;
    
    if (!receiveMessage(type, payload, payloadLen)) return false;
    return type == MSG_EMERGENCY_STOP;
}

bool UARTProtocol::receiveStatusRequest() {
    MessageType type;
    uint8_t payload[1];
    uint8_t payloadLen;
    
    if (!receiveMessage(type, payload, payloadLen)) return false;
    return type == MSG_STATUS_REQUEST;
}

bool UARTProtocol::receiveStatusResponse(StatusResponse& status) {
    MessageType type;
    uint8_t payload[12];
    uint8_t payloadLen;
    
    if (!receiveMessage(type, payload, payloadLen)) return false;
    if (type != MSG_STATUS_RESPONSE || payloadLen != 12) return false;
    
    status.state = (RobotState)payload[0];
    
    uint16_t voltage = (payload[1] << 8) | payload[2];
    status.batteryVoltage = voltage / 1000.0;
    
    int16_t rollInt = (payload[3] << 8) | payload[4];
    status.roll = rollInt / 100.0;
    
    int16_t pitchInt = (payload[5] << 8) | payload[6];
    status.pitch = pitchInt / 100.0;
    
    status.emergencyStop = (payload[7] != 0);
    
    status.uptime = ((uint32_t)payload[8] << 24) | 
                    ((uint32_t)payload[9] << 16) | 
                    ((uint32_t)payload[10] << 8) | 
                    payload[11];
    
    return true;
}

bool UARTProtocol::receiveAck() {
    MessageType type;
    uint8_t payload[1];
    uint8_t payloadLen;
    
    if (!receiveMessage(type, payload, payloadLen)) return false;
    return type == MSG_ACK;
}

bool UARTProtocol::receiveError(uint8_t& errorCode) {
    MessageType type;
    uint8_t payload[1];
    uint8_t payloadLen;
    
    if (!receiveMessage(type, payload, payloadLen)) return false;
    if (type != MSG_ERROR || payloadLen != 1) return false;
    
    errorCode = payload[0];
    return true;
}

MessageType UARTProtocol::peekMessageType() {
    // This is a simplified peek - in practice, you'd need to buffer
    // For now, just check if data is available
    if (!Serial.available()) return (MessageType)0;
    
    // Look for start byte
    while (Serial.available() > 1 && Serial.peek() != 0xAA) {
        Serial.read(); // Discard
    }
    
    if (Serial.available() < 2) return (MessageType)0;
    
    // Read start byte and type, then put them back
    // Note: Arduino Serial doesn't support unget, so we can't truly peek
    // This is a limitation - in practice, you'd use a ring buffer
    return (MessageType)0; // Placeholder
}

void UARTProtocol::flush() {
    Serial.flush();
}

