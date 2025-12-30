#include "uart_comm.h"

UARTComm::UARTComm() {
    uartSerial = new HardwareSerial(2);  // Use UART2 on ESP32
}

void UARTComm::init() {
    uartSerial->begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    uartSerial->setTimeout(UART_TIMEOUT_MS);
}

uint8_t UARTComm::calculateChecksum(const uint8_t* data, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

bool UARTComm::validateChecksum(const uint8_t* data, uint8_t length) {
    if (length < 1) return false;
    uint8_t receivedChecksum = data[length - 1];
    uint8_t calculatedChecksum = calculateChecksum(data, length - 1);
    return receivedChecksum == calculatedChecksum;
}

bool UARTComm::sendMessage(MessageType type, const uint8_t* payload, uint8_t payloadLen) {
    const uint8_t START_BYTE = 0xAA;
    const uint8_t totalLen = 3 + payloadLen + 1;
    
    if (totalLen > UART_BUFFER_SIZE) return false;
    
    txBuffer[0] = START_BYTE;
    txBuffer[1] = (uint8_t)type;
    txBuffer[2] = payloadLen;
    
    if (payload && payloadLen > 0) {
        memcpy(&txBuffer[3], payload, payloadLen);
    }
    
    uint8_t checksum = calculateChecksum(txBuffer, 3 + payloadLen);
    txBuffer[3 + payloadLen] = checksum;
    
    uartSerial->write(txBuffer, totalLen);
    return true;
}

bool UARTComm::receiveMessage(MessageType& type, uint8_t* payload, uint8_t& payloadLen) {
    const uint8_t START_BYTE = 0xAA;
    
    if (!uartSerial->available()) return false;
    
    // Wait for start byte
    while (uartSerial->available() > 0 && uartSerial->peek() != START_BYTE) {
        uartSerial->read();
    }
    
    if (!uartSerial->available()) return false;
    uartSerial->read();  // Read start byte
    
    if (uartSerial->available() < 2) return false;
    type = (MessageType)uartSerial->read();
    payloadLen = uartSerial->read();
    
    if (payloadLen > UART_BUFFER_SIZE - 4) return false;
    
    uint8_t bytesToRead = payloadLen + 1;
    uint8_t bytesRead = 0;
    unsigned long startTime = millis();
    
    while (bytesRead < bytesToRead) {
        if (uartSerial->available()) {
            rxBuffer[bytesRead++] = uartSerial->read();
        }
        if (millis() - startTime > UART_TIMEOUT_MS) {
            return false;
        }
        delay(1);
    }
    
    if (!validateChecksum(rxBuffer, bytesToRead)) {
        return false;
    }
    
    if (payload && payloadLen > 0) {
        memcpy(payload, rxBuffer, payloadLen);
    }
    
    return true;
}

bool UARTComm::sendGaitCommand(GaitType gait, float speed, float direction) {
    uint8_t payload[4];
    payload[0] = (uint8_t)gait;
    payload[1] = (uint8_t)(speed * 255.0);
    
    int16_t dirInt = (int16_t)(direction * 100.0);
    payload[2] = (dirInt >> 8) & 0xFF;
    payload[3] = dirInt & 0xFF;
    
    return sendMessage(MSG_GAIT_COMMAND, payload, 4);
}

bool UARTComm::sendServoControl(uint8_t leg, uint8_t joint, float angle) {
    uint8_t payload[3];
    payload[0] = leg;
    payload[1] = joint;
    payload[2] = (uint8_t)angle;
    
    return sendMessage(MSG_SERVO_CONTROL, payload, 3);
}

bool UARTComm::sendEmergencyStop() {
    return sendMessage(MSG_EMERGENCY_STOP, nullptr, 0);
}

bool UARTComm::sendStatusRequest() {
    return sendMessage(MSG_STATUS_REQUEST, nullptr, 0);
}

bool UARTComm::sendAck() {
    return sendMessage(MSG_ACK, nullptr, 0);
}

bool UARTComm::available() {
    return uartSerial->available() > 0;
}

bool UARTComm::receiveStatusResponse(StatusResponse& status) {
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

bool UARTComm::receiveAck() {
    MessageType type;
    uint8_t payload[1];
    uint8_t payloadLen;
    
    if (!receiveMessage(type, payload, payloadLen)) return false;
    return type == MSG_ACK;
}

bool UARTComm::receiveError(uint8_t& errorCode) {
    MessageType type;
    uint8_t payload[1];
    uint8_t payloadLen;
    
    if (!receiveMessage(type, payload, payloadLen)) return false;
    if (type != MSG_ERROR || payloadLen != 1) return false;
    
    errorCode = payload[0];
    return true;
}

void UARTComm::flush() {
    uartSerial->flush();
}

