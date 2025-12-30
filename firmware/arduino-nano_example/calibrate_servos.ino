/*
 * Servo Calibration Sketch
 * 
 * This sketch helps you find the min/max pulse widths for each servo.
 * Use the serial monitor to interactively calibrate servos.
 * 
 * Commands:
 *   s <channel> - Select servo channel (0-11)
 *   m <pulse>   - Set min pulse width (microseconds)
 *   M <pulse>   - Set max pulse width (microseconds)
 *   c <pulse>   - Set center pulse width (microseconds)
 *   t <angle>   - Test angle (0-180 degrees)
 *   p <pulse>   - Test pulse width directly (microseconds)
 *   r            - Read current limits
 *   w            - Write calibration to EEPROM (optional)
 *   h            - Show help
 */

#include "servo_controller.h"

ServoController servos;
uint8_t currentChannel = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== Servo Calibration Tool ===");
    Serial.println("Initializing servo controller...");
    
    if (!servos.init()) {
        Serial.println("ERROR: Failed to initialize servo controller!");
        while(1) delay(1000);
    }
    
    Serial.println("Ready for calibration commands.");
    Serial.println("Type 'h' for help");
    Serial.println();
    
    printHelp();
    Serial.print("Current channel: ");
    Serial.println(currentChannel);
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 's':  // Select channel
                selectChannel();
                break;
            case 'm':  // Set min pulse
                setMinPulse();
                break;
            case 'M':  // Set max pulse
                setMaxPulse();
                break;
            case 'c':  // Set center pulse
                setCenterPulse();
                break;
            case 't':  // Test angle
                testAngle();
                break;
            case 'p':  // Test pulse width
                testPulseWidth();
                break;
            case 'r':  // Read limits
                readLimits();
                break;
            case 'w':  // Sweep test
                sweepTest();
                break;
            case 'h':  // Help
                printHelp();
                break;
            case '\n':
            case '\r':
                // Ignore newlines
                break;
            default:
                Serial.print("Unknown command: ");
                Serial.println(cmd);
                Serial.println("Type 'h' for help");
                break;
        }
    }
}

void selectChannel() {
    Serial.print("Enter channel (0-11): ");
    while (!Serial.available()) delay(10);
    
    int channel = Serial.parseInt();
    if (channel >= 0 && channel < 12) {
        currentChannel = channel;
        Serial.print("Selected channel: ");
        Serial.println(currentChannel);
    } else {
        Serial.println("Invalid channel!");
    }
    Serial.flush();
}

void setMinPulse() {
    Serial.print("Enter min pulse width (us): ");
    while (!Serial.available()) delay(10);
    
    int pulse = Serial.parseInt();
    if (pulse > 0 && pulse < 3000) {
        uint16_t maxPulse;
        servos.getServoLimits(currentChannel, (uint16_t&)pulse, maxPulse);
        servos.setServoLimits(currentChannel, pulse, maxPulse);
        Serial.print("Set min pulse to: ");
        Serial.println(pulse);
    } else {
        Serial.println("Invalid pulse width!");
    }
    Serial.flush();
}

void setMaxPulse() {
    Serial.print("Enter max pulse width (us): ");
    while (!Serial.available()) delay(10);
    
    int pulse = Serial.parseInt();
    if (pulse > 0 && pulse < 3000) {
        uint16_t minPulse;
        servos.getServoLimits(currentChannel, minPulse, (uint16_t&)pulse);
        servos.setServoLimits(currentChannel, minPulse, pulse);
        Serial.print("Set max pulse to: ");
        Serial.println(pulse);
    } else {
        Serial.println("Invalid pulse width!");
    }
    Serial.flush();
}

void setCenterPulse() {
    Serial.print("Enter center pulse width (us): ");
    while (!Serial.available()) delay(10);
    
    int pulse = Serial.parseInt();
    if (pulse > 0 && pulse < 3000) {
        servos.setServoCenter(currentChannel, pulse);
        Serial.print("Set center pulse to: ");
        Serial.println(pulse);
    } else {
        Serial.println("Invalid pulse width!");
    }
    Serial.flush();
}

void testAngle() {
    Serial.print("Enter angle (0-180): ");
    while (!Serial.available()) delay(10);
    
    float angle = Serial.parseFloat();
    if (angle >= 0 && angle <= 180) {
        servos.setAngle(currentChannel, angle);
        Serial.print("Set angle to: ");
        Serial.println(angle);
    } else {
        Serial.println("Invalid angle!");
    }
    Serial.flush();
}

void testPulseWidth() {
    Serial.print("Enter pulse width (us): ");
    while (!Serial.available()) delay(10);
    
    int pulse = Serial.parseInt();
    if (pulse > 0 && pulse < 3000) {
        servos.setPulseWidth(currentChannel, pulse);
        Serial.print("Set pulse width to: ");
        Serial.println(pulse);
    } else {
        Serial.println("Invalid pulse width!");
    }
    Serial.flush();
}

void readLimits() {
    uint16_t minPulse, maxPulse;
    servos.getServoLimits(currentChannel, minPulse, maxPulse);
    
    Serial.print("Channel ");
    Serial.print(currentChannel);
    Serial.println(" limits:");
    Serial.print("  Min: ");
    Serial.print(minPulse);
    Serial.println(" us");
    Serial.print("  Max: ");
    Serial.print(maxPulse);
    Serial.println(" us");
}

void sweepTest() {
    Serial.println("Sweeping servo...");
    
    // Sweep from min to max angle
    for (float angle = 0; angle <= 180; angle += 1) {
        servos.setAngle(currentChannel, angle);
        delay(20);
    }
    
    for (float angle = 180; angle >= 0; angle -= 1) {
        servos.setAngle(currentChannel, angle);
        delay(20);
    }
    
    // Return to center
    servos.setAngle(currentChannel, 90.0);
    Serial.println("Sweep complete");
}

void printHelp() {
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  s <channel>  - Select servo channel (0-11)");
    Serial.println("  m <pulse>    - Set min pulse width (us)");
    Serial.println("  M <pulse>    - Set max pulse width (us)");
    Serial.println("  c <pulse>    - Set center pulse width (us)");
    Serial.println("  t <angle>    - Test angle (0-180 degrees)");
    Serial.println("  p <pulse>    - Test pulse width directly (us)");
    Serial.println("  r            - Read current limits");
    Serial.println("  w            - Sweep test (0-180 degrees)");
    Serial.println("  h            - Show this help");
    Serial.println();
}

