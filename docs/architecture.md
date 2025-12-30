# System Architecture

The system uses a distributed control architecture to separate communication
tasks from time-critical motion control.

## Control Architecture
- ESP32-CAM handles:
  - Wi-Fi communication
  - Web-based control interface
  - Video streaming
- Arduino Nano handles:
  - Gait execution
  - Servo sequencing
  - Safety logic

Communication between the ESP32-CAM and Arduino Nano is performed via UART.

## Actuation
- PCA9685 provides PWM signals for up to 16 servo channels
- Servo power is supplied externally and not through the PCA9685 board

## Sensing
- MPU6050 IMU provides roll and pitch information
- IMU data is used for safety supervision, not closed-loop balance control
