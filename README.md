# Multi-Gait Quadruped Robot

This repository documents a hobby-level quadruped robot prototype developed within a limited timeframe.
The project focuses on safe power distribution, modular system architecture, and basic multi-gait
locomotion experiments.

## Project Goals
- Build a Wi-Fi controlled quadruped robot
- Develop and test basic walking, rolling, and climbing behaviors in simulation (Isaac Sim)
- Translate simulated gait logic into Arduino-based firmware
- Design a safe and reliable power system for multiple servo actuators
- Implement and validate the design on a physical prototype
- Document architecture and engineering design decisions clearly
  
## Project Status
This project is under active development. Features, firmware, and documentation
are being updated incrementally as the prototype evolves.

## System Overview
- ESP32-CAM: Wi-Fi communication, web-based control, and video streaming
- Arduino Nano: gait logic execution and motion control
- PCA9685: PWM servo driver for multi-servo actuation
- MPU6050: orientation feedback for safety supervision

## Current Status
- System architecture defined
- Power distribution and safety design completed
- Firmware and gait implementation under development

## Scope and Limitations
This is a hobby-level prototype intended for learning and experimentation.
The robot does not implement advanced autonomy, SLAM, or dynamic balance control.
The focus is on robustness, safety, and understanding system-level design trade-offs.

## License
MIT License
