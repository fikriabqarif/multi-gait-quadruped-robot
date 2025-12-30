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

## Related Work

### Academic References
1. **Feng Q., et al.**  
   *Rolling Locomotion Control of a Biologically Inspired Quadruped Robot Based on Energy Compensation.*  
   This work provides the primary inspiration for rolling locomotion concepts explored in this project.

2. **G. Bledt, et al.**  
   *Design and Control of a Robust, Dynamic Quadruped Robot (MIT Cheetah 3).*  
   This paper offers insight into quadruped robot design and locomotion control strategies at a system level.

3. **A. Spröwitz, et al.**  
   *Towards Dynamic Trot Gait Locomotion: Design, Control, and Experiments with Cheetah-cub.*  
   This work discusses gait generation and experimental evaluation of quadruped walking behaviors.

### Open-Source Reference
- **anoochit/arduino-quadruped-robot**  
  https://github.com/anoochit/arduino-quadruped-robot  
  A practical Arduino-based quadruped implementation using hobby servos, referenced for firmware
  structure and multi-servo control patterns.

## License
This project is licensed under the MIT License.
