# Multi-Gait Quadruped Robot

This repository documents a **hobby-level quadruped robot prototype** developed within a limited timeframe.  
The project focuses on **safe power distribution**, **modular system architecture**, and **basic multi-gait locomotion experiments**, rather than high-performance or full autonomy.

---

## Project Goals
- Build a **Wi-Fi controlled quadruped robot** with live video feedback  
- Explore **walking, rolling, and climbing behaviors** in simulation (Isaac Sim)  
- Translate simulated gait logic into **Arduino-based firmware**  
- Design a **safe and robust power system** for multi-servo actuation  
- Clearly document architecture and engineering trade-offs  

---

## System Overview
- **ESP32-CAM** — Wi-Fi communication, web-based control, video streaming  
- **Arduino Nano** — gait execution, motion sequencing, safety logic  
- **PCA9685** — hardware PWM for multi-servo control  
- **MPU6050 IMU** — roll and pitch monitoring for safety supervision  

The system uses a **distributed control architecture**, separating communication tasks from time-critical motion control.

---

## Power and Safety
- 2S Li-ion battery as the main power source  
- Dual Mini560 buck converters for servo power distribution  
- Separate buck converter for logic electronics  
- Star-ground topology to reduce noise and resets  

**Safety features include:**
- Main power switch  
- Latching emergency stop (servo power cut)  
- Battery voltage monitoring  
- IMU-based motion abort logic  

---

## Project Status
The project is **under active development**.  
Firmware, hardware configuration, and documentation are updated incrementally as the prototype evolves.

---

## Scope and Limitations
This is a **learning-oriented prototype**.  
The robot does **not** implement SLAM, autonomous navigation, vision-based perception, or dynamic balance control.  
The focus is on **robustness, safety, and system-level understanding** using affordable hardware.

---

## References
- Feng Q., et al. *Rolling Locomotion Control of a Biologically Inspired Quadruped Robot Based on Energy Compensation*  
- G. Bledt, et al. *Design and Control of a Robust, Dynamic Quadruped Robot (MIT Cheetah 3)*  
- anoochit/arduino-quadruped-robot — https://github.com/anoochit/arduino-quadruped-robot  

---

## License
MIT License
