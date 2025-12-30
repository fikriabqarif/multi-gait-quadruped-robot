# Power System Design

The robot is powered by a 2S Li-ion battery.

## Power Distribution
- Two Mini560 buck converters supply servo power
  - Each converter powers six servos
- A separate buck converter supplies logic power
  - Arduino Nano
  - ESP32-CAM
  - MPU6050

## Grounding
All subsystems share a common ground reference using a star-ground topology.
High-current servo return paths are kept separate from logic ground paths.

## Safety Features
- Main power switch
- Latching emergency stop switch that disconnects servo power
- Battery voltage display for monitoring discharge level
