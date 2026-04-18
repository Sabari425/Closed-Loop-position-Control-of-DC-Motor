# Closed Loop Position Control of a DC Motor

<div align="center">

![Project Banner](https://raw.githubusercontent.com/Sabari425/Portfolio/refs/heads/main/Robot%20arm-rafiki.png)

**Precision PID-Based Position Control for Robotic Joint Applications**

[![ESP32](https://img.shields.io/badge/ESP32--S3-Controller-blue?style=for-the-badge&logo=espressif)](https://www.espressif.com/)
[![PID](https://img.shields.io/badge/PID-Control-00d0ff?style=for-the-badge)](https://en.wikipedia.org/wiki/PID_controller)
[![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Completed-success?style=for-the-badge)]()
[![Commits](https://img.shields.io/badge/Commits-2-blue?style=for-the-badge)]()

</div>

---

## Table of Contents

- [Overview](#overview)
- [Live Demo & Documentation](#live-demo--documentation)
- [Key Performance Metrics](#key-performance-metrics)
- [System Architecture](#system-architecture)
- [Hardware Components](#hardware-components)
- [Control Algorithm](#control-algorithm)
- [Installation & Setup](#installation--setup)
- [Performance Analysis](#performance-analysis)
- [Future Improvements](#future-improvements)
- [Conclusion](#conclusion)

---

## Overview

This project implements a **high-precision closed-loop position controller** for a standard DC gear motor, transforming it into a **Smart Joint** suitable for robotic applications. The system uses an **ESP32-S3** microcontroller as the computational core, an **AS5600 12-bit magnetic encoder** for absolute position feedback, and an **L298N motor driver** for PWM-based actuation.

### Key Objectives

- Achieve sub-degree positioning accuracy
- Minimize overshoot and settling time
- Compensate for mechanical non-linearities (backlash, friction)
- Provide real-time monitoring and tuning capabilities

### Highlights

| Metric | Value |
|:-------|:-----:|
| Encoder Resolution | 0.087° (12-bit magnetic) |
| Rise Time | 0.8 seconds |
| Overshoot | < 5% |
| Settling Time | 1.8 seconds (±2.5° tolerance) |
| Steady-State Error | < 1° |

---

## Live Demo & Documentation

| Resource | Link |
|:---------|:-----|
| 📹 Quick Demo | [DEMO.mp4](DEMO.mp4) |
| 📹 Extended Demo | [Position_Control_of_Robotic_arm_video.mp4](Position_Control_of_Robotic_arm_video.mp4) |
| 📽️ Presentation | [Position Control of Robotic arm.pptx](Position%20Control%20of%20Robotic%20arm.pptx) |
| 📖 Full Documentation | [Position Control of Robotic arm.pdf](Position%20Control%20of%20Robotic%20arm.pdf) |

---

## Key Performance Metrics

| Metric | Value | Rating |
|:-------|:-----:|:------:|
| Rise Time | 0.8 seconds | ⭐⭐⭐⭐⭐ |
| Overshoot | ~5% | ⭐⭐⭐⭐ |
| Settling Time | 1.8 seconds | ⭐⭐⭐⭐ |
| Steady-State Error | < 1° | ⭐⭐⭐⭐⭐ |
| Encoder Resolution | 0.087° (12-bit) | ⭐⭐⭐⭐⭐ |
| Control Frequency | 100 Hz | ⭐⭐⭐⭐ |

---

## System Architecture

<div align="center">
<img src="Architecture.jpeg" alt="System Block Diagram" width="800"/>
</div>

### Control Flow Diagram

<div align="center">
<img src="Sample_flow_diagram.jpeg" alt="Sample Flow Diagram" width="800"/>
</div>

**Algorithm Steps:**

| Step | Action | Description |
|:----:|:-------|:------------|
| 1 | Initialize | Configure I2C, PWM, Serial, calibrate encoder zero |
| 2 | Read Target | Receive desired angle from GUI via Serial |
| 3 | Read Position | Query AS5600 via I2C for absolute angle (0-360°) |
| 4 | Compute Error | Calculate shortest-path angular error with wrap-around |
| 5 | PID Calculation | Execute discrete PID with anti-windup and limiting |
| 6 | Deadband Compensation | Apply minimum PWM to overcome static friction |
| 7 | Slow-Zone Control | Reduce gain when near target for smooth settling |
| 8 | Output PWM | Send calculated value to L298N motor driver |
| 9 | Telemetry | Stream position, error, PWM data to GUI at 100Hz |
| 10 | Loop | Repeat continuously from step 2 |

### Circuit Schematic

<div align="center">
<img src="Schematic_Diagram_for_project_ref.png" alt="Circuit Schematic" width="800"/>
</div>

---

## Hardware Components

| Component | Specification | Purpose |
|:----------|:-------------|:--------|
| ESP32-S3 | Dual-core Xtensa LX7 @ 240MHz | Main controller, PID, I2C/UART |
| AS5600 | 12-bit Magnetic Encoder | Absolute angle feedback (0.087°) |
| L298N | Dual H-Bridge Driver | PWM motor actuation (2A max) |
| DC Gear Motor | 12V, 100 RPM | Actuator with 1:50 gear reduction |
| Neodymium Magnet | 6mm diametrically magnetized | Encoder target on motor shaft |
| LiPo Battery | 11.1V, 2200mAh | Power supply (3S, ~30min runtime) |

### Pin Configuration

| ESP32-S3 Pin | Connected To | Signal Type | Function |
|:------------:|:------------:|:-----------:|:--------:|
| GPIO 21 | AS5600 SDA | I2C Data | Bi-directional data |
| GPIO 22 | AS5600 SCL | I2C Clock | Serial clock output |
| GPIO 25 | L298N ENA | PWM Output | Enable A channel (0-255) |
| GPIO 26 | L298N IN1 | Digital Out | Direction control bit 1 |
| GPIO 27 | L298N IN2 | Digital Out | Direction control bit 2 |
| 3.3V | AS5600 VCC | Power | 3.3V regulated |
| GND | Common | Ground | Shared reference |
| VIN (5V) | L298N 5V | Power | Logic supply |

---

## Control Algorithm

### PID Controller Equation
 --> u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·(de/dt)

| Symbol | Description | Value |
|:------:|:------------|:-----:|
| u(t) | Control output (PWM) | -255 to +255 |
| e(t) | Error = Target - Measured | Variable |
| Kp | Proportional gain | 1.2 |
| Ki | Integral gain | 0 (disabled) |
| Kd | Derivative gain | 0.35 |
| dt | Sample time | 10ms (100Hz) |

### Tuned PID Parameters

| Parameter | Value | Effect | Tuning Method |
|:---------:|:-----:|:-------|:--------------|
| Kp | 1.2 | Main driving force | Increased until oscillation, reduced 50% |
| Ki | 0 | Disabled | Sufficient PD performance |
| Kd | 0.35 | Damping | Increased until overshoot <5% |
| Sample Time | 10ms | Control frequency | Based on motor time constant |
| Output Limit | ±255 | PWM range | Hardware resolution limit |

### Advanced Control Features

| Feature | Implementation | Benefit |
|:--------|:--------------|:--------|
| Shortest-Path Error | `if (error > 180) error -= 360;` | Optimal direction (≤180°) |
| Anti-Windup | Integral clamping | Prevents term explosion |
| Deadband Compensation | Minimum PWM threshold (~15) | Overcomes static friction |
| Slow-Zone Control | 50% gain when error < 10° | Smooth settling |
| Wrap-Around Handling | 0-360° continuous rotation | No discontinuity at boundary |

---

## Installation & Setup

### Hardware Assembly

1. **Encoder Mounting**
   - Attach 6mm diametrically magnetized magnet to motor shaft
   - Position AS5600 sensor centered over magnet (1mm air gap)
   - Secure with 3D-printed bracket or standoffs

2. **Wiring Instructions** *(Reference: Schematic_Diagram_for_project_ref.png)*

3. ESP32-S3 GPIO21 ───► AS5600 SDA ESP32-S3 GPIO25 ───► L298N ENA (PWM)
ESP32-S3 GPIO22 ───► AS5600 SCL ESP32-S3 GPIO26 ───► L298N IN1
ESP32-S3 3.3V ───► AS5600 VCC ESP32-S3 GPIO27 ───► L298N IN2
ESP32-S3 GND ───► AS5600 GND ESP32-S3 GND ───► L298N GND

Battery 11.1V (+) ───► L298N 12V Input L298N OUT1 ───────► Motor Terminal 1
Battery GND (-) ───► L298N GND L298N OUT2 ───────► Motor Terminal 2


## Performance Analysis

### Step Response Characteristics

| Metric | Measured Value | Target Specification | Status |
|:-------|:-------------:|:--------------------:|:------:|
| Rise Time (10% → 90%) | 0.8 seconds | < 1.0 seconds | ✅ PASS |
| Peak Overshoot | 5.0% | < 10% | ✅ PASS |
| Settling Time (±2.5°) | 1.8 seconds | < 2.0 seconds | ✅ PASS |
| Steady-State Error | < 1° | < 2° | ✅ PASS |
| Maximum Tracking Speed | 180°/sec | > 120°/sec | ✅ PASS |

### Error Distribution Analysis

| Error Range | Occurrence Frequency | Cumulative Percentage |
|:-----------:|:--------------------:|:---------------------:|
| 0.0° - 0.5° | ████████████████████ (45%) | 45% |
| 0.5° - 1.0° | ██████████████ (35%) | 80% |
| 1.0° - 2.0° | ██████ (15%) | 95% |
| > 2.0° | ██ (5%) | 100% |

### Frequency Domain Analysis

| Parameter | Value | Interpretation |
|:----------|:-----:|:--------------|
| Bandwidth (-3dB) | 2.5 Hz | System can track inputs up to 2.5Hz |
| Phase Margin | ~60° | Good stability - well above 45° minimum |
| Gain Margin | 8 dB | Robust to gain variations |
| Resonance Peak | None | No oscillatory modes detected |

### Disturbance Rejection

| Disturbance Type | Recovery Time | Max Deviation |
|:-----------------|:-------------:|:-------------:|
| External Torque (50% load) | 0.4 seconds | 8° |
| Sudden Setpoint Change (180°) | 1.2 seconds | 5% overshoot |
| Power Supply Sag | No effect | <0.5° |

---

## Future Improvements

### Short-Term Roadmap (v1.1)
- [ ] Adaptive PID - Auto-tuning based on error magnitude and velocity
- [ ] Velocity Feed-Forward - Improved trajectory tracking accuracy
- [ ] Bluetooth/BLE Interface - Wireless control from mobile devices
- [ ] Web Dashboard - Browser-based monitoring using WebSerial API

### Medium-Term Roadmap (v2.0)
- [ ] AS5048A Encoder - Upgrade to 14-bit (0.022° resolution)
- [ ] Custom H-Bridge - Current sensing and stall detection
- [ ] Trajectory Planner - Trapezoidal/S-curve velocity profiles
- [ ] Multi-Motor Sync - Coordinated control of 2+ motors

### Long-Term Vision (v3.0)
- [ ] ROS2 Integration - Standard robotics middleware compatibility
- [ ] System Identification - ML-based dynamic model estimation
- [ ] Model Predictive Control - Advanced optimal control strategy
- [ ] Force/Torque Feedback - Impedance control for physical interaction

---

## Conclusion

This project successfully demonstrates a **high-precision closed-loop position control system** using ESP32-S3 and AS5600 encoder, achieving **<1° steady-state error**, **0.8s rise time**, and **5% overshoot**.


<div align="center">

**⭐ Project Status: COMPLETED ⭐**

[⬆ Back to Top](#closed-loop-position-control-of-a-dc-motor)

</div>
