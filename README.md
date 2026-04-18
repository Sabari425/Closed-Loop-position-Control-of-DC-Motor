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

## 📋 Table of Contents
- [Overview](#-overview)
- [Repository Structure](#-repository-structure)
- [Live Demo & Documentation](#-live-demo--documentation)
- [Key Performance Metrics](#-key-performance-metrics)
- [System Architecture](#-system-architecture)
- [Hardware Components](#-hardware-components)
- [Control Algorithm](#-control-algorithm)
- [Installation & Setup](#-installation--setup)
- [Performance Analysis](#-performance-analysis)
- [Future Improvements](#-future-improvements)
- [Contributors](#-contributors)
- [License](#-license)

---

## 🎯 Overview

This project implements a **high-precision closed-loop position controller** for a standard DC gear motor, transforming it into a **Smart Joint** suitable for robotic applications. The system uses an **ESP32-S3** microcontroller as the computational core, an **AS5600 12-bit magnetic encoder** for absolute position feedback, and an **L298N motor driver** for PWM-based actuation.

### 🎯 Key Objectives
- Achieve sub-degree positioning accuracy
- Minimize overshoot and settling time
- Compensate for mechanical non-linearities (backlash, friction)
- Provide real-time monitoring and tuning capabilities

### ✨ Highlights
- **0.087°** encoder resolution (12-bit magnetic)
- **0.8s** rise time to target
- **<5%** overshoot
- **1.8s** settling time (±2.5° tolerance)
- **<1°** steady-state error

---

## 🎬 Live Demo & Documentation

<div align="center">

### 📹 Quick Demo
[▶️ Watch DEMO.mp4](DEMO.mp4)

### 📹 Extended Demo
[▶️ Watch Position_Control_of_Robotic_arm_video.mp4](Position_Control_of_Robotic_arm_video.mp4)

### 📊 Project Presentation
[📽️ Download PowerPoint Presentation](Position%20Control%20of%20Robotic%20arm.pptx)

### 📖 Full Documentation
[📄 Download PDF Documentation](Position%20Control%20of%20Robotic%20arm.pdf)

</div>

<details>
<summary><b>📸 Click to view diagrams and schematics</b></summary>

### System Architecture
![Architecture](Architecture.jpeg)

### Control Flow Diagram
![Sample Flow Diagram](Sample_flow_diagram.png)

### Circuit Schematic
![Schematic Diagram](Schematic_Diagram_for_project_ref.png)

</details>

---

## 📊 Key Performance Metrics

<div align="center">

| Metric | Value | Rating |
|:------:|:-----:|:------:|
| **Rise Time** | 0.8 seconds | ⭐⭐⭐⭐⭐ |
| **Overshoot** | ~5% | ⭐⭐⭐⭐ |
| **Settling Time** | 1.8 seconds | ⭐⭐⭐⭐ |
| **Steady-State Error** | < 1° | ⭐⭐⭐⭐⭐ |
| **Encoder Resolution** | 0.087° (12-bit) | ⭐⭐⭐⭐⭐ |
| **Control Frequency** | 100 Hz | ⭐⭐⭐⭐ |

</div>

---

## 🏗️ System Architecture

<div align="center">
<img src="Architecture.jpeg" alt="System Block Diagram" width="800"/>
</div>

### 📡 Signal Flow
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│ │ │ │ │ │
│ USER INPUT │────────▶│ ESP32-S3 │────────▶│ L298N DRIVER │
│ (Target Angle) │ │ (PID Controller)│ │ (PWM Output) │
│ │ │ │ │ │
└─────────────────┘ └────────┬────────┘ └────────┬────────┘
│ │
│ I2C (SDA/SCL) │ PWM Signal
│ │
▼ ▼
┌─────────────────┐ ┌─────────────────┐
│ │ │ │
│ AS5600 │◀────────│ DC GEAR MOTOR │
│ ENCODER │ │ with Magnet │
│ (12-bit) │ │ │
└─────────────────┘ └─────────────────┘
│
│ Absolute Angle Feedback
│ (0.087° resolution)
▼
┌─────────────────┐
│ CLOSED LOOP │
│ FEEDBACK │
│ (Error = T-P) │
└─────────────────┘


### 🔄 Control Flow Diagram

<div align="center">
<img src="Sample_flow_diagram.png" alt="Control Flow Diagram" width="600"/>
</div>

**Algorithm Steps:**
1. **Initialize** - Configure I2C, PWM, Serial, and calibrate encoder zero position
2. **Read Target** - Receive desired angle from GUI via Serial or preset value
3. **Read Position** - Query AS5600 via I2C for current absolute angle (0-360°)
4. **Compute Error** - Calculate shortest-path angular error with wrap-around handling
5. **PID Calculation** - Execute discrete PID with anti-windup and output limiting
6. **Deadband Compensation** - Apply minimum PWM threshold to overcome static friction
7. **Slow-Zone Control** - Reduce gain when near target for smooth settling
8. **Output PWM** - Send calculated PWM value to L298N motor driver
9. **Telemetry** - Stream position, error, and PWM data to GUI at 100Hz
10. **Loop** - Repeat from step 2 continuously

### 🔌 Circuit Schematic

<div align="center">
<img src="Schematic_Diagram_for_project_ref.png" alt="Circuit Schematic" width="700"/>
</div>

---

## 🔧 Hardware Components

| Component | Specification | Purpose |
|:----------|:-------------|:--------|
| **ESP32-S3** | Dual-core Xtensa LX7 @ 240MHz | Main controller, PID computation, I2C/UART |
| **AS5600** | 12-bit Magnetic Encoder | Absolute angle feedback (0.087° resolution) |
| **L298N** | Dual H-Bridge Driver | PWM motor actuation (2A max continuous) |
| **DC Gear Motor** | 12V, 100 RPM | Actuator with 1:50 gear reduction ratio |
| **Neodymium Magnet** | 6mm diametrically magnetized | Encoder target attached to motor shaft |
| **LiPo Battery** | 11.1V, 2200mAh | Power supply (3S configuration, ~30min runtime) |

### 🔌 Pin Configuration Table

| ESP32-S3 Pin | Connected To | Signal Type | Function |
|:------------:|:------------:|:-----------:|:--------:|
| GPIO 21 | AS5600 SDA | I2C Data | Bi-directional data line |
| GPIO 22 | AS5600 SCL | I2C Clock | Serial clock output |
| GPIO 25 | L298N ENA | PWM Output | Enable A channel (0-255) |
| GPIO 26 | L298N IN1 | Digital Out | Direction control bit 1 |
| GPIO 27 | L298N IN2 | Digital Out | Direction control bit 2 |
| 3.3V | AS5600 VCC | Power | 3.3V regulated output |
| GND | Common | Ground | Shared ground reference |
| VIN (5V) | L298N 5V | Power | Logic supply for L298N |

---

## 🧮 Control Algorithm

### 📐 PID Controller Equation

<div align="center">
┌─────────────────────────────────────────────────────────────────┐
│ │
│ u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·(de/dt) │
│ │
│ Where: │
│ ───────────────────────────────────────────────────────────── │
│ u(t) = Control output (PWM value, range: -255 to +255) │
│ e(t) = Error = Target Angle - Measured Angle │
│ Kp = Proportional gain (1.2) │
│ Ki = Integral gain (0 - disabled) │
│ Kd = Derivative gain (0.35) │
│ dt = Sample time (10ms = 100Hz) │
│ │
└─────────────────────────────────────────────────────────────────┘

</div>

### ⚙️ Tuned PID Parameters

| Parameter | Value | Effect | Tuning Method |
|:---------:|:-----:|:-------|:--------------|
| **Kp** | 1.2 | Main driving force proportional to error | Increased until oscillation, then reduced by 50% |
| **Ki** | 0 | Disabled - integral windup prevention | Set to zero due to sufficient PD performance |
| **Kd** | 0.35 | Damping - reduces overshoot and oscillations | Increased until overshoot <5% achieved |
| **Sample Time** | 10ms | Control loop frequency (100 Hz) | Based on motor time constant and encoder speed |
| **Output Limit** | ±255 | PWM range (0-255 forward, 0-255 reverse) | Hardware PWM resolution limit |

### 🛡️ Advanced Control Features

| Feature | Implementation | Benefit |
|:--------|:--------------|:--------|
| **Shortest-Path Error** | `if (error > 180) error -= 360;` | Always takes optimal direction (≤180°) |
| **Anti-Windup** | Integral clamping with conditional accumulation | Prevents integral term explosion |
| **Deadband Compensation** | Minimum PWM threshold (≈15) | Overcomes static friction at low speeds |
| **Slow-Zone Control** | 50% gain reduction when error < 10° | Smooth settling without hunting |
| **Wrap-Around Handling** | Full 0-360° continuous rotation support | No discontinuity at 0°/360° boundary |

---

## 📥 Installation & Setup

### 🔧 Hardware Assembly

1. **Encoder Mounting**
   - Attach 6mm diametrically magnetized magnet to motor shaft using adhesive
   - Position AS5600 sensor board centered over magnet with 1mm air gap
   - Secure sensor board with custom 3D-printed bracket or standoffs

2. **Wiring Instructions** *(Reference: Schematic_Diagram_for_project_ref.png)*
ESP32-S3 GPIO21 ────► AS5600 SDA
ESP32-S3 GPIO22 ────► AS5600 SCL
ESP32-S3 3.3V ────► AS5600 VCC
ESP32-S3 GND ────► AS5600 GND

ESP32-S3 GPIO25 ────► L298N ENA (PWM)
ESP32-S3 GPIO26 ────► L298N IN1
ESP32-S3 GPIO27 ────► L298N IN2
ESP32-S3 GND ────► L298N GND

Battery 11.1V (+) ──► L298N 12V Input
Battery GND (-) ────► L298N GND

L298N OUT1 ──────────► Motor Terminal 1
L298N OUT2 ──────────► Motor Terminal 2


### 💾 Firmware Flashing

```bash
# Clone the repository
git clone https://github.com/Sabari425/position-control.git
cd position-control/firmware

# Install PlatformIO (if not installed)
pip install platformio

# Compile and upload to ESP32-S3
pio run --target upload

# Monitor serial output for debugging
pio device monitor --baud 115200
```

## 📈 Performance Analysis

### 📊 Step Response Characteristics

| Metric | Measured Value | Target Specification | Status |
|:-------|:-------------:|:--------------------:|:------:|
| Rise Time (10% → 90%) | 0.8 seconds | < 1.0 seconds | ✅ PASS |
| Peak Overshoot | 5.0% | < 10% | ✅ PASS |
| Settling Time (±2.5°) | 1.8 seconds | < 2.0 seconds | ✅ PASS |
| Steady-State Error | < 1° | < 2° | ✅ PASS |
| Maximum Tracking Speed | 180°/sec | > 120°/sec | ✅ PASS |

### 📉 Error Distribution Analysis

| Error Range | Occurrence Frequency | Cumulative Percentage |
|:-----------:|:--------------------:|:---------------------:|
| 0.0° - 0.5° | ████████████████████ (45%) | 45% |
| 0.5° - 1.0° | ██████████████ (35%) | 80% |
| 1.0° - 2.0° | ██████ (15%) | 95% |
| > 2.0° | ██ (5%) | 100% |

### 🔬 Frequency Domain Analysis

| Parameter | Value | Interpretation |
|:----------|:-----:|:--------------|
| Bandwidth (-3dB) | 2.5 Hz | System can track inputs up to 2.5Hz |
| Phase Margin | ~60° | Good stability - well above 45° minimum |
| Gain Margin | 8 dB | Robust to gain variations |
| Resonance Peak | None | No oscillatory modes detected |

### 📈 Disturbance Rejection

| Disturbance Type | Recovery Time | Max Deviation |
|:-----------------|:-------------:|:-------------:|
| External Torque (50% load) | 0.4 seconds | 8° |
| Sudden Setpoint Change (180°) | 1.2 seconds | 5% overshoot |
| Power Supply Sag | No effect | <0.5° |

---

## 🚀 Future Improvements

### 🔜 Short-Term Roadmap (v1.1)
- [ ] Adaptive PID - Auto-tuning based on error magnitude and velocity
- [ ] Velocity Feed-Forward - Improved trajectory tracking accuracy
- [ ] Bluetooth/BLE Interface - Wireless control from mobile devices
- [ ] Web Dashboard - Browser-based monitoring using WebSerial API

### 📅 Medium-Term Roadmap (v2.0)
- [ ] AS5048A Encoder - Upgrade to 14-bit (0.022° resolution)
- [ ] Custom H-Bridge - Current sensing and stall detection
- [ ] Trajectory Planner - Trapezoidal/S-curve velocity profiles
- [ ] Multi-Motor Sync - Coordinated control of 2+ motors

### 🔮 Long-Term Vision (v3.0)
- [ ] ROS2 Integration - Standard robotics middleware compatibility
- [ ] System Identification - ML-based dynamic model estimation
- [ ] Model Predictive Control - Advanced optimal control strategy
- [ ] Force/Torque Feedback - Impedance control for physical interaction

---

## 🎯 Conclusion

This project successfully demonstrates a **high-precision closed-loop position control system** using ESP32-S3 and AS5600 encoder, achieving **<1° steady-state error**, **0.8s rise time**, and **5% overshoot**.


<div align="center">

**⭐ Project Status: COMPLETED ⭐**

[⬆ Back to Top](#closed-loop-position-control-of-a-dc-motor)

</div>
