# Adaptive Embedded Fitness Wearable

An embedded wearable system that automatically detects exercises, counts repetitions and sets, tracks steps, and dynamically adapts to different users using motion-based calibration.

This project was developed as part of **Final Year Project**.

---

## Problem Statement

Most fitness trackers rely on cloud processing or predefined thresholds, making them inaccurate across different users and exercises. This project aims to design a **standalone embedded wearable** that performs **on-device motion analysis**, exercise detection, and repetition counting without external computation.

---

## Key Objectives

- Detect multiple strength-training exercises using IMU data
- Count repetitions and sets accurately
- Support both **workout mode** and **step-counting mode**
- Adapt to different users through one-time calibration
- Operate as a self-contained embedded system

---

## Hardware Components

- ESP32 microcontroller  
- MPU6050 (Accelerometer + Gyroscope)  
- 0.96” OLED Display  
- Capacitive Touch Input  
- Heart Rate Sensor (optional / planned extension)  
- Buzzer for feedback  

---

## Modes of Operation

### 1. Workout Mode
- User selects exercise and target repetitions
- One-time calibration captures user-specific motion thresholds
- Real-time repetition counting using axis-specific motion detection
- Visual and audio feedback via OLED and buzzer

### 2. Step Counting Mode
- Continuous background monitoring of accelerometer data
- Noise-filtered step detection
- Automatically resumes after workout completion

---

## Exercise Detection Logic

| Exercise          | Primary Axis Used |
|-------------------|------------------|
| Bicep Curl        | Z-axis           |
| Shoulder Press    | X-axis           |
| Lateral Raise     | Z & Y axes       |
| Squats            | X-axis           |
| Deadlift          | Z-axis           |

Each exercise uses **adaptive thresholds learned during calibration**, rather than fixed values.

---

## System Architecture

![System Architecture](docs/system_architecture.png)

The system operates using a **state-machine-driven architecture**, switching between configuration, calibration, workout execution, and step-counting modes.

This approach ensures deterministic behavior and clean mode transitions.

---

## Software Design Highlights

- Modular exercise detection functions
- Dynamic threshold calibration
- Non-blocking execution using state logic
- Real-time sensor processing on ESP32
- OLED-based user interaction (no smartphone required)

---

## Limitations

- Exercise detection is axis-based and assumes consistent wearable orientation
- Heart-rate-based RPE estimation is planned for future phases
- Battery optimization not implemented in FYP-1

---

## Future Improvements (FYP-2)

- TinyML-based RPE estimation
- Automatic rest-time adaptation
- Improved sensor fusion
- Data logging and analytics

---

## Author

Harshvardhan Rajkumar  
Final Year B.Tech – Electronics & Computer Engineering
