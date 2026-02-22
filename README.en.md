# 🚀 BOZOK Flight Control Computer (FCC)

**2025 TEKNOFEST Rocket Competition – 1st Place**  
Developed by: **Bozok Rocket Team**

This repository contains the source code of the custom-designed Flight Control Computer (FCC) actively used during the 2025 TEKNOFEST Rocket Competition.

The system was flight-proven during official competition launches and satisfies all avionics requirements defined in the competition rules.

---

# 📌 System Overview

The BOZOK Flight Control Computer is a high-reliability real-time embedded system designed for rocket missions.

The software:

- Runs on FreeRTOS  
- Executes the main flight algorithm at 80 Hz  
- Transmits telemetry via LoRa  
- Determines flight state using barometric and IMU data  
- Performs apogee detection  
- Triggers the recovery system  
- Supports SIT and SUT test modes  

---

# ⚙️ Real-Time Architecture

The system is FreeRTOS-based and consists of two main tasks:

---

## 1️⃣ sensorTask

Operating frequency:

- 80 Hz in NORMAL mode  
- 10 Hz in SIT / SUT modes  

Responsibilities:

- IMU data acquisition (I2C)
- BMP390 pressure & altitude calculation
- GPS NMEA parsing via UART interrupt
- FIFO-based vertical velocity calculation (Linear Regression)
- Flight state machine management
- Apogee detection algorithm
- Recovery deployment decision logic

---

## 2️⃣ loraTask

Operating frequency:

- Every 500 ms

Responsibilities:

- Telemetry packet generation
- Big Endian data encoding
- 8-bit checksum calculation
- UART transmission to LoRa module

---

# 🧠 Flight State Machine

The system includes the following flight states:

```
ON_GROUND
PROTECTED_FLIGHT
ALTITUDE_PROTECTED_FLIGHT
FREE_FLIGHT
APOGEE
TRIGGER
```

---

## 🚀 Launch Detection

- NORMAL mode: Based on pressure drop threshold  
- SUT mode: Based on acceleration threshold  

---

## 🎯 Apogee Detection

### NORMAL Mode

- Continuously tracks minimum pressure  
- Detects validated pressure increase beyond threshold  
- Uses counter-based validation to prevent false triggers due to noise  

### SUT Mode

- Detects continuous altitude decrease  
- Uses counter-based validation for reliability  

---

# 📈 Vertical Velocity Estimation

Vertical velocity is not computed using simple differentiation.

Instead, it is calculated using altitude-time samples stored in a FIFO buffer via:

Linear Regression (Least Squares Method)

Advantages:

- Noise-resistant  
- Stable against pressure jitter  
- Reliable for apogee detection  

---

# 📡 Telemetry Packet Structure

The telemetry packet includes:

- Header: 0xAA
- Accelerometer data (int16, scaled)
- Gyroscope data (int16, scaled)
- Quaternion (float)
- GPS latitude & longitude (int32, scaled)
- GPS altitude
- Barometric altitude
- Surface angle
- Flight state
- 8-bit checksum
- End byte (LF)

All multi-byte values are transmitted in Big Endian format.

---

# 🧪 Test Modes

## 🔹 NORMAL
Actual flight mode.

## 🔹 SIT (System Integration Test)
- Transmits filtered sensor data via UART  
- Used for system-level integration validation  

## 🔹 SUT (Sensor Unit Test)
- Accepts externally simulated sensor data  
- Enables hardware-in-the-loop testing  

---

# 🔐 Safety & Reliability

- Deterministic scheduling via FreeRTOS
- Counter-based apogee validation
- Noise-resistant pressure algorithm
- Critical section protection for shared data
- DMA-based UART reception
- Fail-safe reset mechanism

---

# 🛠 Technologies Used

- STM32 Microcontroller
- FreeRTOS
- LoRa Communication
- UART DMA
- I2C Sensor Interfaces
- C (Embedded Systems)

---

# 🏆 Achievement

🥇 1st Place – 2025 TEKNOFEST Rocket Competition

---

# ⚠️ Disclaimer

This repository is shared for educational and research purposes only.  
Flight-critical systems must undergo rigorous testing and validation before real-world deployment.
