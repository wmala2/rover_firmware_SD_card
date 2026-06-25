# ESP32 Differential Drive Rover Firmware

This repository contains firmware for an **ESP32-based differential drive rover** with:

- Quadrature encoders on both wheels
- PID velocity control (closed loop)
- Optional open-loop PWM control
- UDP command interface (low latency)
- HTTP control & diagnostics interface
- Ring-buffer logging with downloadable logs
- Runtime tuning via web API
- Compile-time motor profile selection (130 RPM vs 500 RPM)

The firmware is structured as a clean **PlatformIO project** with clear module boundaries and minimal dynamic allocation.

---

## Features at a Glance

- **Closed-loop velocity control** using encoder feedback  
- **Anti-windup PID** with configurable decay (`AW_DECAY`)  
- **Command queue with playback rate** (web-configurable)  
- **Safety auto-stop** on command timeout  
- **HTTP status & configuration** (no reboot required)  
- **Encoder + PID telemetry logging**  
- **Motor profile abstraction** (RPM, CPR, PWM floor)

---

## Project Structure

```
.
├── platformio.ini
├── include/
│   ├── hardware_config.h     # Pins, constants, motor profiles
│   ├── wifi_config.h         # Wi-Fi credentials & IP config
│   ├── encoder.h             # Quadrature encoder interface
│   ├── motor_driver.h        # H-bridge PWM driver
│   ├── pid.h                 # PID controller
│   ├── control_server.h      # UDP + HTTP + logging
│   ├── http_helpers.h        # Minimal HTTP parsing helpers
│   └── json_helpers.h        # Lightweight JSON parsing
└── src/
    ├── main.cpp
    ├── encoder.cpp
    ├── motor_driver.cpp
    ├── pid.cpp
    ├── control_server.cpp
    ├── http_helpers.cpp
    └── json_helpers.cpp
```

---

## Build & Flash

### Requirements
- PlatformIO
- ESP32 DevKit (or compatible)
- Arduino framework

### Build
```bash
pio run
```

### Upload
```bash
pio run -t upload
```

### Serial Monitor
```bash
pio device monitor
```

---

## Motor Profile Selection (Compile-Time)

The firmware supports different motor/gearbox/encoder combinations via a single macro.

### Supported Profiles
- **130 RPM motors**
- **500 RPM motors**

### Select via PlatformIO
```ini
build_flags =
  -D MOTOR_PROFILE=130
```

or

```ini
build_flags =
  -D MOTOR_PROFILE=500
```

Each profile configures:
- `PWM_MIN` (minimum usable PWM)
- `ENCODER_CPR_WHEEL` (counts per wheel revolution)

All control math automatically adapts.

---

## Runtime Architecture Overview

- **UDP RX task** – receives motion commands and enqueues them  
- **Command playback loop** – dequeues commands at `cmdRate` Hz  
- **PID loop** – runs at `PID_RATE`, computes PWM from encoder deltas  
- **HTTP task** – serves status, logs, and configuration  
- **Encoder ISRs** – update counts asynchronously  

---

## HTTP Interface

### Root
```
GET /
```

### Status
```
GET /status
```

### Set Parameters
```
GET /set?trimL=1.02&trimR=0.98&cmdRate=8
```

### Download Logs
```
GET /log
```

### Clear Logs
```
GET /clear
```

---

## UDP Command Interface

UDP listens on port **9000** (replies on **9001**).

### Closed-Loop Velocity (`m`)
```json
{
  "command": "m",
  "left_mps": 0.20,
  "right_mps": 0.20,
  "index": 42
}
```

### Open-Loop PWM (`o`)
```json
{
  "command": "o",
  "left_pwm": 80,
  "right_pwm": 80
}
```

### Encoder Query (`e`)
```json
{
  "command": "e"
}
```

### Reset Encoders (`r`)
```json
{
  "command": "r"
}
```

---

## PID Anti-Windup (`AW_DECAY`)

When the PID output saturates, the integral term is gradually reduced:

```c
integral *= AW_DECAY;
```

This prevents overshoot and sluggish recovery after saturation.

---

## License

MIT
