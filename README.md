<!-- markdownlint-disable MD033 MD041 MD013 -->
<div align="center">

# AltiOS UAV ✨

**The production-ready flight control system for autonomous altitude hold and safety.**

Built with modern embedded patterns for precision UAV navigation and stability.

[![Arduino](https://img.shields.io/badge/Arduino-UNO_R4-00979D?logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![C++](https://img.shields.io/badge/Language-C++-00599C?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![PID Control](https://img.shields.io/badge/Control-PID-FF6F00?logo=target&logoColor=white)](#algorithm-implementation)
[![LED Matrix](https://img.shields.io/badge/Display-LED_Matrix_12x8-ED1C24?logo=arduino&logoColor=white)](#visual-telemetry)
[![Hardware](https://img.shields.io/badge/Sensor-Ultrasonic_HY--SRF05-4DB6AC?logo=sensor-tower&logoColor=white)](#pin-mapping)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

</div>

---

> **Note**
>
> **Production-Grade UAV Flight Software**
>
> This is a high-performance, real-time implementation of an Altitude Hold system for unmanned aerial vehicles (UAVs). Built on the AltiOS micro-kernel philosophy, it provides precise PID-based altitude maintenance, noise-suppressed sensor feedback, and real-time visual telemetry.
>
> **Perfect for:** Autonomous drone development, flight control research, embedded systems education, and high-precision altitude maintenance projects.

---

**AltiOS UAV provides the reliable path from raw sensor data to stable flight**, offering real-time PID tuning, visual state diagnostics, and robust error handling.

The [Altitude Hold algorithm](https://en.wikipedia.org/wiki/Flight_control_surfaces) is the backbone of autonomous hover. This implementation leverages a high-speed PID controller and moving average filtering to transform noisy ultrasonic distance readings into a smooth, stable throttle response.

```cpp
// Core PID altitude hold implementation
void computePID(double dt) {
  double error = setpoint - altCurrent;

  // Integral with anti-windup protection
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);

  // Derivative component
  double derivative = (error - errorPrev) / dt;
  errorPrev = error;

  pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
}
```

## 📋 Table of Contents

- [What is AltiOS?](#what-is-altios)
- [Why This Implementation?](#why-this-implementation)
- [System Features](#system-features)
- [Hardware Setup](#hardware-setup)
- [Quick Start](#quick-start)
- [Usage & Calibration](#usage--calibration)
- [Visual Telemetry](#visual-telemetry)
- [Algorithm Implementation](#algorithm-implementation)
- [Architecture](#architecture)
- [Deployment](#deployment)
- [Contributing](#contributing)
- [Senior Developer Information](#senior-developer-information)

## What is AltiOS?

AltiOS (Altitude Operating System) is a specialized flight control layer designed for the **Arduino UNO R4 WiFi** architecture. It focuses on:

- **Stability**: Utilizing 20Hz PID loops for millisecond-accurate throttle correction.
- **Safety**: Implementing anti-windup integral logic and throttle safety bounds.
- **Awareness**: Real-time 12x8 LED Matrix visualization of the UAV's flight state.
- **Interaction**: A comprehensive serial CLI for run-time gain tuning and system diagnosis.

The system simulates a virtual "safety tether," ensuring that even with fluctuating sensor data, the UAV maintains its target altitude within a ±10cm hold zone.

### Key Concepts

- **PID Control**: The mathematical heart that manages proportional, integral, and derivative corrections.
- **Filtering**: A multi-sample moving average filter that ignores sonic reflections and sensor glitches.
- **Telemetry**: Visual feedback on the R4's built-in LED matrix for immediate state identification.
- **Arming Sequence**: A safe startup routine that initializes sensors before enabling ESC output.

## Why This Implementation?

This firmware handles the complex physics of vertical displacement while providing a high-level developer interface.

🚀 **High-Performance**: Optimized PID loop running at 20Hz for immediate response.  
🎓 **Modular**: Clean separation between sensor logic, control theory, and visualization.  
🔍 **Transparent**: Real-time telemetry via Serial and LED Matrix.  
🧪 **Safety-First**: Hardcoded throttle limits (1100-1700µs) and anti-windup logic.  
💻 **Future-Proof**: Built for the Renesas RA4M1 on the Arduino UNO R4.  
📱 **Visual**: Interactive boot animations and dynamic flight indicators.  
🎨 **Refined UI**: Custom hand-drawn animation frames for the LED matrix.

## System Features

### Core Control Engine

- **Safety-First PID**: Complete implementation with user-tunable Kp, Ki, and Kd constants.
- **Precision Filtering**: 5-sample moving average filter for ultrasonic stability.
- **Dynamic Setpoints**: Modify target altitude on-the-fly via Serial commands.
- **Anti-Windup**: Prevents integral saturation during long climb phases.
- **Emergency Stop**: Global `stop` command to immediately disarm all motors.

### Visual Telemetry (LED Matrix)

- 🎯 **Hold Indicator**: Solid square when within the ±10cm target zone.
- 🔼 **Climb Indicator**: Single/Double arrows for slight or critical altitude gain.
- 🔽 **Descend Indicator**: Single/Double arrows for slight or critical altitude loss.
- 🌀 **Radar Sweep**: A custom 2.2s radar-style boot sequence during initialization.

### Interactive CLI

- ⌨️ **Live Tuning**: Change PID gains (`p`, `i`, `d`) without reflashing.
- 📊 **Status Snapshots**: Comprehensive system health report with a single command.
- 🚀 **Telemetry Stream**: Detailed CSV-ready output for flight analysis.

## Hardware Setup

### Pin Mapping

| Component | Pin | Description                   |
| --------- | --- | ----------------------------- |
| **TRIG**  | D9  | Ultrasonic Trigger Signal     |
| **ECHO**  | D10 | Ultrasonic Echo Return        |
| **ESC 1** | D3  | Electronic Speed Controller 1 |
| **ESC 2** | D5  | Electronic Speed Controller 2 |
| **ESC 3** | D6  | Electronic Speed Controller 3 |
| **ESC 4** | D11 | Electronic Speed Controller 4 |

### Requirements

- **Microcontroller**: Arduino UNO R4 WiFi (Mandatory for LED Matrix)
- **Sensor**: HY-SRF05 or HC-SR04 Ultrasonic module
- **Power**: 5V stable source for sensor array
- **Frames**: `animation.h` must reside in the same directory as `altitude_hold.ino`.

## Quick Start

### Installation

```bash
# Clone the repository
git clone https://github.com/AlphsX/AntiOS_UAV.git
cd AntiOS_UAV/altitude_hold

# Open altitude_hold.ino in Arduino IDE 2.x
# Ensure 'Arduino UNO R4 WiFi' board package is installed
```

### First Setup

1. **Mount Sensor**: Ensure the ultrasonic head points **directly downward**.
2. **Flash Firmware**: Upload via USB-C to your R4 WiFi.
3. **Verify Boot**: Observe the "Radar Sweep" animation on the 12x8 matrix.
4. **Open Serial Monitor**: Set baud rate to `115200`.

## Usage & Calibration

### Calibration Commands

Enter these commands into the Serial Monitor to fine-tune your flight:

| Command  | Action                   | Example                  |
| -------- | ------------------------ | ------------------------ |
| `t[val]` | Set Target Altitude (cm) | `t100` (Hold at 1 meter) |
| `p[val]` | Set Proportional Gain    | `p2.8`                   |
| `i[val]` | Set Integral Gain        | `i0.05`                  |
| `d[val]` | Set Derivative Gain      | `d1.5`                   |
| `status` | System Health Report     | `status`                 |
| `stop`   | Emergency Disarm         | `stop`                   |

### Understanding Telemetry

The LED matrix reflects the `Error = Setpoint - Current_Altitude`:

- **Double-Up (Flash)**: Error > 40cm (Add aggressive power)
- **Single-Up (Slow)**: Error 10-40cm (Add slight power)
- **Solid Square**: Target Reached (HOLD)
- **Single-Down (Slow)**: Error -10 to -40cm (Reduce slight power)
- **Double-Down (Flash)**: Error < -40cm (Reduce aggressive power)

## Algorithm Implementation

### The Pulse Logic

The system utilizes high-precision timing for the ultrasonic return, converted to metric units before entering the filter pipeline.

```cpp
double readDistanceCM() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  return (dur * 0.0343) / 2.0;
}
```

### System Stability (Moving Average)

To combat sonic reflections from ground surfaces, we use a 5-sample circular buffer. This prevents the drone from reacting to "phantom" spikes in distance readings.

## Architecture

```text
AltiOS_UAV/
├── altitude_hold/
│   ├── altitude_hold.ino   # Main controller & loops
│   └── animation.h         # LED Matrix frame definitions
├── .gitignore              # Repository hygiene
└── README.md               # You are here
```

## Senior Developer Information

### Technical Expertise

**Project Architect** specializing in Embedded Flight Systems, Control Theory, and Real-Time Architectures.

#### Core Competencies:

- 🚁 **Autonomous Systems**: PID tuning, Kalman filtering, and flight dynamics.
- ⚙️ **Embedded Architecture**: Arduino R4 (Renesas), ESP32, and ARM Cortex.
- 💻 **Modern Embedded C++**: OOP patterns and efficient memory management.
- 📊 **Signal Processing**: Noise reduction, sensor fusion, and telemetry streams.

---

### Contact & Links

- **GitHub**: [@AlphsX](https://github.com/AlphsX)
- **Repo**: [AntiOS_UAV](https://github.com/AlphsX/AntiOS_UAV.git)

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

<div align="center">

**⭐ Star this repository if you find it helpful!**

Made with ❣️ for the UAV development community

© 2026 AlphsX, Inc.

</div>
