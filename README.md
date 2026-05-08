# Fastbot Advanced: Mixed-Criticality Autonomous Robot

[![Firmware Static Analysis](https://github.com/kuralme/fastbot_advanced/actions/workflows/static_analysis.yml/badge.svg)](https://github.com/kuralme/fastbot_advanced/actions/workflows/static_analysis.yml)

This project focuses on the development of a differential drive robot - Fastbot, with a criticality in mind. The system is designed with lower/higher level architectures, utilizing an **ESP32** for hard real-time motion control and safety, and an **Raspberry Pi 4B** for high-level SLAM execution.

## 🚀 Project Goals

* **Mixed-Criticality Execution:** Separating safety-critical tasks (Tier 1) from computationally intensive SLAM (Tier 2).
* **Deterministic Motion Control:** Ensuring the PID loop and safety monitors are never interrupted by communication overhead.
* **System Reliability:** Self-healing communication loops and latching hardware fault states.
* **Advanced Navigation:** Implementing *Stella-SLAM* for visual localization and high-level trajectory control on a high-performance Linux environment.
* **SLAM & Fusion:** Stereo camera based SLAM fused with high-frequency wheel odometry and IMU.

---

## System Architecture

The project is organized as a monorepo containing both the low-level firmware and the high-level ROS2 SLAM stack.

### 1. 🦾 Firmware (ESP32 - Tier 1: High Criticality)

The firmware is built using **ESP-IDF**. It leverages the dual-core architecture to isolate motor control from the micro-ROS transport layer communicating with uros agent through UART.

#### Core Logic Features:

* **Dual-Core Isolation:** Core 0 handles the 50Hz PID loop; Core 1 handles the micro-ROS communication.
* **Atomic Safety Layer:** Uses C11 atomics to share system status between cores without locks.
* **Self-Healing Transport:** A custom reconnection state machine that automatically restores the micro-ROS session if the RPi reboots.
* **Latching Stall Protection:** Monitors "PWM Effort vs. Encoder Result" to kill motor power if a physical stall is detected.

#### Task Distribution Table

| Task | Priority | Core | Input | Output | Destination | Criticality Role |
| --- | --- | --- | --- | --- | --- | --- |
| **PID Timer (ISR)** | High | 0 | Encoder Ticks, Setpoints | PWM Signals | Motor Driver | Hard Real-time motor velocity regulation. |
| **Safety Watchdog** | High | 0 | `cmd_vel` timestamp | Motor Shutdown | Hardware Pins | Failsafe for communication loss. |
| **Stall Monitor** | High | 0 | Velocity vs PWM | `system_status` | Atomic Var | Prevents motor burnout/hardware damage. |
| **micro-ROS Task** | Medium | 1 | Agent Ping, `cmd_vel` | Odometry, Heartbeat | Orange Pi | Asynchronous telemetry and command bridge. |
| **Reset Service** | Medium | 1 | Trigger Request | `SYSTEM_OK` | Atomic Var | Software recovery from latching faults. |

#### 🛡️ Reliability & Static Analysis

To ensure execution safety and deterministic behavior, the firmware is validated against industry-standard rule sets:

* **MISRA C:2012 / CERT C Compliance:** Using `cppcheck` and `clang-tidy` (via `cppcoreguidelines-*`), the codebase is audited for undefined behavior, pointer safety, and integer overflows.
* **Zero-Allocation Principle:** Post-initialization, the Tier 1 control loops (PID, Safety Watchdog) avoid dynamic memory allocation (`malloc/free`) to prevent heap fragmentation and non-deterministic timing.
* **Type Safety:** Strict enforcement of fixed-width integers (`uint32_t`, etc.) and `const`-correctness to ensure cross-platform predictability between the ESP32 and Raspberry Pi.

---

### 2. 🧠 SLAM (Raspberry Pi 4 - Tier 2: Medium Criticality)

The high-level perception layer is hosted on a **Raspberry Pi 4B** running Ubuntu Server 22.04. This tier transforms raw sensor data into spatial intelligence, providing the robot with localization and environmental awareness while remaining isolated from the motor-critical safety loops.

#### Core Logic Features

* **Visual SLAM Engine:** Uses [stella_vslam](https://stella-cv.readthedocs.io/en/latest/overview.html) for feature-based (ORB) stereo framework for robust localization and mapping. It maintains a sparse 3D point cloud and handles global optimization to eliminate spatial drift.
* **Manual Mapping Workflow:** Currently optimized for joystick-driven mapping. High-level autonomous navigation is bypassed in favor of manual `teleop_twist_joy` control to ensure high-quality feature extraction in complex environments.
* **Containerized Stack:** The entire ROS 2 environment is dockerized, ensuring consistent behavior between the RPi hardware and online running / offline processing on a pc.
* **Unified Data Logging:** Uses the MCAP storage format for high-efficiency recording of stereo image streams, IMU data, and wheel odometry, allowing for deterministic offline re-runs.

#### Technical Specifications

| Component | Implementation | Role |
| --- | --- | --- |
| **Vision Input** | OAK-D Lite | Stereo rectified image streams (640x480). |
| **Control Mode** | Teleop (Joystick) | Manual velocity command generation for mapping. |
| **Data Storage** | MCAP (.mcap) | High-bandwidth recording of vision and telemetry data. |
| **Map Database** | MessagePack (.msg) | Fast serialization for saving/loading spatial landmarks. |
| **Transport** | micro-ROS Agent | High-speed UART bridge to the Tier 1 hardware. |

#### Operation Modes

Currently, the tested operations mode on the robot:

* **Online Mapping:** The robot processes live stereo streams to build a sparse 3D map while navigating. This mode generates the initial .msg map database.

* **Offline Re-localization:** Processing recorded .mcap bags on a workstation to refine the map or test new navigation parameters without physical hardware deployment.


### 🚧 Improvements & Future Works

* **Sensor Fusion:** It is possible to integrate an EKF fusion to the project; combining wheel odometry (from ESP32) with stereo visual odometry and IMU data.

* **Global Optimization:** Stella-vslam uses _g2o_ optimization library by default, however its possible(and potentially better) to enable _GTSAM_ as well.