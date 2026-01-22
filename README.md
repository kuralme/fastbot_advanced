# Fastbot Advanced: Mixed-Criticality Autonomous Robot

This project focuses on the development of a differential drive robot - Fastbot, with a criticality in mind. The system is designed with lower/higher level architectures, utilizing an **ESP32** for hard real-time motion control and safety, and an **Orange Pi 5B (OPi)** for high-level SLAM and Nonlinear Model Predictive Control (NMPC).

## 🚀 Project Goals

* **Mixed-Criticality Execution:** Separating safety-critical tasks (Tier 1) from computationally intensive navigation (Tier 2).
* **Deterministic Motion Control:** Ensuring the PID loop and safety monitors are never interrupted by communication overhead.
* **System Reliability:** Self-healing communication loops and latching hardware fault states.
* **Advanced Navigation:** Implementing NMPC on a real-time Linux kernel for stable high-speed trajectory tracking.
* **SLAM & Fusion:** Lidar-based mapping fused with high-frequency wheel odometry and IMU.

---

## System Architecture

The project is organized as a monorepo containing both the low-level firmware and the high-level ROS 2 navigation stack.

### 1. Firmware (ESP32 - Tier 1: High Criticality)

The firmware is built using **ESP-IDF**. It leverages the dual-core architecture to isolate motor control from the micro-ROS transport layer communicating with uros agent through UART.

#### Core Logic Features:

* **Dual-Core Isolation:** Core 0 handles the 50Hz PID loop; Core 1 handles the micro-ROS communication.
* **Atomic Safety Layer:** Uses C11 atomics to share system status between cores without locks.
* **Self-Healing Transport:** A custom reconnection state machine that automatically restores the micro-ROS session if the Orange Pi reboots.
* **Latching Stall Protection:** Monitors "PWM Effort vs. Encoder Result" to kill motor power if a physical stall is detected.

#### Task Distribution Table

| Task | Priority | Core | Input | Output | Destination | Criticality Role |
| --- | --- | --- | --- | --- | --- | --- |
| **PID Timer (ISR)** | High | 0 | Encoder Ticks, Setpoints | PWM Signals | Motor Driver | Hard Real-time motor velocity regulation. |
| **Safety Watchdog** | High | 0 | `cmd_vel` timestamp | Motor Shutdown | Hardware Pins | Failsafe for communication loss. |
| **Stall Monitor** | High | 0 | Velocity vs PWM | `system_status` | Atomic Var | Prevents motor burnout/hardware damage. |
| **micro-ROS Task** | Medium | 1 | Agent Ping, `cmd_vel` | Odometry, Heartbeat | Orange Pi | Asynchronous telemetry and command bridge. |
| **Reset Service** | Medium | 1 | Trigger Request | `SYSTEM_OK` | Atomic Var | Software recovery from latching faults. |

### 2. SLAM & Navigation (Orange Pi 5B - Tier 2: Medium Criticality)

*Status: Under Development*

The high-level logic resides on an OPi running Ubuntu 24.

#### Key Components:
* **Slam Toolbox:** 2D Lidar-based mapping and localization.

* **Nonlinear MPC:** A nonlinear optimizer that calculates the best path while respecting the robot's physical limits.

* **Sensor Fusion:** Fusing Encoders with IMU (BNO085) for slip-resistant odometry.
