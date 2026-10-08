# Fastbot Advanced

[![Firmware Static Analysis](https://github.com/kuralme/fastbot_advanced/actions/workflows/static_analysis.yml/badge.svg)](https://github.com/kuralme/fastbot_advanced/actions/workflows/static_analysis.yml)

Fastbot Advanced is a differential-drive robot project spanning ESP32 motor-control firmware and a ROS 2 perception stack. The ESP32 closes the wheel-speed loops, estimates encoder odometry, and handles motor fault states; RPi4s runs stereo visual SLAM and sensor tools. The current mapping workflow is joystick-driven rather than autonomous navigation.

## System overview

- **ESP32 firmware (`firmware/`):** ESP-IDF C application with a 50 Hz feed-forward PID controller, encoder-based odometry, PWM motor outputs, command-loss stopping, and stall-fault handling.
- **ROS 2 (`slam/`):** Stella-VSLAM ROS integration for the OAK-D Lite stereo camera, a BNO085 IMU driver, and MCAP recording scripts for sensor and SLAM topics.
- **Transport and deployment (`docker/`):** Docker Compose services for the micro-ROS agent, SLAM stack, and IMU driver. The ESP32 communicates with the agent over UART.

The firmware exposes `/fastbot/cmd_vel`, publishes `/fastbot/encoder_odom` and `/fastbot/heartbeat`, and provides `/fastbot/reset_fault` to clear a latched motor-stall fault. The IMU node publishes orientation, angular velocity, and acceleration on `/fastbot/imu`.

## Build and run

Build and flash the ESP32 firmware with ESP-IDF installed:

```sh
cd firmware
make setup
make build
make flash PORT=/dev/ttyUSB0
```

Start the ROS 2 services from the repository root:

```sh
docker compose -f docker/docker-compose.yml up --build
```

Use a joystick to drive the robot during mapping. MCAP recording helpers and their topic configuration are under `slam/recordings/`.

## Firmware checks

The firmware Makefile provides `make tidy-check`, `make cppcheck`, and `make lint` targets. The repository also runs firmware static analysis in GitHub Actions.

## Future work

- Evaluate GTSAM as an alternative optimization backend to Stella-VSLAM's default g2o backend.
