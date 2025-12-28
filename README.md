# RoverBot v1

Autonomous rover platform built on Raspberry Pi 5 with ROS2.

![RoverBot v1 Concept](rover.png)

## Specs

| Spec | Value |
|------|-------|
| Chassis | 24" x 12" T-slot aluminum |
| Motors | Pololu 37D 131:1 (76 RPM) |
| Wheels | 7.6" Wasteland (192mm) |
| Speed | 1.5 mph |
| Ground Clearance | ~4" |
| Weight | ~8.3 kg |
| Battery | 4S LiPo 8000mAh |
| Runtime | ~2+ hours |
| Brain | Raspberry Pi 5 16GB (ROS2 Jazzy) |

## Hardware Summary

| Component | Details |
|-----------|---------|
| CPU | Cortex-A76 @ 2.4GHz (4 cores) |
| RAM | 16 GB LPDDR4X |
| Storage | Transcend ESD310C USB SSD |
| GPU | VideoCore VII (Vulkan 1.2) |
| Cooling | Aluminum passive case |
| Architecture | ARM64 |
| Power | ~8W typical |
| Size | 85 x 56 x 20 mm (+ case) |

## Architecture

Indoor/outdoor rover with full autonomous navigation:

```
+------------------+
| Raspberry Pi 5   |  <-- Mounted in aluminum case
|  +------------+  |
|  | 2D Lidar   |  |  <-- RPLidar A1
|  +------------+  |
|  | Depth Cam  |  |  <-- Intel RealSense D435
|  +------------+  |
|  | IMU        |  |  <-- BNO055
|  +------------+  |
+------------------+
        |
+------------------+
| Arduino Mega     |  <-- Motor control, encoders
+------------------+
        |
+------------------+
| Cytron MDD10A x2 |  <-- Dual motor drivers
+------------------+
        |
+------------------+
| 4WD Chassis      |  <-- 24"x12" platform
+------------------+
```

Capabilities:
- SLAM (build maps autonomously)
- Autonomous navigation (Nav2)
- Object detection and tracking
- Multi-sensor fusion
- Remote operation via web/app

## Recommended Software Stack

| Layer | Component |
|-------|-----------|
| OS | Ubuntu 24.04 LTS |
| Middleware | ROS2 Jazzy |
| Navigation | Nav2 |
| SLAM | slam_toolbox |
| Vision | OpenVINO + YOLOv8 |
| Simulation | Gazebo Harmonic |
| Visualization | RViz2 |

## Documents

| File | Purpose |
|------|---------|
| README.md | This overview |
| SHOPPING_LIST.md | Complete parts list with URLs |
| CHECKLIST.md | Shopping checklist with status |
| CHASSIS_BUILD.md | Custom 24"x12" T-slot chassis build |
| MOTOR_MOUNT_DESIGN.md | Motor mounting details |
| ROS2_GUIDE.md | ROS2 concepts and getting started |
| old-parts.md | Inventory of existing parts |

## Getting Started

1. Install Ubuntu 24.04 or Raspberry Pi OS on Pi 5
2. Install ROS2 Jazzy (see ROS2_SLAM_STACK.md)
3. Acquire hardware (see SHOPPING_LIST.md)
4. Build chassis (see CHASSIS_BUILD.md)
5. Integrate and test
