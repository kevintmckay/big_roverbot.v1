# RoverBot v1

Autonomous rover platform built on Intel NUC with ROS2.

## Hardware Summary

| Component | Details |
|-----------|---------|
| CPU | Intel Celeron N5105 @ 2.00GHz (4 cores, 2.9GHz boost) |
| RAM | 32 GB DDR4 |
| Storage | Intel SSDPEKNU020TZ NVMe SSD (1.9 TB) |
| GPU | Intel UHD Graphics (Jasper Lake, OpenVINO capable) |
| Motherboard | Intel NUC11ATBC4 (M53051-400) |
| Architecture | x86_64 with VT-x virtualization |
| Power | 10-15W typical |
| Size | ~117 x 112 x 37 mm |

## Comparison: NUC vs Pi 5

| Spec | Pi 5 (Rock Crawler) | Intel NUC (RoverBot v1) |
|------|------------------|----------------------|
| CPU | Cortex-A76 @ 2.4GHz | Celeron N5105 @ 2.9GHz |
| Cores | 4 | 4 |
| RAM | 16GB | 32GB |
| Storage | 256GB USB | 2TB NVMe |
| GPU Accel | None (Hailo optional) | Intel UHD + OpenVINO |
| Power | 5W | 10-15W |
| OS | Debian ARM64 | Ubuntu x86_64 |
| Best For | Small crawler, edge | Larger robot, ROS2 |

## AI Inference Performance

Using OpenVINO acceleration on Intel UHD Graphics:

| Model | CPU Only | OpenVINO GPU | Pi 5 (reference) |
|-------|----------|--------------|------------------|
| YOLOv8n 320 | ~50ms (20 FPS) | ~30ms (33 FPS) | 130ms (8 FPS) |
| YOLOv8n 640 | ~150ms (7 FPS) | ~80ms (12 FPS) | 457ms (2 FPS) |
| YOLOv8s 640 | ~400ms (2.5 FPS) | ~200ms (5 FPS) | Too slow |

## Project Options

### Option A: Medium Robot Platform (Recommended)

Indoor/outdoor rover with full autonomous navigation:

```
+------------------+
|    Intel NUC     |  <-- Mounted on platform
|  +------------+  |
|  | 2D Lidar   |  |  <-- RPLidar A1/A2
|  +------------+  |
|  | Depth Cam  |  |  <-- Intel RealSense D435
|  +------------+  |
|  | IMU        |  |  <-- BNO055 or MPU9250
|  +------------+  |
+------------------+
        |
+------------------+
| Motor Controller |  <-- ODrive, Roboclaw, or similar
+------------------+
        |
+------------------+
| 4WD/6WD Chassis  |  <-- 30cm+ platform
+------------------+
```

Capabilities:
- SLAM (build maps autonomously)
- Autonomous navigation (Nav2)
- Object detection and tracking
- Multi-sensor fusion
- Remote operation via web/app

### Option B: Crawler Fleet Controller

Base station for multiple Pi-based crawlers:

```
                    Intel NUC (Base Station)
                           |
        +------------------+------------------+
        |                  |                  |
   Pi Crawler 1       Pi Crawler 2       Pi Crawler 3
```

Features:
- Centralized mission planning
- Aggregate video processing
- Fleet coordination
- Data logging (2TB storage)

### Option C: Robot Arm Platform

Desktop manipulation robot:

```
+------------------+
|    Intel NUC     |
+------------------+
        |
+------------------+
| 6-DOF Robot Arm  |  <-- MoveIt2 control
+------------------+
        |
+------------------+
|  Depth Camera    |  <-- Object detection
+------------------+
```

### Option D: Development Server

Train models and develop code for Pi deployment:

- Faster iteration than Colab
- Local Jupyter notebooks
- Docker containers
- CI/CD for robot code

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
| ROS2_GUIDE.md | ROS2 concepts and getting started |
| CHASSIS_BUILD.md | Custom 24"x12" T-slot chassis build |
| SHOPPING.md | Bill of materials |

## Getting Started

1. Install Ubuntu 24.04 on NUC
2. Install ROS2 Jazzy (see ROS2_GUIDE.md)
3. Choose robot platform
4. Acquire hardware (see SHOPPING.md)
5. Build and integrate

## Related Project

See `/home/kevin/robot/` for Pi 5 Rock Crawler project
