# RoverBot v1

Autonomous rover platform built on Intel NUC with ROS2.

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
| Runtime | ~1.8 hours |
| Brain | Intel NUC (ROS2 Jazzy) |

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

## Architecture

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

1. Install Ubuntu 24.04 on NUC
2. Install ROS2 Jazzy (see ROS2_GUIDE.md)
3. Acquire hardware (see SHOPPING.md)
4. Build chassis (see CHASSIS_BUILD.md)
5. Integrate and test

## Related Project

See `/home/kevin/robot/` for Pi 5 Rock Crawler project
