# Project 2 Shopping List

Hardware for Intel NUC robot platform with ROS2.

## Already Owned

| Component | Details | Status |
|-----------|---------|--------|
| Intel NUC | Celeron N5105, 32GB RAM, 2TB NVMe | Confirmed |
| Zippy Flightmax 4S LiPo | 8000mAh, 14.8V, 30C | Confirmed - main battery |
| Turnigy nano-tech 4S LiPo | 1800mAh, 14.8V, 65-130C | Confirmed - testing/backup |
| TN GPS with Compass | GPS + magnetometer | Available - TBD |
| Generic GPS (1SU6 V1.1) | UART GPS module | Available - TBD |

See `old-parts.md` for full inventory of available parts.

## Selected Build: Custom T-Slot Chassis

See `CHASSIS_BUILD.md` for full details.

### Summary

| Category | Cost |
|----------|------|
| Frame + hardware | $100-150 |
| Drivetrain | $230-250 |
| Lidar | $99 |
| **Rover Total** | **$450-500** |

### Key Components

| Component | Spec | Qty | Source |
|-----------|------|-----|--------|
| Pololu 37D motor | 12V, 131 RPM, encoder | 4 | Pololu.com |
| Cytron MDD10A | Dual motor driver | 2 | Cytron/Amazon |
| 6" wheels | Aluminum hub | 4 | ServoCity |
| T-slot extrusion | 1" or 1.5" aluminum | 4-6m | OpenBuilds |
| RPLidar A1 | 12m range, 5.5Hz | 1 | Slamtec/Amazon |

## Option A: Medium Robot Platform

Full autonomous navigation rover.

### Tier 1: Essential (~$200-350)

| Component | Purpose | Price | Links |
|-----------|---------|-------|-------|
| Robot Chassis (4WD/6WD) | Base platform | $50-150 | See chassis options below |
| Motor Driver | Control motors | $30-60 | Roboclaw 2x7A, ODrive |
| 2D Lidar | SLAM and navigation | $99-200 | RPLidar A1 ($99), A2 ($300) |
| ~~12V Battery~~ | ~~Power system~~ | ~~$30-50~~ | **Already owned** (Zippy 8000mAh 4S) |
| DC-DC Converters | Voltage regulation | $15-25 | 19V for NUC, 5V for sensors |
| Wiring/Connectors | Integration | $20-30 | XT60, JST, barrel jacks |

**Tier 1 Total: ~$220-450** (saved ~$50 on battery)

### Tier 2: Enhanced Sensing (~$150-300)

| Component | Purpose | Price | Links |
|-----------|---------|-------|-------|
| Depth Camera | 3D perception | $150-250 | Intel RealSense D435 |
| IMU | Orientation | $15-30 | BNO055, MPU9250 |
| Wheel Encoders | Odometry | $20-40 | Optical or magnetic |
| USB Hub | Expand ports | $15-25 | Powered USB 3.0 |

**Tier 2 Total: ~$200-350**

### Tier 3: Optional Upgrades

| Component | Purpose | Price |
|-----------|---------|-------|
| 3D Lidar | Better mapping | $200-500 |
| GPS Module | Outdoor nav | $20-50 | *Have TN GPS w/compass available* |
| Robot Arm | Manipulation | $200-800 |
| Speaker/Mic | Voice interaction | $30-50 |

## Chassis Options

### Ready-to-Use Platforms

| Platform | Size | Drive | Price | Notes |
|----------|------|-------|-------|-------|
| Yahboom ROSMASTER X3 | 30cm | 4WD Mecanum | $300-400 | ROS2 ready, includes lidar |
| Waveshare JETBOT | 20cm | 2WD | $100-150 | Compact, needs adaptation |
| Agilex Scout Mini | 50cm | 4WD | $3000+ | Professional, outdoor |
| Clearpath Jackal | 50cm | 4WD | $10000+ | Research grade |

### DIY Chassis

| Type | Size | Price | Notes |
|------|------|-------|-------|
| Aluminum 4WD Platform | 25-40cm | $50-100 | AliExpress/Amazon |
| Rocker-Bogie Kit | 40-60cm | $100-200 | Mars rover style |
| 6WD Wild Thumper | 40cm | $150-250 | Powerful, outdoor |
| Mecanum Wheel Kit | 30cm | $80-150 | Omnidirectional |

## Lidar Options

| Model | Range | Rate | Price | Notes |
|-------|-------|------|-------|-------|
| RPLidar A1 | 12m | 5.5Hz | $99 | Budget, good for indoor |
| RPLidar A2 | 12m | 10Hz | $300 | Faster, more reliable |
| YDLidar X4 | 10m | 6Hz | $70 | Cheapest option |
| Slamtec Mapper | 20m | 20Hz | $400 | High performance |
| SICK TIM | 25m | 15Hz | $1500+ | Industrial grade |

## Depth Camera Options

| Model | Resolution | Range | Price | Notes |
|-------|------------|-------|-------|-------|
| Intel RealSense D435 | 1280x720 | 0.2-10m | $180 | Best for ROS2 |
| Intel RealSense D455 | 1280x800 | 0.4-6m | $250 | Better outdoor |
| Orbbec Astra | 640x480 | 0.4-8m | $150 | Budget option |
| OAK-D | 1280x800 | 0.2-20m | $200 | Includes AI chip |
| Kinect Azure | 1024x1024 | 0.25-5m | $400 | Highest quality |

## Motor Controller Options

| Controller | Channels | Current | Price | Notes |
|------------|----------|---------|-------|-------|
| Roboclaw 2x7A | 2 | 7A cont. | $80 | Easy, encoders built-in |
| Roboclaw 2x15A | 2 | 15A cont. | $120 | More power |
| ODrive v3.6 | 2 | 60A peak | $150 | High performance BLDC |
| Cytron MD10C | 1 | 10A cont. | $15 | Simple, cheap |
| Sabertooth 2x12 | 2 | 12A cont. | $80 | Reliable, analog/RC |

## Power System

### Battery Options

| Type | Voltage | Capacity | Price | Notes |
|------|---------|----------|-------|-------|
| 4S LiPo | 14.8V | 5000mAh | $50-80 | Lightweight, needs charger |
| 6S LiPo | 22.2V | 5000mAh | $70-100 | More headroom |
| 12V SLA | 12V | 7Ah | $25-35 | Heavy, cheap, safe |
| LiFePO4 | 12.8V | 6Ah | $50-80 | Safe, long life |

### Power Distribution

```
Battery (14.8V-22V)
      |
      +---> DC-DC 19V --> Intel NUC (barrel jack)
      |
      +---> DC-DC 12V --> Motor driver, Lidar
      |
      +---> DC-DC 5V  --> Sensors, USB hub
```

| Converter | Output | Current | Price |
|-----------|--------|---------|-------|
| 19V Step-down | 19V | 4A | $15-25 |
| 12V Step-down | 12V | 5A | $10-15 |
| 5V Step-down | 5V | 5A | $8-12 |

## Recommended Build: Indoor Rover

Balanced cost and capability:

| Component | Selection | Price |
|-----------|-----------|-------|
| Chassis | 4WD Aluminum 30cm | $80 |
| Motors | DC geared with encoders | (included) |
| Motor Driver | Roboclaw 2x7A | $80 |
| Lidar | RPLidar A1 | $99 |
| Depth Camera | Intel RealSense D435 | $180 |
| IMU | BNO055 | $25 |
| ~~Battery~~ | ~~4S LiPo 5000mAh~~ | ~~$60~~ | **Owned** (Zippy 8000mAh) |
| Power Board | Custom with DC-DC | $40 |
| Mounting | Standoffs, plates | $30 |
| Wiring | Connectors, cables | $25 |
| **Total** | | **~$560** (was $620) |

## Option B: Fleet Controller

Base station for Pi crawlers (no mobile hardware needed):

| Component | Purpose | Price |
|-----------|---------|-------|
| WiFi Router | Dedicated robot network | $30-50 |
| Monitor (optional) | Dashboard display | $100-150 |
| UPS | Backup power | $50-100 |

**Total: ~$180-300** (most cost is Pi crawlers)

## Software (Free)

| Software | Purpose |
|----------|---------|
| Ubuntu 24.04 | Operating system |
| ROS2 Jazzy | Robot middleware |
| Nav2 | Navigation stack |
| slam_toolbox | SLAM |
| OpenVINO | AI acceleration |
| Gazebo | Simulation |

## Where to Buy

### Robotics Specialists
- RobotShop: https://www.robotshop.com/
- Pololu: https://www.pololu.com/
- SparkFun: https://www.sparkfun.com/
- Adafruit: https://www.adafruit.com/

### Lidar/Cameras
- Slamtec (RPLidar): https://www.slamtec.com/
- Intel RealSense: https://www.intelrealsense.com/

### Budget Options
- AliExpress: Chassis, motors, basic sensors
- Amazon: Quick delivery, returns

## Summary by Budget

| Budget | What You Get |
|--------|--------------|
| $300 | Basic chassis + motors + lidar (no depth) |
| $500 | Above + depth camera + IMU |
| $700 | Above + better lidar + encoders |
| $1000+ | High-end sensors, professional chassis |
