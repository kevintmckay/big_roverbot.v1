# Mid-Rover: LX-16A Serial Bus Rover

A compact, lightweight rover built around Hiwonder LX-16A serial bus servos.

## Design Philosophy

The LX-16A servos offer excellent value with built-in feedback, but their torque and voltage specs require a lighter platform. This rover is designed for:
- Indoor navigation and mapping
- Light outdoor use (sidewalks, short grass)
- Educational robotics and ROS2 learning
- Budget-friendly build (~$400-500)

## Specifications

| Spec | Value |
|------|-------|
| Chassis | 12" x 8" (300mm x 200mm) |
| Motors | 4x Hiwonder LX-16A serial servos |
| Wheels | 85mm diameter, 33mm wide thick rubber |
| Max Speed | ~0.5 mph (0.24 m/s) |
| Ground Clearance | ~1.7" (42mm) |
| Target Weight | 2.0-2.5 kg |
| Battery | 2S LiPo 2200-5000mAh |
| Runtime | ~1.5-2 hours |
| Control | Raspberry Pi Zero 2W or Pi 5 |

## Key Differences from RoverBot v1

| Aspect | Mid-Rover | RoverBot v1 |
|--------|-----------|-------------|
| Motors | LX-16A servos ($64) | Pololu 37D ($284) |
| Motor drivers | None needed | 2x Cytron MDD10A |
| Voltage | 7.4V (2S) | 12V (via buck from 4S) |
| Wheels | 85mm thick rubber | 192mm Wasteland |
| Frame | 3D printed | Aluminum extrusion |
| Weight | ~1.5 kg | ~8.3 kg |
| Terrain | Indoor/light outdoor | All-terrain |
| Cost | ~$320 | ~$1000 |

## Servo Wiring

The LX-16A uses a daisy-chain serial bus - all 4 servos connect to a single UART:

```
Pi/Arduino TX ──┬── Servo 1 ──┬── Servo 2 ──┬── Servo 3 ──┬── Servo 4
                │             │             │             │
              ID:1          ID:2          ID:3          ID:4
```

- Baud rate: 115200
- Protocol: Hiwonder serial protocol (documented)
- Each servo has unique ID (0-253)
- Feedback: position, temperature, voltage

## Wheel Configuration

```
        FRONT
    ┌───────────┐
    │ [1]   [2] │   Servo IDs:
    │           │   1 = Front Left
    │           │   2 = Front Right
    │           │   3 = Rear Left
    │ [3]   [4] │   4 = Rear Right
    └───────────┘
        REAR
```

## Power Architecture

```
2S LiPo (7.4V)
    │
    ├── LX-16A Servos (direct, 6-8.4V) ─── ~4A peak
    │
    └── 5V Buck Converter
            │
            ├── Raspberry Pi ─── ~2A
            └── Sensors ─── ~0.5A

Total: ~30W peak, ~15W cruise
```

## Documents

- [SHOPPING_LIST.md](SHOPPING_LIST.md) - Complete parts list with links
- [CHASSIS_BUILD.md](CHASSIS_BUILD.md) - Frame design and assembly

## Variants

**Want a bigger rover?** See [Mid-Rover XL](../mid-rover-xl/) - same LX-16A servos, but:
- 500×350mm chassis (vs 300×200mm)
- 100mm RC crawler wheels with deep lugs
- ~4.5 kg with 1.7 kg payload capacity
- Aluminum tube frame
- ~$540 vs ~$320

## Torque Analysis

LX-16A with 85mm wheels:
- Torque: 20 kg.cm = 1.96 Nm
- Wheel radius: 42.5mm = 0.0425m
- Force per wheel: 1.96 / 0.0425 = **46 N**
- 4 wheels total: **184 N thrust**
- Rover weight: 1.5 kg = 14.7 N

Thrust-to-weight ratio: **12:1** - excellent for indoor and light outdoor use.

## Speed Calculation

- Servo speed: 0.19 sec/60° at 7.4V = 316°/sec
- In continuous mode: ~53 RPM equivalent
- Wheel circumference: π × 85mm = 267mm
- Speed: 53 × 0.267m/min = 14.2 m/min = **0.24 m/s (0.5 mph)**

Slower than RoverBot v1 (1.5 mph) but adequate for indoor mapping.

## Sensor Options

### Minimum (SLAM-capable)
- RPLidar A1 (or cheaper LD19)

### Enhanced
- RPLidar A1 + IMU (BNO055 or MPU6050)
- Optional: Ultrasonic sensors for close obstacle detection

### No depth camera recommended
The D435 is overkill for this platform. A simple 2D lidar is sufficient.

## Compute Options

### Option A: Raspberry Pi Zero 2W (~$15)
- Lightweight, low power
- Adequate for basic SLAM
- Limited USB (need hub for lidar)

### Option B: Raspberry Pi 5 (already owned)
- Overkill but works
- Easy development
- Higher power draw

### Option C: ESP32 + Remote Compute
- ESP32 handles motor control
- Streams sensor data to laptop/desktop
- Lowest on-board cost

---

*A simpler, cheaper path to learning ROS2 and mobile robotics.*
