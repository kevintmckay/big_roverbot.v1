# Custom Rover Chassis Build

24" x 12" aluminum T-slot frame with Pololu motors.

## Specifications

### Chassis

| Component | Specification |
|-----------|---------------|
| Dimensions | 24" x 12" (610mm x 305mm) |
| Frame | 1" aluminum T-slot extrusion |
| Joints | Corner brackets, T-nuts, bolts |
| Deck | 0.125" aluminum sheet, bolted to frame |
| Motor mounts | Custom plates (SendCutSend) |
| Ground clearance | ~4" (100mm) |

### Drivetrain (Option B - Selected)

| Component | Specification |
|-----------|---------------|
| Motors | Pololu 37D 131:1, 12V, 76 RPM, with encoders (x4) |
| Wheels | 7.6" (192mm) Wasteland, airless rubber (x4) |
| Hubs | goBILDA 1309 Sonic Hub, 6mm D-Bore (x4) |
| Motor drivers | Cytron MDD10A dual channel (x2) |

### Computed Specs

```
Wheel diameter:     7.6" (192mm)
Motor RPM:          76 RPM (no load)
Gear ratio:         131.25:1
Wheel circumference: 7.6" × π = 23.88" (606mm)

Max speed: 76 RPM × 23.88"/rev = 1815 in/min = 1.5 mph (2.4 km/h)

Motor stall torque:  45 kg-cm
Ground force (4 wheels): 188N total

Good torque for hills and rough terrain.
```

### Weight Budget

| Component | Weight |
|-----------|--------|
| Raspberry Pi 5 | 0.1 kg |
| 4S LiPo 8000mAh | 0.8 kg |
| Frame + plates | 2.5 kg |
| Motors (4x) | 0.8 kg |
| Wheels (4x 192mm) | 3.2 kg |
| Electronics | 0.5 kg |
| **Total** | **~8.3 kg** |

Wheel capacity: 4 × 6kg = 24kg ✓

## Bill of Materials

### Frame (~$100)

| Item | Qty | Source | Est. Price |
|------|-----|--------|------------|
| 1" T-slot extrusion (1m lengths) | 4 | OpenBuilds | $60 |
| Corner brackets, T-nuts, hardware | 1 kit | OpenBuilds | $40 |

### Frame Hardware (~$50)

| Item | Qty | Source | Est. Price |
|------|-----|--------|------------|
| Motor mount plates (custom cut) | 4 | SendCutSend | $40 |
| Pi 5 mounting hardware | 1 set | 3D printed | $0 |

### Drivetrain (~$506)

| Item | Qty | Source | Est. Price |
|------|-----|--------|------------|
| Pololu 37D 131:1 12V w/encoder | 4 | Pololu | $284 |
| 7.6" Wasteland Wheels (192mm) | 4 | ServoCity | $160 |
| goBILDA 1309 Sonic Hub 6mm D-Bore | 4 | goBILDA | $32 |
| Cytron MDD10A motor driver | 2 | Amazon | $30 |

### Total Frame + Drivetrain: ~$656

## Sources

| Category | Suppliers |
|----------|-----------|
| Extrusion | [OpenBuilds](https://openbuildspartstore.com) |
| Motors | [Pololu #4756](https://www.pololu.com/product/4756) |
| Wheels | [ServoCity](https://www.servocity.com/192mm-wasteland-wheel/) |
| Hubs | [goBILDA](https://www.gobilda.com/1309-series-sonic-hub-6mm-d-bore/) |
| Cut plates | [SendCutSend](https://sendcutsend.com) |

## Frame Design

### Top View
```
          24" (610mm)
+-------------------------------+
|  [Pi5]    [Lidar]    [GPS]    |
|                               |  12"
|  [Arduino] [Batt] [Drivers]   |  (305mm)
|                               |
+-------------------------------+
  ||                         ||
[====]                     [====]  <- 7.6" Wasteland wheels
  ||                         ||
[====]                     [====]
  M1,M3                    M2,M4
```

### Side View
```
        +---------------------+
        |    Electronics      |  <- Pi 5, Arduino, drivers
        +---------------------+
           |               |
        [=====]         [=====]   <- 7.6" wheels (192mm)
           |               |
    ─────────────────────────────  Ground
              ~4" clearance
```

### Overall Dimensions with Wheels
```
Total width:  12" + 2×2.8" = ~17.6" (447mm)
Total length: 24" (610mm)
Total height: ~8" (frame + electronics)
```

## Motor Wiring

### Pololu 37D Pinout
```
Motor: Red (+), Black (-)
Encoder:
  - Green:  Encoder GND
  - Blue:   Encoder Vcc (3.5-20V)
  - Yellow: Encoder A output
  - White:  Encoder B output
```

### Cytron MDD10A Connections
```
MDD10A Driver #1 (Left side)
  - Motor A: Front Left (M1)
  - Motor B: Rear Left (M3)
  - PWM1/DIR1: Arduino pins
  - PWM2/DIR2: Arduino pins

MDD10A Driver #2 (Right side)
  - Motor A: Front Right (M2)
  - Motor B: Rear Right (M4)
  - PWM3/DIR3: Arduino pins
  - PWM4/DIR4: Arduino pins
```

### Arduino Mega Pin Assignments
```
Motor Control (8 pins):
  - Pin 2:  PWM1 (M1 speed)
  - Pin 3:  DIR1 (M1 direction)
  - Pin 4:  PWM2 (M3 speed)
  - Pin 5:  DIR2 (M3 direction)
  - Pin 6:  PWM3 (M2 speed)
  - Pin 7:  DIR3 (M2 direction)
  - Pin 8:  PWM4 (M4 speed)
  - Pin 9:  DIR4 (M4 direction)

Encoder Inputs (8 pins):
  - Pin 18: M1 Encoder A (interrupt)
  - Pin 19: M1 Encoder B (interrupt)
  - Pin 20: M2 Encoder A (interrupt)
  - Pin 21: M2 Encoder B (interrupt)
  - Pin 22: M3 Encoder A
  - Pin 23: M3 Encoder B
  - Pin 24: M4 Encoder A
  - Pin 25: M4 Encoder B
```

## Power Distribution

### Power Architecture
```
4S LiPo (14.8V nominal, 8000mAh)
         │
         ├──→ 20A Fuse
         │         │
         │         ├──→ Buck 12V 10A ──→ MDD10A #1 VIN ──→ M1, M3
         │         │                 └──→ MDD10A #2 VIN ──→ M2, M4
         │         │
         │         └──→ Buck 5V 5A ───→ Raspberry Pi 5 (USB-C)
         │                           ├──→ Arduino Mega (USB from Pi)
         │                           └──→ Encoder Vcc
         │
         └──→ E-Stop ──→ Motor power cutoff
```

### Power Budget
```
Pi 5:          8W typical (5V × 1.6A)
Motors (4x):   48W peak (12V × 1A each typical)
Arduino:       0.5W (powered via Pi USB)
Sensors:       2W
─────────────────────────
Total:         ~58W typical, ~85W peak

Runtime: 8000mAh × 14.8V = 118Wh
         118Wh / 58W = ~2 hours typical
```

## Control Architecture

### System Overview
```
                            USB Serial
    Raspberry Pi 5 (ROS2) ←────────────→ Arduino Mega 2560
         │                                      │
         │                                 PWM/DIR signals
         │                                      │
         │                              Cytron MDD10A (x2)
         │                                      │
         │                              Pololu 37D Motors (x4)
         │                                      │
         │                              Encoder feedback (x4)
         │
         ├── USB ──→ FTDI ──→ TN GPS
         ├── USB ──→ RPLidar A1
         ├── USB ──→ Intel RealSense D435
         └── USB ──→ FT232H ──→ BNO055 IMU (I2C)
```

### Why Arduino?
Pi 5 GPIO is limited and not ideal for real-time motor control. Arduino Mega provides:
- PWM outputs for motor speed control
- Digital outputs for direction control
- Interrupt-capable inputs for encoder counting
- USB serial communication with Pi 5

## Assembly Steps

1. **Cut extrusions** to length (or order pre-cut)
   - 2x 24" lengths (sides)
   - 2x 12" lengths (ends)
   - Optional cross braces

2. **Assemble frame** with corner brackets
   - Use T-nuts and bolts
   - Square the frame before tightening

3. **Mount motors**
   - Attach custom motor plates to T-slot frame
   - Bolt Pololu 37D motors to plates (M3 screws)
   - Motors mount on outside of frame

4. **Install wheels**
   - Press goBILDA 1309 hubs onto motor D-shafts
   - Tighten hub pinch bolts
   - Attach Wasteland wheels to hubs (M4 screws)

5. **Mount electronics**
   - Pi 5 in aluminum case (center)
   - Arduino Mega near motor drivers
   - MDD10A drivers near motors (short motor wires)

6. **Wire power system**
   - Fuse near battery
   - DC-DC converters on frame
   - E-stop accessible from outside

7. **Wire motors**
   - Motor power to MDD10A outputs
   - PWM/DIR from Arduino to MDD10A inputs
   - Encoders to Arduino digital pins

8. **Test**
   - Power on, verify motor directions
   - Test encoder counting
   - Calibrate odometry

## ROS2 Integration

### Motor Control Node (Arduino)
```cpp
// Arduino receives cmd_vel via serial
// Converts to individual motor speeds (differential drive)
// Reads encoders, sends odometry back to Pi

// Serial protocol:
// Pi → Arduino: "V,left_speed,right_speed\n"
// Arduino → Pi: "O,left_ticks,right_ticks\n"
```

### ROS2 Nodes (Pi 5)
```
ros2_serial_bridge    - Arduino communication
diff_drive_controller - cmd_vel to wheel speeds
robot_localization    - sensor fusion (encoders + IMU + GPS)
rplidar_ros          - lidar driver
realsense_ros        - depth camera driver
nav2                 - navigation stack
slam_toolbox         - SLAM
```

### Encoder Specs (Pololu 37D 131:1)
```
Counts per revolution: 64 CPR (motor shaft)
Gear ratio: 131.25:1
Counts per wheel rev: 64 × 131.25 = 8,400 CPR
Wheel circumference: 606mm

Resolution: 606mm / 8400 = 0.072mm per count

At max speed (76 RPM):
  Ticks/sec = 76 × 8400 / 60 = 10,640 ticks/sec per motor
  Arduino can handle this with interrupts
```

## Sensors

### Lidar: RPLidar A1

| Spec | Value |
|------|-------|
| Model | Slamtec RPLidar A1M8 |
| Range | 0.15 - 12m |
| Scan Rate | 5.5Hz |
| Interface | USB |
| Price | ~$99 |

**Mounting:**
```
Height: 15-20cm above deck (clears 7.6" wheels)
Location: Center-front or center-top
Clearance: 360° unobstructed view
```

### Depth Camera: Intel RealSense D435

| Spec | Value |
|------|-------|
| Resolution | 1280×720 |
| Range | 0.2 - 10m |
| Interface | USB 3.0 |
| Price | ~$180 |

### IMU: BNO055

| Spec | Value |
|------|-------|
| Type | 9-DOF (accel + gyro + mag) |
| Interface | I2C via FT232H USB adapter |
| Price | ~$25 |

### GPS: TN GPS with Compass (owned)

| Spec | Value |
|------|-------|
| Interface | UART via FTDI USB adapter |
| Features | GPS + magnetometer |

## Motor Mount Design

### Pololu 37D Mounting Pattern
```
Front view of motor:

        ┌─────────┐
        │  shaft  │
        │    ○    │  <- 6mm D-shaft
        │         │
    ────┼────○────┼────  <- M3 mounting holes
        │         │       0.875" (22.2mm) bolt circle
        │    ○    │       6 holes, 60° apart
        │         │
    ────┼────○────┼────
        │         │
        └─────────┘
            37mm diameter
```

### Mount Plate Design (SendCutSend)
```
Material: 3mm aluminum
Size: ~60mm × 80mm per plate

Features:
- 3x M3 holes for motor (0.875" bolt circle)
- 2x slots for T-slot mounting
- Cutout for motor shaft
```

## Safety Features

| Feature | Purpose |
|---------|---------|
| 20A Fuse | Overcurrent protection |
| E-Stop | Emergency motor cutoff |
| LiPo Alarm | Low voltage warning |
| Aluminum case | Passive cooling for Pi 5 |

---

*Last updated: 2024-12-27*
