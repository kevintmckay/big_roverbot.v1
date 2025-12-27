# Custom Rover Chassis Build

24" x 12" aluminum T-slot frame with Pololu motors.

## Specifications

### Chassis

| Component | Specification |
|-----------|---------------|
| Dimensions | 24" x 12" (610mm x 305mm) |
| Frame | 1" or 1.5" aluminum T-slot extrusion |
| Joints | Corner brackets, T-nuts, bolts |
| Deck | 0.125" aluminum sheet, bolted to frame |
| Motor mounts | Custom plates (SendCutSend or similar) |

### Drivetrain

| Component | Specification |
|-----------|---------------|
| Motors | Pololu 37D, 12V, 131 RPM, with encoders (x4) |
| Wheels | 6" diameter, aluminum hub (x4) |
| Hubs | 6mm D-shaft adapters (x4) |
| Motor drivers | Cytron MDD10A dual channel (x2) |

### Computed Specs

```
Wheel diameter:    6" (152mm)
Motor RPM:         131 RPM (no load)
Wheel circumference: 6" × π = 18.85" (478mm)

Max speed: 131 RPM × 18.85"/rev = 2469 in/min = 3.4 mph (5.5 km/h)

Good speed for outdoor rover with sensors.
```

## Bill of Materials

### Frame (~$100-150)

| Item | Qty | Source | Est. Price |
|------|-----|--------|------------|
| 1" T-slot extrusion (1m lengths) | 4-6 | 80/20, OpenBuilds, Misumi | $40-60 |
| Corner brackets | 8-12 | OpenBuilds, Amazon | $20-30 |
| T-nuts (pack) | 50+ | OpenBuilds, Amazon | $10-15 |
| Bolts/hardware kit | 1 | McMaster-Carr | $15-20 |
| 0.125" aluminum deck plate | 1 | SendCutSend, local metal | $20-30 |
| Motor mount plates (custom cut) | 4 | SendCutSend | $20-40 |

### Drivetrain (~$230-250)

| Item | Qty | Source | Est. Price |
|------|-----|--------|------------|
| Pololu 37D 12V 131RPM w/encoder | 4 | Pololu.com | $140 ($35 ea) |
| 6" wheels with aluminum hub | 4 | ServoCity, AndyMark | $60-80 |
| 6mm D-shaft hub adapters | 4 | ServoCity | $15-20 |
| Cytron MDD10A motor driver | 2 | Cytron, Amazon | $30-40 |

### Total: ~$350-400

## Sources

| Category | Suppliers |
|----------|-----------|
| Extrusion | [80/20](https://8020.net), [OpenBuilds](https://openbuildspartstore.com), [Misumi](https://us.misumi-ec.com) |
| Motors | [Pololu](https://www.pololu.com/product/4756) |
| Wheels/Hubs | [ServoCity](https://www.servocity.com), [AndyMark](https://www.andymark.com) |
| Cut plates | [SendCutSend](https://sendcutsend.com) |
| Hardware | [McMaster-Carr](https://www.mcmaster.com) |

## Frame Design

### Top View
```
        24" (610mm)
+---------------------------+
|  [NUC]   [Lidar]   [GPS]  |
|                           |  12"
|  [Driver] [Batt] [Driver] |  (305mm)
|                           |
+---------------------------+
[M1]                     [M2]
[W1]                     [W2]
[W3]                     [W4]
[M3]                     [M4]
```

### Side View
```
     +-------------------+
     |    Electronics    |
     +-------------------+
        |             |
      [===]         [===]   <- 6" wheels
        |             |
     +-------------------+
     |      Deck         |
     +-------------------+
```

### T-Slot Frame Assembly
```
Corner detail:

    +=======+
    |       |
    |   +---+---
    |   | bracket
    +===+===+---
        |
        |
```

## Motor Wiring

### Pololu 37D Pinout
```
Motor: Red (+), Black (-)
Encoder:
  - Green: Encoder GND
  - Blue:  Encoder Vcc (3.5-20V)
  - Yellow: Encoder A output
  - White:  Encoder B output
```

### Cytron MDD10A Connections
```
MDD10A Driver #1 (Left side)
  - Motor A: Front Left (M1)
  - Motor B: Rear Left (M3)
  - PWM1/DIR1: Pi GPIO or NUC
  - PWM2/DIR2: Pi GPIO or NUC

MDD10A Driver #2 (Right side)
  - Motor A: Front Right (M2)
  - Motor B: Rear Right (M4)
  - PWM3/DIR3: Pi GPIO or NUC
  - PWM4/DIR4: Pi GPIO or NUC
```

### Power Distribution
```
Battery (12V LiPo 3S or 4S with regulator)
    |
    +---> MDD10A #1 VIN ---> Motors M1, M3
    |
    +---> MDD10A #2 VIN ---> Motors M2, M4
    |
    +---> 19V Buck --------> Intel NUC
    |
    +---> 5V Buck ---------> Sensors, encoders
```

## Integration with Existing Hardware

### From Quad/Drone Inventory

| Component | Use in Rover |
|-----------|--------------|
| LiPo batteries (3S/4S) | Main power (with voltage check) |
| Flight controller (Pixhawk) | IMU + sensor fusion (optional) |
| GPS module | Outdoor navigation |
| Telemetry radio | Remote monitoring |

### NUC Mounting

```
T-slot mounting options:

1. Direct bolt through deck plate
2. Rubber standoffs for vibration isolation
3. Quick-release plate for easy removal

Recommended: Rubber standoffs + aluminum plate
```

## Assembly Steps

1. **Cut extrusions** to length (or order pre-cut)
   - 2x 24" lengths (sides)
   - 2x 12" lengths (ends)
   - Optional cross braces

2. **Assemble frame** with corner brackets
   - Use T-nuts and bolts
   - Square the frame before tightening

3. **Attach deck plate**
   - Drill mounting holes
   - Use T-nuts from below

4. **Mount motors**
   - Attach custom motor plates to frame
   - Bolt Pololu 37D motors to plates
   - Align with wheels

5. **Install wheels**
   - Attach hub adapters to motor shafts
   - Mount wheels to hubs

6. **Wire motors**
   - Connect motors to MDD10A drivers
   - Connect encoders to controller

7. **Mount electronics**
   - NUC, drivers, power distribution
   - Route and secure wiring

8. **Test**
   - Power on, verify motor directions
   - Test encoder feedback
   - Calibrate if needed

## ROS2 Integration

### Motor Control Node
```python
# Uses Cytron MDD10A via GPIO PWM
# Encoder feedback for odometry
# Publishes: /odom
# Subscribes: /cmd_vel
```

### Encoder Specs (Pololu 37D)
```
Counts per revolution: 64 CPR (motor shaft)
Gear ratio: ~131:1
Counts per wheel rev: 64 × 131 = 8,384 CPR
Wheel circumference: 478mm

Resolution: 478mm / 8384 = 0.057mm per count
```

High resolution odometry - excellent for ROS2 Nav2.

## Sensors

### Lidar: RPLidar A1 (Recommended)

| Spec | Value |
|------|-------|
| Model | Slamtec RPLidar A1M8 |
| Range | 0.15 - 12m |
| Scan Rate | 5.5Hz |
| Sample Rate | 8KHz |
| Interface | USB (UART adapter included) |
| Price | ~$99 |

**Mounting:**
```
Height: 10-15cm above deck
Location: Center-front or center-top
Clearance: 360° unobstructed view

        [Lidar]  <- 10-15cm standoff
           |
+----------+----------+
|       Deck          |
+---------------------+
```

**ROS2 Setup:**
```bash
# Install
sudo apt install ros-jazzy-rplidar-ros

# Launch
ros2 launch rplidar_ros rplidar_a1_launch.py

# Publishes: /scan (sensor_msgs/LaserScan)
```

### Lidar Options Comparison

| Model | Range | Scan Rate | Price | Notes |
|-------|-------|-----------|-------|-------|
| RPLidar A1 | 12m | 5.5Hz | $99 | Best value, recommended |
| RPLidar A2 | 16m | 10Hz | $300 | Brushless, faster, quieter |
| RPLidar C1 | 12m | 10Hz | $80 | Newer budget option |
| YDLidar X4 | 10m | 6Hz | $70 | Budget, less support |

### Other Sensors (from quad inventory)

| Sensor | Source | Use |
|--------|--------|-----|
| GPS | Flight controller | Outdoor waypoint navigation |
| IMU | Flight controller | Orientation, sensor fusion |
| Barometer | Flight controller | Altitude (optional) |

### Depth Camera (Optional Upgrade)

| Model | Price | Use Case |
|-------|-------|----------|
| Intel RealSense D435 | $180 | 3D obstacle detection |
| OAK-D Lite | $150 | AI + depth |

## Future Upgrades

| Upgrade | Purpose | Est. Cost |
|---------|---------|-----------|
| Suspension | Rough terrain | $50-100 |
| Larger wheels (8") | Ground clearance | $80-120 |
| Brushless motors | More power | $200+ |
| Weatherproofing | Outdoor use | $30-50 |
| Depth camera | 3D perception | $150-200 |
