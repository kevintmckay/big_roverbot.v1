# Mid-Rover Chassis Build Guide

A 3D-printed chassis designed for Hiwonder LX-16A serial servos.

---

## Frame Overview

```
                    300mm (12")
    ┌─────────────────────────────────────┐
    │                                     │
    │   ┌─────┐               ┌─────┐     │
    │   │ S1  │               │ S2  │     │  ← Front servos
    │   │ FL  │               │ FR  │     │
    │   └──┬──┘               └──┬──┘     │
    │      │                     │        │
    │      ○                     ○        │  ← 70mm wheels
    │                                     │
    │      ┌─────────────────────┐        │  200mm
    │      │    Electronics      │        │  (8")
    │      │    Tray Area        │        │
    │      └─────────────────────┘        │
    │                                     │
    │      ○                     ○        │  ← 70mm wheels
    │      │                     │        │
    │   ┌──┴──┐               ┌──┴──┐     │
    │   │ S3  │               │ S4  │     │  ← Rear servos
    │   │ RL  │               │ RR  │     │
    │   └─────┘               └─────┘     │
    │                                     │
    └─────────────────────────────────────┘
```

## Printable Parts

### 1. Main Chassis Plate
- **Size:** 300mm x 200mm x 4mm
- **Material:** PETG
- **Weight:** ~150g
- **Features:**
  - 4x servo mount cutouts (45mm x 25mm)
  - Center electronics mounting area
  - M3 mounting holes throughout
  - Ventilation slots

### 2. Servo Mount Brackets (x4)
- **Size:** 50mm x 30mm x 20mm each
- **Material:** PETG
- **Weight:** ~20g each (80g total)
- **Features:**
  - LX-16A mounting pattern (4x M3 holes)
  - Angled for slight ground clearance
  - Strain relief for servo cables

### 3. Electronics Tray
- **Size:** 180mm x 100mm x 15mm
- **Material:** PLA or PETG
- **Weight:** ~60g
- **Mounts:**
  - Raspberry Pi Zero 2W or Pi 5
  - 5V buck converter
  - Servo controller board
  - LiPo voltage alarm

### 4. Sensor Mount - Lidar
- **Size:** 80mm x 60mm x 40mm
- **Material:** PETG
- **Weight:** ~30g
- **Position:** Center front, elevated

### 5. Battery Tray
- **Size:** 150mm x 50mm x 20mm
- **Material:** PETG
- **Weight:** ~25g
- **Features:**
  - Velcro strap slots
  - Sized for 2S 5000mAh pack

---

## Print Settings

| Setting | Value |
|---------|-------|
| Layer height | 0.2mm |
| Infill | 20-30% |
| Perimeters | 3 |
| Material | PETG (recommended) |
| Supports | Yes, for brackets |
| Bed temp | 80°C |
| Nozzle temp | 240°C |

**Total print time:** ~8-10 hours
**Total filament:** ~350g PETG

---

## LX-16A Mounting

The LX-16A servo has 4 mounting holes on its bottom plate:

```
    ┌─────────────────────┐
    │  ●             ●    │   ● = M3 mounting hole
    │                     │
    │        ═══          │   ═══ = Output shaft
    │                     │
    │  ●             ●    │
    └─────────────────────┘
         45.2mm x 24.7mm
```

Mounting hole pattern: 40mm x 20mm (center to center)

Use M3x8mm screws with M3 nuts or heat-set inserts.

---

## Assembly Order

### Step 1: Print All Parts
Print the chassis plate flat. Print servo brackets with the motor face down.

### Step 2: Install Heat-Set Inserts (Optional)
Insert M3 heat-set inserts into the chassis plate mounting points for repeated assembly/disassembly.

### Step 3: Mount Servos to Brackets
Attach each LX-16A to its bracket using 4x M3x8mm screws.

### Step 4: Attach Brackets to Chassis
Secure the servo brackets to the main chassis plate. Ensure all servos are oriented with output shafts facing outward.

### Step 5: Install Couplers and Wheels
1. Attach the goBILDA 25T-to-6mm coupler to each servo's spline output
2. Tighten the coupler's clamping screw
3. Insert a short 6mm stub shaft (or the wheel's built-in coupling) into the coupler
4. Secure the 85mm rubber wheel to the shaft

**Assembly order:** Servo → goBILDA coupler → Wheel hub

### Step 6: Mount Electronics Tray
Attach the electronics tray to the center of the chassis using M3 standoffs.

### Step 7: Install Electronics
- Secure Pi Zero/Pi 5 to tray
- Mount buck converter
- Mount servo controller (if using)
- Install LiPo alarm

### Step 8: Mount Lidar
Attach the lidar mount to the front-center of the chassis. The lidar should be elevated above the wheel height.

### Step 9: Wire Everything
See wiring diagram in SHOPPING_LIST.md.

### Step 10: Install Battery
Velcro the 2S LiPo in the battery tray area.

---

## Ground Clearance

```
                  Side View

    ───────────────────────────────────
                Chassis (4mm)
    ───────────────────────────────────
         │                     │
         │   Servo (36mm)      │
         │   Coupler (6mm)     │
    ─────┴─────           ─────┴─────
        ╱   ╲                 ╱   ╲
       │     │               │     │
       │ 85mm│               │ 85mm│
       │     │               │     │
        ╲   ╱                 ╲   ╱
    ═══════════════════════════════════  Ground

    Ground clearance: 42.5mm (wheel radius) - ~0mm (shaft center)
                    = ~42mm (1.7")
```

With 85mm thick rubber wheels (42.5mm radius) and servo centered, expect ~42mm ground clearance. Good for indoor floors and light outdoor surfaces.

---

## Alternative: Larger Wheels

For more ground clearance and speed, use 90mm industrial wheels:

| Wheel Size | Ground Clearance | Speed | Torque at Ground |
|------------|------------------|-------|------------------|
| 85mm (current) | 42mm | 0.24 m/s | 46 N |
| 90mm | 45mm | 0.25 m/s | 44 N |

[90mm industrial robot wheels](https://www.amazon.com/dp/B08BJY7YX1) with 6mm D-shaft are available on Amazon.

---

## Servo ID Configuration

Before assembly, set each servo's ID using the Hiwonder debug tool:

1. Connect servo controller to PC via USB
2. Connect single servo
3. Use Hiwonder software to set ID
4. Repeat for each servo

| Position | Servo ID |
|----------|----------|
| Front Left | 1 |
| Front Right | 2 |
| Rear Left | 3 |
| Rear Right | 4 |

---

## Continuous Rotation Mode

The LX-16A has two modes:
- **Servo mode:** 0-240° position control
- **Motor mode:** Continuous rotation with speed control

For driving, use **Motor mode**:

```python
# Example: Set servo to motor mode, speed 500 (range: -1000 to 1000)
# Protocol: 0x55 0x55 ID LEN CMD DATA...

# Set to motor mode
send_command(servo_id=1, cmd=0x1D, speed=500)

# Stop
send_command(servo_id=1, cmd=0x1D, speed=0)
```

In motor mode, you lose position feedback. For odometry, you'll need to:
1. Calculate distance from speed × time, or
2. Add external wheel encoders (optional upgrade)

---

## Weight Distribution

```
    ┌─────────────────────────────────────┐
    │                                     │
    │   [Servo+Wheel]     [Servo+Wheel]   │   Front
    │       110g              110g        │
    │                                     │
    │         ┌───────────────┐           │
    │         │  Electronics  │           │
    │         │     200g      │           │
    │         └───────────────┘           │
    │                                     │
    │         ┌───────────────┐           │
    │         │    Battery    │           │
    │         │     270g      │           │
    │         └───────────────┘           │
    │                                     │
    │   [Servo+Wheel]     [Servo+Wheel]   │   Rear
    │       110g              110g        │
    │                                     │
    └─────────────────────────────────────┘

    Per corner: Servo (54g) + Coupler (6g) + Wheel (~50g) = ~110g
```

Center of gravity is roughly centered, with slight rear bias from battery. This is good for traction on the drive wheels.

---

## STL Files Needed

Design these in CAD (Fusion 360, FreeCAD, or similar):

1. `chassis_plate.stl` - Main 300x200mm plate
2. `servo_bracket.stl` - Print 4x
3. `electronics_tray.stl` - Center mount
4. `lidar_mount.stl` - For LD19 or RPLidar A1
5. `battery_tray.stl` - 2S LiPo holder

Alternatively, search Thingiverse/Printables for "LX-16A rover" or adapt existing small rover chassis designs.

---

*Mid-Rover Chassis Build Guide - LX-16A Platform*
