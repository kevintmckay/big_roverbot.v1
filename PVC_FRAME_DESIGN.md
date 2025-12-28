# PVC Frame Design with Flat Bar Motor Mounts

Alternative low-cost frame using 1" PVC pipe, 3D printed connectors, and aluminum flat bar for motor mounts.

**Why 1" PVC?** 2.3x stiffer than 3/4" for only ~$2 more.

---

## Overview

```
                        24" (610mm)
    ←─────────────────────────────────────────→

    ┌─────────────────────────────────────────┐   ↑
    │ ○                                     ○ │   │
    │ ║                                     ║ │   │
    │ ║   FRONT                             ║ │   │
    │ ║                                     ║ │   │
    │ ○─────────────────────────────────────○ │   │  12" (305mm)
    │ ║           CROSS BRACE               ║ │   │
    │ ○─────────────────────────────────────○ │   │
    │ ║                                     ║ │   │
    │ ║   REAR                              ║ │   │
    │ ║                                     ║ │   │
    │ ○                                     ○ │   ↓
    └─────────────────────────────────────────┘

    ○ = 3D printed corner connector
    ║ = 1" Schedule 40 PVC pipe (1.315" OD)
```

---

## Bill of Materials

### PVC Pipe - ~$10

| Qty | Item | Length | Source |
|-----|------|--------|--------|
| 2 | 1" Sch 40 PVC | 24" (610mm) | Front/rear rails |
| 4 | 1" Sch 40 PVC | 10" (254mm) | Side rails |
| 2 | 1" Sch 40 PVC | 22" (559mm) | Cross braces |

**Buy:** One 10' stick of 1" Schedule 40 PVC (~$8-10 at Home Depot/Lowes)

**Specs:** 1.315" OD, 1.049" ID, 0.133" wall thickness

### Aluminum Flat Bar (Motor Mounts) - ~$15

| Qty | Item | Size | Notes |
|-----|------|------|-------|
| 4 | Aluminum flat bar | 1.5" x 3" x 1/8" | Motor mount plates |
| 4 | Aluminum angle | 1" x 1" x 3" x 1/8" | PVC-to-flat bar brackets |

**Buy:**
- 1x 36" aluminum flat bar 1.5" x 1/8" (~$8)
- 1x 36" aluminum angle 1" x 1" x 1/8" (~$7)

### 3D Printed Parts - ~$5 filament

| Qty | Part | Material | Notes |
|-----|------|----------|-------|
| 8 | Corner connector | PETG | Frame corners |
| 4 | T-connector | PETG | Cross brace joints |
| 4 | Motor mount clamp | PETG | Clamps flat bar to PVC |
| 4 | Pipe clamp (NUC/electronics) | PETG | Component mounting |

### Hardware - ~$10

| Qty | Item | Notes |
|-----|------|-------|
| 16 | M5 x 16mm bolts | Printed connectors |
| 16 | M5 nuts | Printed connectors |
| 12 | M3 x 8mm bolts | Motor mounting |
| 8 | M4 x 12mm bolts | Flat bar to angle |
| 1 | PVC primer + cement | Optional, for permanent joints |

### Total Frame Cost: ~$40

vs T-slot aluminum: ~$100
**Savings: ~$60**

---

## Printed Connector Designs

### 1. Corner Connector (x8)

```
Top View:
        ┌───────────┐
        │   ┌───┐   │
        │   │ ○ │   │  ← PVC socket (33.5mm ID for 1" pipe OD)
        │   └───┘   │
        │     │     │
        └─────┼─────┘
              │
           ┌──┴──┐
           │  ○  │  ← PVC socket
           └─────┘

Side View:
    ┌─────────────┐
    │  ╔═══════╗  │  ← Socket depth: 25mm
    │  ║  PVC  ║  │
    │  ╚═══════╝  │
    │      │      │
    │  ╔═══╧═══╗  │
    │  ║  PVC  ║  │
    │  ╚═══════╝  │
    └─────────────┘

    Bolt holes (M5) through both walls for clamping
```

**Print settings:**
- Material: PETG (strength + slight flex)
- Infill: 50%+
- Walls: 4+
- Layer: 0.2mm

### 2. T-Connector (x4)

```
Top View:
              ┌───┐
              │ ○ │  ← Cross brace socket
              └─┬─┘
        ┌───────┴───────┐
        │ ○ │     │ ○ │  ← Side rail sockets
        └───┘     └───┘

    Three-way connector for cross braces
```

### 3. Motor Mount Clamp (x4)

```
Side View:
    ╔═══════════════════╗
    ║   Flat bar slot   ║  ← Aluminum flat bar slides in
    ╠═══════════════════╣
    ║                   ║
    ║   ┌───────────┐   ║
    ║   │    PVC    │   ║  ← Clamps around PVC pipe
    ║   │   socket  │   ║
    ║   └───────────┘   ║
    ║                   ║
    ╚═════════╦═════════╝
              ║
         Bolt clamp

Front View:
    ┌─────────────────┐
    │ ═══════════════ │  ← Flat bar slot (1.5" x 1/8")
    │                 │
    │    ┌───────┐    │
    │    │   ○   │    │  ← PVC pipe
    │    └───────┘    │
    │   ╔═══╗ ╔═══╗   │  ← M5 clamp bolts
    └───╩═══╩═╩═══╩───┘
```

### 4. Pipe Clamp for Components (x4+)

```
    ┌─────────────────────┐
    │   Mounting holes    │  ← M3 holes for components
    │   ○     ○     ○     │
    ├─────────────────────┤
    │      ┌───────┐      │
    │      │  PVC  │      │  ← Clamps to pipe
    │      └───────┘      │
    │     ╔═╗     ╔═╗     │
    └─────╩═╩─────╩═╩─────┘
           Clamp bolts
```

---

## Aluminum Motor Mount Detail

```
Top View (per motor):

    ┌─────────────────────────┐
    │                         │
    │    ○       ○       ○    │  ← M3 motor mount holes (31mm bolt circle)
    │         ┌───┐           │
    │         │   │           │     10mm shaft clearance hole
    │         └───┘           │
    │    ○               ○    │
    │                         │
    │  ══════════════════════ │  ← Angle bracket bolted here
    └─────────────────────────┘

    Flat bar: 1.5" x 3" x 1/8" (38mm x 76mm x 3mm)

Side View:
                              ┌── Flat bar (motor mount)
                              │
    ════════════════════════════
                    │
                    │  ← Aluminum angle
                    │
    ────────────────┴───────────  ← PVC pipe

Assembly:
    1. Drill flat bar for motor (3x M3 on 31mm circle + 10mm center)
    2. Bolt angle to flat bar (2x M4)
    3. Clamp angle + flat bar assembly to PVC with printed clamp
```

### Drilling Template for Motor Mount

```
    All dimensions in mm from center

                    (0, 15.5)
                        ○  M3


           ┌────────────────────┐
           │                    │
    (-13.4, -7.75)        (13.4, -7.75)
         ○                    ○   M3
           │      (0,0)       │
           │        ●         │   10mm hole
           │                  │
           └────────────────────┘

    Hole positions (from center):
    - Center: 10mm drill
    - Top: (0, 15.5mm) - 3.4mm drill
    - Bottom-left: (-13.4, -7.75mm) - 3.4mm drill
    - Bottom-right: (13.4, -7.75mm) - 3.4mm drill
```

---

## Frame Assembly

### Step 1: Cut PVC

```
From 10' (120") stick of 1" Schedule 40:

Cut 1: 24" - Front rail
Cut 2: 24" - Rear rail
Cut 3: 22" - Cross brace 1
Cut 4: 22" - Cross brace 2
Cut 5: 10" - Side rail 1
Cut 6: 10" - Side rail 2
Cut 7: 10" - Side rail 3
Cut 8: 10" - Side rail 4

Total: 114" used, 6" waste
```

### Step 2: Dry Fit

```
    ┌──○──────────────────────────○──┐
    │                                │
    ○                                ○
    │                                │
    ├──○──────────────────────────○──┤  ← Cross brace
    │                                │
    ○                                ○
    │                                │
    └──○──────────────────────────○──┘

    1. Insert pipes into printed connectors
    2. Check squareness (measure diagonals)
    3. Mark alignment before gluing
```

### Step 3: Glue (Optional)

- Use PVC primer + cement for permanent joints
- Or leave dry-fit with bolt clamps for adjustability
- Recommendation: Glue corners, leave cross braces removable

### Step 4: Attach Motor Mounts

```
    Motor mount position (viewed from above):

    ┌─────────────────────────────────┐
    │  [M]                       [M]  │  ← Front motors, 2" from corners
    │                                 │
    │                                 │
    │                                 │
    │  [M]                       [M]  │  ← Rear motors, 2" from corners
    └─────────────────────────────────┘

    1. Slide angle bracket into motor clamp
    2. Clamp assembly to PVC side rail
    3. Bolt flat bar to angle bracket
    4. Mount motor to flat bar with M3 x 8mm
```

### Step 5: Add Cross Braces

```
    Cross braces provide:
    - Torsional rigidity
    - Mounting surface for electronics
    - Wire routing channels

    Position: 4" from front and rear
```

---

## Component Mounting

### NUC Mount

```
    Print a flat platform that clamps to both cross braces:

    ════════════════════════════  Cross brace 1
         │                  │
         │   ┌──────────┐   │
         │   │   NUC    │   │     Platform with rubber standoffs
         │   └──────────┘   │
         │                  │
    ════════════════════════════  Cross brace 2
```

### Sensor Mounts

Use pipe clamps (printed) that grip the PVC and provide mounting holes:

| Sensor | Mount Location |
|--------|----------------|
| RPLidar A1 | Front rail, center, elevated |
| RealSense D435 | Front rail, next to lidar |
| GPS | Rear rail, on printed mast |
| IMU | Center platform, with NUC |

### Electronics Tray

```
    Print a tray that sits on the cross braces:

    ┌─────────────────────────────┐
    │  [Arduino]  [MDD10A x2]     │
    │                             │
    │  [Power dist]  [Converters] │
    └─────────────────────────────┘
```

---

## Structural Analysis

### Load Path

```
    Weight distribution:

    NUC + electronics: ~1.5 kg (center)
    Battery: ~0.8 kg (center-rear)
    Motors: ~0.6 kg (corners)
    Sensors: ~0.3 kg (front)
    Frame: ~0.5 kg

    Total: ~3.7 kg + wheels/hubs ~1 kg = ~4.7 kg

    (Lighter than T-slot estimate of 8.3 kg)
```

### Stress Points

| Location | Load Type | Mitigation |
|----------|-----------|------------|
| Motor mounts | Torque reaction | Aluminum flat bar + angle |
| Corners | Torsion | 50%+ infill PETG connectors |
| Cross braces | Bending | Two braces distribute load |
| Sensor mounts | Vibration | Printed with damping |

### Deflection Estimate

1" Sch 40 PVC with 12" span, 2 kg center load:
- Deflection: ~1mm (better than 3/4" at ~2-3mm)
- Cross braces reduce this further

---

## Comparison

| Aspect | T-Slot ($100) | 1" PVC+Flat Bar ($40) |
|--------|---------------|------------------------|
| Rigidity | Excellent | Good (2.3x stiffer than 3/4") |
| Weight | ~2.5 kg | ~1.2 kg |
| Adjustability | Infinite | Limited |
| Precision | High | Medium |
| Build time | 1 hour | 2-3 hours |
| Durability | 10+ years | 3-5 years outdoor |
| Looks | Professional | DIY |

---

## STL Files Needed

| Part | Quantity | Notes |
|------|----------|-------|
| corner_connector.stl | 8 | 90° PVC joint |
| t_connector.stl | 4 | 3-way for cross braces |
| motor_mount_clamp.stl | 4 | Holds flat bar to PVC |
| pipe_clamp_small.stl | 4 | For sensors |
| pipe_clamp_large.stl | 2 | For NUC platform |
| electronics_tray.stl | 1 | Sits on cross braces |
| gps_mast.stl | 1 | 150mm elevated mount |

---

## Tools Required

- Hacksaw or PVC cutter
- Drill + bits (3.4mm, 5mm, 10mm)
- File or sandpaper
- Measuring tape
- Square (for alignment)
- Screwdriver / hex keys

---

## Upgrade Path

If 1" PVC proves too flexible:
1. Add diagonal bracing (printed or aluminum)
2. Replace side rails with aluminum tube
3. Switch to 1-1/4" PVC (5x stiffer than 3/4")
4. Graduate to full T-slot later

---

*Design created: 2024-12-28*
