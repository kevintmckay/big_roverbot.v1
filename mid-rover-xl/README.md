# Mid-Rover XL: Maximum Size LX-16A Platform

A larger rover that pushes the LX-16A servos to their practical limits.

## Design Philosophy

The standard mid-rover is conservatively sized with a 12:1 thrust-to-weight ratio. This XL variant maximizes the platform size while maintaining a **3:1 thrust-to-weight ratio** for reliable indoor/outdoor performance.

## Specifications

| Spec | Mid-Rover | Mid-Rover XL |
|------|-----------|--------------|
| Chassis | 300×200mm (12"×8") | **500×350mm (20"×14")** |
| Wheels | 85mm × 33mm rubber | **100mm × 40mm RC crawler** |
| Weight | 1.5 kg | **~4.5 kg** |
| Ground Clearance | 42mm | **50mm** |
| Max Speed | 0.24 m/s | **0.28 m/s** |
| Thrust:Weight | 12:1 | **3.5:1** |
| Frame | 3D printed PETG | **Aluminum tube + printed** |
| Battery | 2S 5000mAh | **2S 10000mAh** |
| Runtime | ~3 hours | **~4 hours** |

## Size Comparison

```
                    Crawlbot        Mid-Rover       Mid-Rover XL
                    305×170mm       300×200mm       500×350mm

                    ┌─────────┐     ┌──────────┐    ┌────────────────┐
                    │ ●     ● │     │ ●      ● │    │ ●            ● │
                    │         │     │          │    │                │
                    │         │     │          │    │                │
                    │ ●     ● │     │ ●      ● │    │                │
                    └─────────┘     └──────────┘    │                │
                                                    │ ●            ● │
                     ~1.2 kg         ~1.5 kg        └────────────────┘

                                                         ~4.5 kg
```

## Torque Budget

**LX-16A with 100mm wheels:**
- Torque: 20 kg.cm = 1.96 Nm per servo
- Wheel radius: 50mm = 0.05m
- Force per wheel: 1.96 ÷ 0.05 = **39.2 N**
- Total thrust (4 wheels): **157 N**
- Target weight: 4.5 kg = 44 N
- **Thrust-to-weight: 3.5:1** ✓

### Performance Envelope

| Scenario | Required Ratio | Max Weight | Status |
|----------|---------------|------------|--------|
| Flat indoor | 2:1 | 8.0 kg | ✓ |
| Carpet/outdoor | 3:1 | 5.3 kg | ✓ |
| 15° incline | 4:1 | 4.0 kg | marginal |
| 20° incline | 5:1 | 3.2 kg | ✗ |

**Recommended max weight: 4.5-5.0 kg** for versatile indoor/outdoor use.

## Weight Budget

| Component | Weight |
|-----------|--------|
| 4x LX-16A servos | 216g |
| 4x 100mm RC crawler wheels (foam) | 400g |
| 4x goBILDA couplers + hex adapters | 60g |
| Aluminum tube frame | 550g |
| 3D printed brackets/mounts | 350g |
| 2S 10000mAh LiPo | 450g |
| Raspberry Pi 5 + case | 150g |
| RPLidar A1 | 170g |
| RealSense D435 (optional) | 72g |
| BNO055 IMU | 10g |
| Electronics (buck, wiring) | 150g |
| Payload margin | ~1.2 kg |
| **TOTAL** | **~2.8 kg base** |

Leaves **~1.7 kg payload capacity** for accessories (robot arm, gripper, etc.)

## Frame Design

Multiple options available (see [FRAME_OPTIONS.md](FRAME_OPTIONS.md)):

| Option | Weight | Cost | Best For |
|--------|--------|------|----------|
| A: Aluminum Tube + Printed | 550g | $20 | Strongest, recommended |
| B: Hybrid (Printed + Rod) | 450g | $15 | Lightest, all prints fit MK4S |
| C: Full Print (Halves) | 600g | $15 | Large bed printers only |
| D: Full Print (Panels) | 800g | $20 | Enclosed electronics bay |

### Default: Aluminum Tube + Printed Brackets

```
        500mm (20")
    ┌─────────────────────────────────────────────┐
    │                                             │
    │   ┌─────┐                         ┌─────┐   │
    │   │ S1  │                         │ S2  │   │
    │   └──┬──┘                         └──┬──┘   │
    │      ○ 100mm                    100mm ○     │
    │                                             │
    │   ═══════════════════════════════════════   │  ← 1" Al tube  350mm
    │                                             │    (25mm sq)   (14")
    │            ┌─────────────────┐              │
    │            │   Electronics   │              │
    │            │      Bay        │              │
    │            └─────────────────┘              │
    │                                             │
    │   ═══════════════════════════════════════   │  ← 1" Al tube
    │                                             │
    │      ○ 100mm                    100mm ○     │
    │   ┌──┴──┐                         ┌──┴──┐   │
    │   │ S3  │                         │ S4  │   │
    │   └─────┘                         └─────┘   │
    │                                             │
    └─────────────────────────────────────────────┘
```

**Frame materials:**
- 2x 500mm (20") aluminum square tube 25mm (1")
- 2x 350mm (14") aluminum square tube 25mm (1")
- 4x 3D printed corner brackets
- 4x 3D printed servo mounts
- 1x 3D printed electronics tray

## Sensor Configuration

```
                    FRONT
         ┌─────────────────────────┐
         │      [RPLidar A1]       │  ← Elevated on 50mm standoff
         │           ○             │
         │                         │
         │    [RealSense D435]     │  ← Front-facing depth
         │         ═══             │
         │                         │
         │  ┌─────────────────┐    │
         │  │   [Pi 5]        │    │
         │  │   [IMU]         │    │
         │  │   [Power dist]  │    │
         │  └─────────────────┘    │
         │                         │
         │      [Battery Bay]      │
         │     ┌───────────┐       │
         │     │  10Ah 2S  │       │
         │     └───────────┘       │
         └─────────────────────────┘
                    REAR
```

## Power Architecture

```
2S LiPo 10000mAh (7.4V, 74Wh)
         │
         ├──[20A Fuse]
         │
         ├── LX-16A Bus (direct 7.4V) ─── 4A peak
         │
         └── 5V 5A Buck ─────────────┬── Pi 5 (3A)
                                     ├── RPLidar (0.5A)
                                     ├── RealSense (0.8A)
                                     └── IMU, misc (0.2A)

Peak draw: ~35W
Cruise draw: ~18W
Runtime: 74Wh ÷ 18W = ~4 hours
```

## Expansion Options

With ~1 kg payload capacity, consider:

| Add-on | Weight | Notes |
|--------|--------|-------|
| 3-DOF robot arm | 400-800g | Pick and place |
| Pan-tilt camera | 150g | Surveillance |
| Speaker + mic array | 100g | Voice interaction |
| LED light bar | 50g | Night operation |
| Additional sensors | 200g | Ultrasonic, gas, etc. |

## Documents

- [SHOPPING_LIST.md](SHOPPING_LIST.md) - Complete parts list
- [FRAME_OPTIONS.md](FRAME_OPTIONS.md) - Frame build options (aluminum, hybrid, full print)

## Comparison: All Rover Variants

| Spec | Crawlbot | Mid-Rover | Mid-Rover XL | RoverBot v1 |
|------|----------|-----------|--------------|-------------|
| Size (L×W) | 305×170mm | 300×200mm | 500×350mm | 610×305mm |
| Weight | 1.2 kg | 1.5 kg | 4.5 kg | 8.3 kg |
| Wheels | 95mm | 85mm | 100mm | 192mm |
| Motors | Brushed DC | LX-16A | LX-16A | Pololu 37D |
| Voltage | 7.4V | 7.4V | 7.4V | 12V |
| Cost | ~$150* | ~$320 | ~$500 | ~$1000 |
| Terrain | All-terrain | Light outdoor | Light outdoor | All-terrain |

*Crawlbot uses converted RC chassis

---

*Mid-Rover XL - Maximum practical size for LX-16A servo platform*
