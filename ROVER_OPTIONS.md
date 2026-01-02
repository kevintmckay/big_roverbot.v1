# Rover Platform Comparison

A complete comparison of all rover options.

---

## Quick Summary

| Rover | Size | Weight | Wheels | Cost | Best For |
|-------|------|--------|--------|------|----------|
| Crawlbot v1 | 305×170mm | 1.2 kg | 95mm | ~$150 | All-terrain, existing |
| Mid-Rover | 300×200mm | 1.5 kg | 85mm | ~$320 | Budget, indoor |
| Mid-Rover XL | 500×350mm | 4.5 kg | 100mm | ~$540 | Payload, dev platform |
| Big RoverBot v1 | 610×305mm | 8.3 kg | 192mm | ~$1000 | Outdoor, professional |

---

## Visual Scale Comparison

```
                        ← 610mm (24") →
    ┌─────────────────────────────────────────────────────┐
    │ ●                                                 ● │
    │                  Big RoverBot v1                    │  305mm
    │                    8.3 kg                           │  (12")
    │ ●                                                 ● │
    └─────────────────────────────────────────────────────┘
                           192mm wheels

              ← 500mm (20") →
    ┌───────────────────────────────────────┐
    │ ●                                   ● │
    │            Mid-Rover XL               │  350mm
    │               4.5 kg                  │  (14")
    │ ●                                   ● │
    └───────────────────────────────────────┘
                   100mm crawler wheels

        ← 305mm (12") →        ← 300mm (12") →
    ┌─────────────────┐    ┌───────────────────┐
    │ ●             ● │    │ ●               ● │
    │   Crawlbot v1   │    │    Mid-Rover      │  200mm
    │     1.2 kg      │    │      1.5 kg       │  (8")
    │ ●             ● │    │ ●               ● │
    └─────────────────┘    └───────────────────┘
       170mm wide              200mm wide
       95mm wheels             85mm wheels
```

---

## Coke Can Scale Reference

Standard 12oz Coca-Cola can: **122mm tall × 66mm diameter**

| Rover | Length (Coke cans) | Wheel vs Can |
|-------|-------------------|--------------|
| Crawlbot v1 | 2.5× | 78% height |
| Mid-Rover | 2.5× | 70% height |
| Mid-Rover XL | 4× | 82% height |
| Big RoverBot v1 | 5× | 157% height |

---

## Crawlbot v1

**Status:** Existing build (RC crawler conversion)

| Spec | Value |
|------|-------|
| Chassis | 305×170mm (12"×6.7") |
| Wheels | 95mm × 35mm |
| Weight | ~1.2 kg |
| Motors | Brushed DC (stock RC) |
| Voltage | 7.4V (2S) |
| Sensors | OAK-D Lite, ToF, IMU |
| Cost | ~$150 |

**Pros:** Already built, true all-terrain, compact
**Cons:** No encoders, fixed platform

**Location:** `/home/kevin/robots/crawlbot.v1/`

---

## Mid-Rover

**Status:** Designed, not built

| Spec | Value |
|------|-------|
| Chassis | 300×200mm (12"×8") |
| Wheels | 85mm × 33mm thick rubber |
| Weight | ~1.5 kg |
| Motors | 4× LX-16A servos |
| Voltage | 7.4V (2S) |
| Speed | 0.24 m/s (0.5 mph) |
| Frame | 3D printed PETG |
| Cost | ~$320 |

**Drivetrain:**
```
LX-16A → goBILDA 25T-to-6mm → 85mm Wheel
```

**Pros:** Cheapest, simple wiring, lightweight
**Cons:** Slow, no encoder odometry, needs 2S battery

**Location:** `/home/kevin/robots/big_roverbot.v1/mid-rover/`

---

## Mid-Rover XL

**Status:** Designed, not built

| Spec | Value |
|------|-------|
| Chassis | 500×350mm (20"×14") |
| Wheels | 100mm × 40mm RC crawler |
| Weight | ~4.5 kg (1.7 kg payload) |
| Motors | 4× LX-16A servos |
| Voltage | 7.4V (2S) |
| Speed | 0.28 m/s (0.6 mph) |
| Frame | See options below |
| Cost | ~$540 |

**Drivetrain:**
```
LX-16A → goBILDA 25T-to-6mm → 6mm-to-12mm Hex → 100mm Crawler Wheel
```

**Frame Options:** (see [mid-rover-xl/FRAME_OPTIONS.md](mid-rover-xl/FRAME_OPTIONS.md))

| Option | Weight | Cost | Notes |
|--------|--------|------|-------|
| A: Aluminum tube + printed | 550g | $20 | Strongest, recommended |
| B: Hybrid (printed + rod) | 450g | $15 | Lightest, fits MK4S |
| C: Full print halves | 600g | $15 | Large bed only |
| D: Full print panels | 800g | $20 | Enclosed bay |

**Pros:** Payload capacity, chunky wheels, expandable
**Cons:** Near torque limits, slow, complex adapters

**Location:** `/home/kevin/robots/big_roverbot.v1/mid-rover-xl/`

---

## Big RoverBot v1

**Status:** Designed, not built

| Spec | Value |
|------|-------|
| Chassis | 610×305mm (24"×12") |
| Wheels | 192mm × 72mm Wasteland |
| Weight | ~8.3 kg |
| Motors | 4× Pololu 37D 131:1 |
| Voltage | 12V (from 4S) |
| Speed | 0.67 m/s (1.5 mph) |
| Frame | Al T-slot or Al tube |
| Cost | ~$1000 |

**Drivetrain:**
```
Pololu 37D → L-Bracket → goBILDA Sonic Hub → 192mm Wasteland Wheel
```

**Frame Options:** (see main [CHASSIS_BUILD.md](CHASSIS_BUILD.md))

| Option | Weight | Cost |
|--------|--------|------|
| T-Slot Aluminum | 2.5 kg | $100 |
| Aluminum Square Tube | 0.8 kg | $25 |

**Pros:** Fast, true all-terrain, encoders, uses 4S batteries
**Cons:** Most expensive, heaviest

**Location:** `/home/kevin/robots/big_roverbot.v1/`

---

## Motor Comparison

| Motor | Torque | Speed | Voltage | Feedback | Price ×4 |
|-------|--------|-------|---------|----------|----------|
| LX-16A | 20 kg.cm | 53 RPM | 6-8.4V | Temp/position | $64 |
| Pololu 37D 131:1 | 18 kg.cm | 76 RPM | 12V | 64 CPR encoder | $284 |

---

## Wheel Comparison

| Rover | Wheel | Diameter | Width | Style |
|-------|-------|----------|-------|-------|
| Crawlbot | Stock RC | 95mm | 35mm | Crawler |
| Mid-Rover | Rubber+sponge | 85mm | 33mm | Thick rubber |
| Mid-Rover XL | RC Crawler | 100mm | 40mm | Deep lug, foam |
| Big RoverBot v1 | Wasteland | 192mm | 72mm | Off-road |

---

## Power Comparison

| Rover | Battery | Capacity | Runtime |
|-------|---------|----------|---------|
| Crawlbot | 2S | 2200mAh | ~1 hr |
| Mid-Rover | 2S | 5000mAh | ~3 hr |
| Mid-Rover XL | 2S | 10000mAh | ~4 hr |
| Big RoverBot v1 | 4S→12V | 8000mAh | ~2 hr |

---

## Decision Guide

```
                        ┌─────────────────┐
                        │ Need all-terrain │
                        │ outdoor capability?│
                        └────────┬────────┘
                                 │
                    ┌────────────┴────────────┐
                    │                         │
                   YES                        NO
                    │                         │
                    ▼                         ▼
            ┌───────────────┐         ┌───────────────┐
            │ Budget > $500? │         │ Need payload  │
            └───────┬───────┘         │ capacity?     │
                    │                 └───────┬───────┘
           ┌────────┴────────┐                │
           │                 │       ┌────────┴────────┐
          YES               NO       │                 │
           │                 │      YES               NO
           ▼                 ▼       │                 │
     ┌──────────┐     ┌──────────┐   ▼                 ▼
     │   Big    │     │ Crawlbot │  ┌──────────┐  ┌──────────┐
     │    v1    │     │    v1    │  │ Mid-Rover│  │ Mid-Rover│
     │  $1000   │     │  $150    │  │    XL    │  │  $320    │
     └──────────┘     │ (exists) │  │   $540   │  └──────────┘
                      └──────────┘  └──────────┘
```

---

## Cost Breakdown

```
$0        $250       $500       $750       $1000
|----------|----------|----------|----------|
Crawlbot   ████ $150
Mid-Rover  ██████████ $320
Mid-Rover XL ████████████████ $540
Big RoverBot v1 █████████████████████████████████ $1000
```

---

## Files Reference

```
/home/kevin/robots/
├── crawlbot.v1/
│   └── ros2_ws/src/.../crawlbot.urdf.xacro
│
└── big_roverbot.v1/
    ├── README.md
    ├── SHOPPING_LIST.md
    ├── CHASSIS_BUILD.md
    ├── ROVER_OPTIONS.md        ← This file
    │
    ├── mid-rover/
    │   ├── README.md
    │   ├── SHOPPING_LIST.md
    │   └── CHASSIS_BUILD.md
    │
    └── mid-rover-xl/
        ├── README.md
        ├── SHOPPING_LIST.md
        └── FRAME_OPTIONS.md
```

---

*Last updated: 2025-01*
