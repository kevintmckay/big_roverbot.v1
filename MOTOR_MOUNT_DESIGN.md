# Motor Mount Plate Design

**UPDATE: Using Pololu L-Brackets instead of custom plates.**
- Order: 2x Pololu Stamped L-Bracket pairs ($11.95 each) = $23.90
- Link: https://www.pololu.com/product/1084
- Modification: Drill 2 holes per bracket to 5.5mm for M5 T-slot mounting

---

## Original Custom Plate Design (for reference)

Custom plates for mounting Pololu 37D motors to OpenBuilds 20x20 V-Slot extrusion.

## Specifications

### Pololu 37D Motor Mounting

| Dimension | Value |
|-----------|-------|
| Motor diameter | 37mm |
| Mounting holes | 6x M3 threaded (use 3) |
| Bolt circle diameter | 31mm (15.5mm radius) |
| Hole spacing (adjacent) | 15.5mm |
| Max screw depth | 3mm |
| Shaft | 6mm D-shaft |

### Plate Design

| Parameter | Value |
|-----------|-------|
| Material | 3mm Aluminum 6061 |
| Dimensions | 60mm x 70mm |
| Quantity | 4 plates |
| Finish | As-cut (deburred) |

---

## Drawing

### Top View (dimensions in mm)
```
                    60mm
    ←───────────────────────────────→

    ┌─────────────────────────────────┐  ↑
    │                                 │  │
    │   ┌───┐               ┌───┐     │  │
    │   │ S │               │ S │     │  │  S = M5 slot
    │   │ L │               │ L │     │  │      5.5mm x 15mm
    │   │ O │               │ O │     │  │
    │   │ T │               │ T │     │  │
    │   └───┘               └───┘     │  │
    │                                 │  │
    │         ○           ○           │  │  70mm
    │              ╭───╮              │  │
    │              │   │              │  │  Center = 10mm hole
    │              ╰───╯              │  │  (shaft clearance)
    │         ○                       │  │
    │                                 │  │  ○ = M3 clearance hole
    │                                 │  │      3.4mm diameter
    │                                 │  │
    └─────────────────────────────────┘  ↓

    ←──12──→     ←──36──→     ←──12──→
```

---

## Hole Coordinates

All coordinates from plate center (30mm, 35mm from bottom-left corner).

### Motor Mounting Holes (M3 clearance, 3.4mm dia)

Using 3 of 6 motor holes on 31mm bolt circle (15.5mm radius):

| Hole | Angle | X (from center) | Y (from center) |
|------|-------|-----------------|-----------------|
| M1 | 90° | 0.00 | +15.50 |
| M2 | 210° | -13.43 | -7.75 |
| M3 | 330° | +13.43 | -7.75 |

**Absolute coordinates (from bottom-left):**

| Hole | X | Y | Diameter |
|------|---|---|----------|
| M1 | 30.00 | 50.50 | 3.4mm |
| M2 | 16.57 | 27.25 | 3.4mm |
| M3 | 43.43 | 27.25 | 3.4mm |

### Center Shaft Hole

| Hole | X | Y | Diameter |
|------|---|---|----------|
| Center | 30.00 | 35.00 | 10.0mm |

### T-Slot Mounting Slots (M5 clearance)

Slots allow vertical adjustment for belt/chain tension or alignment.

| Slot | X center | Y start | Y end | Width | Length |
|------|----------|---------|-------|-------|--------|
| Left | 12.00 | 50.00 | 65.00 | 5.5mm | 15mm |
| Right | 48.00 | 50.00 | 65.00 | 5.5mm | 15mm |

**Slot coordinates (rounded rectangle or stadium shape):**
- Left slot: center (12, 57.5), 5.5mm wide x 15mm tall
- Right slot: center (48, 57.5), 5.5mm wide x 15mm tall

---

## Side View (mounted)

```
        V-Slot 20x20
    ┌─────────────────┐
    │    ┌─┐   ┌─┐    │  ← T-nuts + M5 screws
    │    │ │   │ │    │
    └────┴─┴───┴─┴────┘
         │       │
    ┌────┴───────┴────┐  ← Motor mount plate (3mm)
    │   ┌─────────┐   │
    │   │  MOTOR  │   │  ← Pololu 37D (37mm dia)
    │   │   37D   │   │
    │   │         │   │
    │   └────○────┘   │  ← 6mm D-shaft
    │        │        │
    └────────┼────────┘
             │
         ┌───┴───┐
         │ WHEEL │       ← Wasteland 192mm via hub
         └───────┘
```

---

## Hardware Required (per motor)

| Item | Qty | Notes |
|------|-----|-------|
| M3 x 6mm socket head cap screw | 3 | Motor mounting (max 3mm into motor) |
| M5 x 10mm low profile screw | 2 | T-slot mounting |
| M5 drop-in T-nut | 2 | For V-slot |

**Total for 4 motors:**
- 12x M3 x 6mm screws
- 8x M5 x 10mm screws
- 8x M5 T-nuts

---

## SendCutSend Order

### File Format
Upload DXF file with:
- Outer profile: 60mm x 70mm rectangle with 3mm corner radius
- 3x 3.4mm holes (motor mounting)
- 1x 10mm hole (shaft clearance)
- 2x 5.5mm x 15mm slots (T-slot mounting)

### Order Details
```
Material:     6061 Aluminum
Thickness:    3mm (0.118")
Quantity:     4
Finish:       None (as-cut, deburred)
Tolerance:    Standard (±0.005")
```

### Estimated Cost
~$8-12 per plate = $32-48 total

---

## DXF Coordinates (for CAD)

### Outer Rectangle
```
(0, 0) - (60, 0) - (60, 70) - (0, 70) - (0, 0)
Corner radius: 3mm
```

### Circles
```
Center hole:  center(30, 35), radius 5.0mm
Motor hole 1: center(30.00, 50.50), radius 1.7mm
Motor hole 2: center(16.57, 27.25), radius 1.7mm
Motor hole 3: center(43.43, 27.25), radius 1.7mm
```

### Slots (stadium/rounded rectangle)
```
Left slot:  center(12, 57.5), width 5.5mm, height 15mm, end radius 2.75mm
Right slot: center(48, 57.5), width 5.5mm, height 15mm, end radius 2.75mm
```

---

## Assembly Notes

1. **Motor Orientation**: Mount motor with shaft pointing outward from frame
2. **Slot Adjustment**: Slots allow ~10mm vertical adjustment for alignment
3. **Screw Depth**: Do not exceed 3mm into motor mounting holes (hits gears)
4. **Shaft Clearance**: 10mm hole provides clearance for 6mm shaft + hub

---

## Alternative: Buy Pre-made Brackets

If custom plates are too complex, Pololu sells brackets:

| Option | Price | Link |
|--------|-------|------|
| Stamped L-Bracket (pair) | $7.95 | https://www.pololu.com/product/1084 |
| Machined Bracket (single) | $12.95 | https://www.pololu.com/product/1995 |

**Note:** Pre-made brackets would need additional adapters to mount to T-slot.

---

*Design created: 2024-12-27*
