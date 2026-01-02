# Mid-Rover Shopping List

A lightweight rover built around LX-16A serial bus servos.

---

## TIER 1: ESSENTIAL (~$320)

### Drivetrain - $120

| Qty | Item | Price | URL |
|-----|------|-------|-----|
| 4 | Hiwonder LX-16A Serial Servo | $64 | https://www.amazon.com/dp/B073WR3SK9 |
| 4 | goBILDA 25T-to-6mm Shaft Coupler | $40 | https://www.gobilda.com/4001-series-clamping-servo-to-shaft-coupler-25-tooth-spline-to-6mm-round-bore/ |
| 4 | 85mm Rubber Wheel w/ 6mm Coupling | $16 | https://www.amazon.com/dp/B0DHZFMGZ5 |

**Wheel Specs:**
- Diameter: 85mm (3.3")
- Width: 33mm (1.3") - thick rubber with sponge liner
- Hub: Metal coupling, 6mm bore
- Tread: High-grip rubber, excellent for indoor/outdoor

**Assembly:** Servo → goBILDA coupler → 6mm stub shaft → Wheel hub

### Frame - $25 (3D Printed)

| Qty | Item | Price | Source |
|-----|------|-------|--------|
| 1 | Main chassis plate | ~$5 | 3D print (~150g PETG) |
| 4 | Servo mount brackets | ~$3 | 3D print (~80g PETG) |
| 1 | Electronics tray | ~$2 | 3D print (~60g PETG) |
| 1 | Sensor mounts | ~$2 | 3D print (~50g PETG) |
| 1 | M3 hardware kit (screws, nuts, standoffs) | $12 | https://www.amazon.com/dp/B08JCKH31Q |

**Frame Dimensions:** 300mm x 200mm (12" x 8")

### Sensors - $50

| Qty | Item | Price | URL |
|-----|------|-------|-----|
| 1 | LD19 Lidar (budget option) | $50 | https://www.amazon.com/dp/B0B1XDNPN5 |

*Alternative: RPLidar A1 ($99) for better range/accuracy*

### Control - $30

| Qty | Item | Price | URL |
|-----|------|-------|-----|
| 1 | Hiwonder Serial Servo Controller | $15 | https://www.amazon.com/dp/B0749BVPSQ |
| 1 | Raspberry Pi Zero 2W | $15 | https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/ |

*Alternative: Use Pi 5 (already owned) and skip the servo controller - connect servos directly to Pi UART*

### Power System - $45

| Qty | Item | Price | URL |
|-----|------|-------|-----|
| 1 | 2S LiPo 5000mAh | $25 | https://www.amazon.com/dp/B08QCXK4KV |
| 1 | 5V 3A Buck Converter | $8 | https://www.amazon.com/dp/B01MQGMOKI |
| 1 | XT60 Connectors (5 pairs) | $8 | https://www.amazon.com/dp/B07VRZR5TK |
| 1 | LiPo Voltage Alarm | $4 | https://www.amazon.com/dp/B08P72MKJ5 |

### Wiring & Misc - $20

| Qty | Item | Price | URL |
|-----|------|-------|-----|
| 1 | JST-XH 3-pin cables (servo extensions) | $8 | https://www.amazon.com/dp/B01DUC1PFE |
| 1 | MicroSD Card 32GB | $8 | https://www.amazon.com/dp/B07FCMBLV6 |
| 1 | USB OTG Hub for Pi Zero | $4 | https://www.amazon.com/dp/B01JL837X8 |

### **TIER 1 SUBTOTAL: ~$320**

---

## TIER 2: ENHANCED (~$70)

| Qty | Item | Price | URL |
|-----|------|-------|-----|
| 1 | RPLidar A1 (upgrade from LD19) | +$49 | https://www.amazon.com/dp/B07TJW5SXF |
| 1 | MPU6050 IMU (cheap but functional) | $6 | https://www.amazon.com/dp/B00LP25V1A |
| 2 | HC-SR04 Ultrasonic Sensors | $6 | https://www.amazon.com/dp/B07RGB4W8V |
| 1 | Pi Camera Module v3 | $25 | https://www.raspberrypi.com/products/camera-module-3/ |

### **TIER 2 SUBTOTAL: ~$70** (net +$35 if upgrading lidar)

---

## ALREADY OWNED (Potential Use)

| Item | Notes |
|------|-------|
| Raspberry Pi 5 16GB | Overkill but works - skip Pi Zero if using this |
| BNO055 IMU | Better than MPU6050 |
| GPS module | Not needed for indoor use |
| 4S LiPo batteries | **Not compatible** - need 2S for LX-16A |

---

## GRAND TOTAL

| Configuration | Cost |
|---------------|------|
| Tier 1 (Pi Zero, LD19) | ~$320 |
| Tier 1 + Pi 5 instead | ~$305 (skip Pi Zero) |
| Tier 1 + Tier 2 | ~$390 |
| Using Pi 5 + BNO055 owned | ~$280 |

---

## WEIGHT BUDGET

| Component | Weight |
|-----------|--------|
| 4x LX-16A servos | 216g |
| 4x 85mm rubber wheels | ~200g |
| 4x goBILDA couplers | 24g |
| 3D printed frame | ~350g |
| 2S 5000mAh LiPo | ~270g |
| Raspberry Pi + electronics | ~150g |
| Lidar | ~170g |
| Wiring, misc | ~100g |
| **TOTAL** | **~1.5 kg** |

Well under the 2.5kg design target - excellent thrust margin.

---

## POWER BUDGET

| Component | Current @ 7.4V |
|-----------|----------------|
| 4x LX-16A (cruise) | 0.4A |
| 4x LX-16A (peak) | 4.0A |
| Raspberry Pi (via 5V) | 0.5A equiv |
| Lidar | 0.3A equiv |
| **Total Cruise** | **~1.5A** |
| **Total Peak** | **~5A** |

5000mAh 2S battery runtime: 5000mAh / 1500mA = **~3.3 hours cruise**

---

## SERVO BUS WIRING

All 4 servos daisy-chain on a single 3-wire bus:

```
Power (7.4V) ────┬────┬────┬────┐
                 │    │    │    │
GND ─────────────┼────┼────┼────┤
                 │    │    │    │
Signal ──────────┼────┼────┼────┤
                 │    │    │    │
              [S1]  [S2]  [S3]  [S4]
               FL    FR    RL    RR
```

Each servo has unique ID (set via Hiwonder debug tool):
- ID 1: Front Left
- ID 2: Front Right
- ID 3: Rear Left
- ID 4: Rear Right

---

## ORDER BY VENDOR

### Amazon - ~$211
```
Servos:
- https://www.amazon.com/dp/B073WR3SK9 - LX-16A x4 = $64

Wheels:
- https://www.amazon.com/dp/B0DHZFMGZ5 - 85mm Rubber Wheel 6mm x4 = $16
  (Select 6MM aperture option)

Lidar:
- https://www.amazon.com/dp/B0B1XDNPN5 - LD19 Lidar = $50

Power:
- https://www.amazon.com/dp/B08QCXK4KV - 2S 5000mAh LiPo = $25
- https://www.amazon.com/dp/B01MQGMOKI - 5V Buck = $8
- https://www.amazon.com/dp/B07VRZR5TK - XT60 = $8
- https://www.amazon.com/dp/B08P72MKJ5 - LiPo Alarm = $4

Wiring/Control:
- https://www.amazon.com/dp/B0749BVPSQ - Servo Controller = $15
- https://www.amazon.com/dp/B01DUC1PFE - Servo cables = $8
- https://www.amazon.com/dp/B07FCMBLV6 - MicroSD = $8
- https://www.amazon.com/dp/B01JL837X8 - USB OTG = $4

Hardware:
- https://www.amazon.com/dp/B08JCKH31Q - M3 kit = $12
```

### goBILDA - $40
```
https://www.gobilda.com/4001-series-clamping-servo-to-shaft-coupler-25-tooth-spline-to-6mm-round-bore/
- 4x 25T Spline to 6mm Shaft Coupler
  @ $9.99 each = $39.96
```

### Raspberry Pi - $15
```
https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/
- 1x Raspberry Pi Zero 2W = $15
```

---

## COMPARISON: Mid-Rover vs RoverBot v1

| Aspect | Mid-Rover | RoverBot v1 | Savings |
|--------|-----------|-------------|---------|
| Motors | $64 | $284 | $220 |
| Motor drivers | $0 | $30 | $30 |
| Wheels + couplers | $56 | $192 | $136 |
| Frame | $25 | $100 | $75 |
| Compute | $15 | $0 (owned) | - |
| **Total** | **~$320** | **~$1000** | **~$680** |

The Mid-Rover costs roughly **1/3 the price** of RoverBot v1.

---

*Generated for Mid-Rover - LX-16A based platform*
