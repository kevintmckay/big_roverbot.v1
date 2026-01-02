# Mid-Rover XL Shopping List

Maximum size LX-16A rover - 500mm × 350mm chassis, ~4.5kg target weight.

---

## TIER 1: ESSENTIAL (~$430)

### Drivetrain - $155

| Qty | Item | Price | URL |
|-----|------|-------|-----|
| 4 | Hiwonder LX-16A Serial Servo | $64 | https://www.amazon.com/dp/B073WR3SK9 |
| 4 | goBILDA 25T-to-6mm Shaft Coupler | $40 | https://www.gobilda.com/4001-series-clamping-servo-to-shaft-coupler-25-tooth-spline-to-6mm-round-bore/ |
| 1 | 6mm-to-12mm Hex Adapter (4-pack) | $10 | https://www.amazon.com/dp/B07J19PGNN |
| 1 | 1.9" RC Crawler Wheels 100mm (4-pack) | $25 | https://www.amazon.com/dp/B0B4ZX8QC9 |
| 4 | M4x8mm set screws (for hex adapters) | $6 | https://www.amazon.com/dp/B07L3S64YP |

**Wheel Specs (1.9" RC Crawler Style):**
- Diameter: 100mm (3.9")
- Width: 36-40mm (1.4-1.6") - **CHUNKY**
- Hub: 12mm hex (standard RC)
- Material: Soft rubber with **foam/sponge insert**
- Tread: Deep off-road lugs

**Adapter Chain:**
```
LX-16A Servo → goBILDA 25T-to-6mm → 6mm Hex Adapter → 12mm Hex Wheel
     ▼              ▼                    ▼                ▼
  [spline]      [coupler]           [brass hex]      [crawler tire]
```

### Frame - $60 (Aluminum + Printed)

| Qty | Item | Price | Source |
|-----|------|-------|--------|
| 2 | 1" Al square tube 500mm | $10 | Home Depot / metals supplier |
| 2 | 1" Al square tube 350mm | $7 | Home Depot / metals supplier |
| 4 | 3D printed corner brackets | $4 | ~100g PETG |
| 4 | 3D printed servo mounts | $4 | ~100g PETG |
| 1 | 3D printed electronics tray | $3 | ~80g PETG |
| 1 | 3D printed sensor mounts | $2 | ~60g PETG |
| 1 | M4/M5 hardware kit | $15 | https://www.amazon.com/dp/B08JCKH31Q |
| 1 | M3 hardware kit | $12 | https://www.amazon.com/dp/B08JCKH31Q |

**Frame cuts:** 2× 500mm + 2× 350mm = 1.7m total (one 6ft tube)

### Sensors - $99

| Qty | Item | Price | URL |
|-----|------|-------|-----|
| 1 | RPLidar A1 | $99 | https://www.amazon.com/dp/B07TJW5SXF |

### Control - $15

| Qty | Item | Price | URL |
|-----|------|-------|-----|
| 1 | Hiwonder Serial Servo Controller | $15 | https://www.amazon.com/dp/B0749BVPSQ |

*Note: Use already-owned Pi 5 as main compute*

### Power System - $65

| Qty | Item | Price | URL |
|-----|------|-------|-----|
| 1 | 2S LiPo 10000mAh | $45 | https://www.amazon.com/dp/B0B5B8F8QH |
| 1 | 5V 5A Buck Converter | $10 | https://www.amazon.com/dp/B09VY3WQ61 |
| 1 | XT60 Connectors (5 pairs) | $8 | https://www.amazon.com/dp/B07VRZR5TK |
| 1 | Inline Fuse Holder + 20A Fuse | $8 | https://www.amazon.com/dp/B07PZMHGVK |
| 1 | LiPo Voltage Alarm | $4 | https://www.amazon.com/dp/B08P72MKJ5 |

### Wiring - $25

| Qty | Item | Price | URL |
|-----|------|-------|-----|
| 1 | JST-XH 3-pin cables (servo ext) | $8 | https://www.amazon.com/dp/B01DUC1PFE |
| 1 | 14AWG Silicone Wire | $10 | https://www.amazon.com/dp/B01KCPL3GW |
| 1 | Power Distribution Block | $7 | https://www.amazon.com/dp/B08G8PPGLR |

### **TIER 1 SUBTOTAL: ~$430**

---

## TIER 2: ENHANCED (~$140)

| Qty | Item | Price | URL |
|-----|------|-------|-----|
| 1 | Intel RealSense D435 | $180 | https://www.intelrealsense.com/depth-camera-d435/ |
| 1 | Adafruit BNO055 IMU | $25 | https://www.adafruit.com/product/4646 |
| 1 | Powered USB 3.0 Hub | $20 | https://www.amazon.com/dp/B00JX1ZS5O |

*RealSense optional - RPLidar sufficient for 2D SLAM*

### **TIER 2 SUBTOTAL: ~$225**

---

## TIER 3: EXPANSION (~$150)

| Qty | Item | Price | URL |
|-----|------|-------|-----|
| 1 | 3-DOF Robot Arm Kit | $80 | https://www.amazon.com/dp/B07GWNPM5W |
| 1 | Pan-Tilt Camera Mount | $25 | https://www.amazon.com/dp/B07Q2RQYPF |
| 2 | SG90 Micro Servos | $8 | https://www.amazon.com/dp/B07MLR1498 |
| 1 | LED Light Bar 12V | $15 | https://www.amazon.com/dp/B09GXHCFXF |
| 1 | Speaker + Amp Module | $12 | https://www.amazon.com/dp/B0756GLQWL |

### **TIER 3 SUBTOTAL: ~$140**

---

## ALREADY OWNED

| Item | Value | Notes |
|------|-------|-------|
| Raspberry Pi 5 16GB | $90 | Main compute |
| Pi 5 Aluminum Case | $20 | Passive cooling |
| Transcend USB SSD | $30 | ROS2 storage |
| BNO055 IMU | $25 | Skip Tier 2 IMU |
| Various LiPos | - | Need 2S specifically |

---

## GRAND TOTAL

| Configuration | Cost |
|---------------|------|
| Tier 1 (Essential) | ~$430 |
| Tier 1 + Tier 2 | ~$655 |
| Tier 1 + 2 + 3 (Full) | ~$795 |
| Using owned Pi 5 + IMU | -$115 |
| **Practical build** | **~$540** |

---

## WEIGHT BREAKDOWN

| Component | Weight |
|-----------|--------|
| 4× LX-16A servos | 216g |
| 4× 100mm crawler wheels (foam filled) | 400g |
| 4× goBILDA couplers + hex adapters | 60g |
| Aluminum frame (1.7m tube) | 550g |
| 3D printed parts | 350g |
| 2S 10000mAh LiPo | 450g |
| Raspberry Pi 5 + case + SSD | 200g |
| RPLidar A1 | 170g |
| RealSense D435 | 72g |
| IMU + electronics | 100g |
| Wiring, hardware | 200g |
| **SUBTOTAL** | **~2.8 kg** |
| **Payload capacity** | **~1.7 kg** |
| **MAX TOTAL** | **~4.5 kg** |

---

## POWER BUDGET

| Component | Current @ 7.4V |
|-----------|----------------|
| 4× LX-16A (cruise) | 0.5A |
| 4× LX-16A (peak) | 5.0A |
| Pi 5 (via 5V buck) | 0.7A equiv |
| RPLidar A1 | 0.15A equiv |
| RealSense D435 | 0.25A equiv |
| IMU + misc | 0.05A equiv |
| **Total Cruise** | **~1.7A (12W)** |
| **Total Peak** | **~6.0A (44W)** |

**Runtime:** 10000mAh ÷ 1700mA = **~5.8 hours cruise**

Realistically with mixed use: **~4 hours**

---

## ORDER BY VENDOR

### Amazon - ~$350
```
Servos:
- https://www.amazon.com/dp/B073WR3SK9 - LX-16A ×4 = $64

Wheels/Adapters:
- https://www.amazon.com/dp/B0B4ZX8QC9 - 1.9" Crawler Wheels 100mm (4-pack) = $25
- https://www.amazon.com/dp/B07J19PGNN - 6mm-to-12mm Hex Adapter (4-pack) = $10
- https://www.amazon.com/dp/B07L3S64YP - Hex adapter set screws = $6

Lidar:
- https://www.amazon.com/dp/B07TJW5SXF - RPLidar A1 = $99

Power:
- https://www.amazon.com/dp/B0B5B8F8QH - 2S 10000mAh = $45
- https://www.amazon.com/dp/B09VY3WQ61 - 5V 5A Buck = $10
- https://www.amazon.com/dp/B07VRZR5TK - XT60 = $8
- https://www.amazon.com/dp/B07PZMHGVK - Fuse = $8
- https://www.amazon.com/dp/B08P72MKJ5 - LiPo Alarm = $4
- https://www.amazon.com/dp/B08G8PPGLR - Power Dist = $7

Wiring/Control:
- https://www.amazon.com/dp/B0749BVPSQ - Servo Controller = $15
- https://www.amazon.com/dp/B01DUC1PFE - Servo cables = $8
- https://www.amazon.com/dp/B01KCPL3GW - 14AWG Wire = $10

Hardware:
- https://www.amazon.com/dp/B08JCKH31Q - M3/M4/M5 kit = $27
```

### goBILDA - $40
```
https://www.gobilda.com/4001-series-clamping-servo-to-shaft-coupler-25-tooth-spline-to-6mm-round-bore/
- 4× 25T Spline to 6mm Shaft Coupler @ $9.99 = $39.96
```

### Home Depot / Metal Supplier - $20
```
- 1× 6ft 1" aluminum square tube (cut to 2×500mm + 2×350mm)
  ~$15-20
```

### 3D Printing - ~$15 filament
```
- Corner brackets ×4: 100g PETG
- Servo mounts ×4: 100g PETG
- Electronics tray: 80g PETG
- Sensor mounts: 60g PETG
Total: ~350g PETG @ $25/kg = ~$9 + time
```

---

## COMPARISON: Mid-Rover vs Mid-Rover XL

| Aspect | Mid-Rover | Mid-Rover XL | Difference |
|--------|-----------|--------------|------------|
| Size | 300×200mm | 500×350mm | +67% |
| Weight | 1.5 kg | 4.5 kg | +200% |
| Wheels | 85mm×33mm | 100mm×40mm crawler | +18% dia, +21% wide |
| Speed | 0.24 m/s | 0.28 m/s | +17% |
| Battery | 5000mAh | 10000mAh | +100% |
| Runtime | ~3 hr | ~4 hr | +33% |
| Cost | ~$320 | ~$540 | +69% |
| Payload | minimal | ~1.7 kg | +++ |

**XL is best for:** Adding robot arm, multiple sensors, extended missions, development platform.

**Standard is best for:** Quick build, budget-conscious, indoor-only, learning platform.

---

*Mid-Rover XL - Maximum practical LX-16A platform*
