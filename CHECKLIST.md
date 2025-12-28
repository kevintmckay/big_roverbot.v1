# RoverBot v1 Shopping Checklist

## Already Owned

- [x] Intel NUC (Celeron N5105, 32GB RAM, 2TB NVMe)
- [x] Zippy Flightmax 4S LiPo (8000mAh, 14.8V) - main battery
- [x] Turnigy nano-tech 4S LiPo (1800mAh, 14.8V) - backup
- [x] TN GPS with Compass - primary GPS
- [x] FTDI USB-UART Adapter (FT232RL) - for GPS connection
- [x] SparkFun FTDI Basic 5V - spare USB-UART
- [x] Pololu 5V Regulator (D37034) - small 5V supply (~500mA)

---

## To Buy - Tier 1 (Essential) ~$912

### Drivetrain (~$506) - OPTION B SELECTED

- [ ] Pololu 37D 131:1 motor 12V 76RPM w/encoder (x4) - ~$284
  - Link: https://www.pololu.com/product/4756
  - Torque: 45 kg-cm, Speed: 76 RPM
- [ ] 7.6" Wasteland Wheels 192mm (x4) - ~$160
  - Link: https://www.servocity.com/192mm-wasteland-wheel/
  - 2.8" wide, airless rubber tread
- [ ] Cytron MDD10A dual motor driver (x2) - ~$30
  - Link: https://www.amazon.com/dp/B01N0XHZZ0
- [ ] goBILDA 1309 Series Sonic Hub 6mm D-Bore (x4) - ~$32
  - Link: https://www.gobilda.com/1309-series-sonic-hub-6mm-d-bore/
  - Connects Pololu D-shaft to Wasteland wheel (16mm pattern)
  - ⚠️ VERIFY: Confirm 192mm Wasteland uses 16mm goBILDA pattern before ordering!

**Specs: 1.5 mph top speed, ~4" ground clearance, best torque for hills**

### Frame (~$100)

- [ ] T-slot aluminum extrusion 1" (~4m total) - ~$60
  - Link: https://openbuildspartstore.com/
- [ ] Brackets, corner cubes, T-nuts, screws - ~$40
  - Link: https://openbuildspartstore.com/

### Sensors (~$99)

- [ ] RPLidar A1 (12m, 5.5Hz) - ~$99
  - Link: https://www.amazon.com/dp/B07TJW5SXF

### Power (~$50)

- [ ] DC-DC Boost Converter 19V 5A (for NUC) - ~$25
  - Search: "DC-DC boost converter 12V to 19V 5A"
  - NOTE: Must be BOOST (step-up), 4S LiPo is only 14.8V!
- [ ] DC-DC Step-down 12V 10A (for motors) - ~$15
  - Search: "DC-DC 12V 10A buck converter"
  - Or run motors direct from 4S battery (14.8V OK for 12V motors)
- [ ] DC-DC Step-down 5V 3A (for sensors) - ~$8
  - For Arduino, encoders, small sensors

### Motor Control Interface (~$25)

- [ ] Arduino Mega 2560 or Teensy 4.1 - ~$25
  - Link: https://store.arduino.cc/products/arduino-mega-2560-rev3
  - NUC has NO GPIO - need microcontroller for PWM/encoder
  - Communicates with NUC via USB serial
- [x] ~~USB-UART adapter (for GPS)~~ - **Already owned** (FTDI FT232RL)

### Frame Hardware (~$50)

- [ ] Motor mount plates (custom cut) - ~$40
  - Link: https://sendcutsend.com/
  - 4x plates to mount Pololu 37D to T-slot frame
  - Need to design for 37mm motor diameter
- [ ] NUC mounting standoffs (rubber, vibration isolating) - ~$10
  - Search: "M3 rubber standoffs vibration"

### Safety (~$25)

- [ ] Inline fuse holder + 20A fuse - ~$8
  - Between battery and power distribution
- [ ] Emergency stop switch (mushroom button) - ~$10
  - Cuts motor power, easy to reach
- [ ] LiPo battery alarm/monitor - ~$7
  - Prevents over-discharge, plugs into balance lead

### Wiring/Misc (~$25)

- [ ] XT60 connectors (male/female pairs) - ~$8
- [ ] Barrel jack 5.5x2.5mm (for NUC) - ~$5
- [ ] Wire 14AWG (power) + 22AWG (signal) - ~$12

**Tier 1 Subtotal: ~$912**

---

## To Buy - Tier 2 (Enhanced) ~$220

### Depth Perception

- [ ] Intel RealSense D435 - ~$180
  - Link: https://www.intelrealsense.com/depth-camera-d435/

### Orientation

- [ ] Adafruit BNO055 IMU - ~$25
  - Link: https://www.adafruit.com/product/4646

### Connectivity

- [ ] Powered USB 3.0 Hub - ~$20
  - Search: "Powered USB 3.0 hub 4 port"

**Tier 2 Subtotal: ~$220**

---

## Summary

| Category | Status | Cost |
|----------|--------|------|
| Already owned | Done | $0 (saved ~$128) |
| Tier 1 (Essential) | To buy | ~$912 |
| Tier 2 (Enhanced) | To buy | ~$220 |
| **Total** | | **~$1132** |

### Selected Build (Option B)

| Spec | Value |
|------|-------|
| Motors | Pololu 37D 131:1 (76 RPM, 45 kg-cm) |
| Wheels | 7.6" Wasteland (192mm) |
| Speed | 1.5 mph |
| Clearance | ~4" |
| Torque | Best for hills |

---

## Future Upgrades (Optional)

- [ ] Beitian BN-880 GPS (~$30) - if TN GPS insufficient
- [ ] Intel RealSense D455 (~$250) - better outdoor performance
- [ ] RPLidar A2 (~$300) - faster scan rate

---

## Architecture Notes

**NUC has NO GPIO** - requires microcontroller interface:

```
                         USB Serial
    Intel NUC (ROS2) ←──────────────→ Arduino Mega
                                           │
                                      PWM/DIR → Cytron MDD10A → Motors
                                      Encoder inputs (4x)

    USB-UART ──→ TN GPS
    USB ──────→ RPLidar A1
    USB ──────→ RealSense D435
    USB/I2C ──→ BNO055 IMU
```

**Power flow:**
```
    4S LiPo (14.8V)
         │
         ├──→ Fuse (20A)
         │         │
         │         ├──→ Boost 19V ──→ NUC
         │         ├──→ Buck 12V ───→ Motors (via MDD10A)
         │         └──→ Buck 5V ────→ Arduino, sensors
         │
         └──→ E-Stop switch (motor power only)
```

---

## Pre-Order Verification

- [x] Confirm 192mm Wasteland wheel hub pattern - VERIFIED: goBILDA 16mm thru-holes
- [ ] Check Pololu 37D mounting hole dimensions for motor plates
- [x] ~~Verify TN GPS voltage levels~~ - Have FTDI adapters with 3.3V/5V selection

---

*Last updated: 2024-12-27*
