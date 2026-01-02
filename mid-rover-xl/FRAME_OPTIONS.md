# Mid-Rover XL Frame Options

The 500×350mm chassis is too large for most 3D printer beds. Here are the build options.

---

## Quick Comparison

| Option | Weight | Rigidity | Cost | Print Time | Difficulty |
|--------|--------|----------|------|------------|------------|
| A: Aluminum Tube | 550g | Best | $20 | 4 hrs (brackets only) | Easy |
| B: Hybrid Modular | 450g | Very Good | $15 | 8 hrs | Easy |
| C: Full Print (Halves) | 600g | Good | $15 | 16 hrs | Medium |
| D: Full Print (Panels) | 800g | Good | $20 | 20 hrs | Medium |

---

## Option A: Aluminum Tube + Printed Brackets (Recommended)

The original design. Aluminum square tube rails with 3D printed corners and mounts.

```
        ════════════════════════════════════════
       ║                                        ║
   [Corner]                                [Corner]
       ║                                        ║
       ║              500mm                     ║
       ║         aluminum tube                  ║
       ║                                        ║
   [Corner]                                [Corner]
       ║                                        ║
        ════════════════════════════════════════
                      350mm
```

### Materials
| Item | Qty | Price |
|------|-----|-------|
| 1" Al square tube 6ft | 1 | $15 |
| 3D printed corners | 4 | $4 filament |
| 3D printed servo mounts | 4 | $4 filament |
| M5 hardware | - | $5 |

### Cuts
- 2× 500mm (long rails)
- 2× 350mm (cross members)
- Total: 1.7m from 1.8m (6ft) tube

### Printed Parts
| Part | Size | Weight | Time |
|------|------|--------|------|
| Corner bracket ×4 | 80×80×40mm | 25g each | 1 hr each |
| Servo mount ×4 | 60×50×30mm | 20g each | 45 min each |
| Electronics tray | 180×100×20mm | 60g | 2 hrs |
| **Total** | - | **240g** | **~8 hrs** |

### Pros
- Stiffest option
- Easiest to build
- Most adjustable
- Handles crashes well

### Cons
- Requires cutting aluminum
- Need to drill holes
- Slightly heavier

---

## Option B: Hybrid Modular (Printed + Rod)

Printed corner modules connected by aluminum or steel rods running through printed tubes.

```
    ┌──────────┐══════════════════════┌──────────┐
    │  CORNER  │      6mm Al rod      │  CORNER  │
    │  MODULE  │══════════════════════│  MODULE  │
    │   (L)    │                      │   (R)    │
    └────╫─────┘                      └────╫─────┘
         ║                                  ║
         ║  printed tube                    ║
         ║  with rod inside                 ║
         ║                                  ║
    ┌────╫─────┐                      ┌────╫─────┐
    │  CORNER  │══════════════════════│  CORNER  │
    │  MODULE  │      6mm Al rod      │  MODULE  │
    │   (L)    │                      │   (R)    │
    └──────────┘══════════════════════└──────────┘
```

### Materials
| Item | Qty | Price |
|------|-----|-------|
| 6mm aluminum rod 1m | 3 | $8 |
| PETG filament | 350g | $9 |
| M3/M4 hardware | - | $5 |

### Printed Parts
| Part | Size | Print Bed Fit | Weight |
|------|------|---------------|--------|
| Corner module ×4 | 120×100×50mm | ✓ 250mm bed | 80g each |
| Tube sleeve ×4 | 200×30×30mm | ✓ 250mm bed | 15g each |
| Electronics tray | 180×100×20mm | ✓ 250mm bed | 60g |
| **Total** | - | - | **~450g** |

### Assembly
1. Print 4 corner modules (include servo mount + rod holes)
2. Print 4 tube sleeves (hollow with rod channel)
3. Cut aluminum rods: 2× 500mm, 2× 350mm
4. Slide rods through sleeves and into corner modules
5. Secure with set screws or printed caps

### Pros
- Lightest option
- All parts fit standard print beds
- Rods add stiffness where needed
- Easy to disassemble

### Cons
- More printed parts
- Alignment can be tricky
- Rods can slide if not secured

---

## Option C: Full Print in Halves

Print two halves of the frame and bolt together.

```
         ┌─────────────────────┬─────────────────────┐
         │                     │                     │
         │      LEFT HALF      │     RIGHT HALF      │
         │      250×350mm      │      250×350mm      │
         │                     │                     │
         │    [servo]          │          [servo]    │
         │                     │                     │
         │                     │                     │
         │                     │                     │
         │    [servo]          │          [servo]    │
         │                     │                     │
         └─────────────────────┴─────────────────────┘
                               ↑
                         Bolted splice
```

### Print Specs
| Half | Size | Weight | Time | Bed Required |
|------|------|--------|------|--------------|
| Left | 250×350mm | 300g | 8 hrs | 250×350mm+ |
| Right | 250×350mm | 300g | 8 hrs | 250×350mm+ |

**Note:** Requires 350mm Y-axis. Prusa MK4S is only 210mm Y.

### Alternative: Print Rotated
- Print each half at 45° diagonal
- Fits 250×210mm bed (just barely for 250mm piece)
- Wastes more filament on supports

### Splice Joint
```
    ───────┐   ┌───────
           │   │
    ───────┘   └───────
           ↑   ↑
    Printed tabs with M4 bolts
    or aluminum splice plate underneath
```

### Pros
- Fewest separate parts
- Clean integrated look
- Servo mounts built-in

### Cons
- Needs large print bed OR diagonal printing
- Long print times
- Splice joint is a weak point
- Harder to repair if damaged

---

## Option D: Full Print (Panel Construction)

Print flat panels and assemble into box frame structure.

```
         TOP VIEW (assembled)
    ┌─────────────────────────────────┐
    │ ┌─────────────────────────────┐ │
    │ │                             │ │
    │ │      TOP DECK PANEL         │ │
    │ │        (printed)            │ │
    │ │                             │ │
    │ └─────────────────────────────┘ │
    │                                 │
    │   SIDE      ▓▓▓▓▓▓▓      SIDE   │
    │   RAIL      ▓▓▓▓▓▓▓      RAIL   │
    │             ▓▓▓▓▓▓▓             │
    │                                 │
    └─────────────────────────────────┘

         SIDE VIEW
    ┌──────────────────────────────────┐  ← Top deck
    │▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓│  ← Side rails
    └──────────────────────────────────┘  ← Bottom optional
```

### Printed Parts
| Panel | Size | Qty | Weight | Time |
|-------|------|-----|--------|------|
| Top deck (split) | 250×175mm | 2 | 150g each | 4 hrs each |
| Side rail | 500×40×40mm | 2 | 80g each | 3 hrs each |
| End rail | 350×40×40mm | 2 | 60g each | 2 hrs each |
| Corner gusset | 60×60×40mm | 4 | 20g each | 1 hr each |
| **Total** | - | - | **~800g** | **~20 hrs** |

### Pros
- Box structure is very rigid
- All panels fit standard beds
- Enclosed space protects electronics
- Can add bottom panel for full enclosure

### Cons
- Heaviest option
- Most print time
- Complex assembly
- Harder to access internals

---

## Comparison By Printer

### Prusa MK4S (250×210mm bed)

| Option | Compatible? | Notes |
|--------|-------------|-------|
| A: Al Tube | ✓ Yes | Corners fit easily |
| B: Hybrid | ✓ Yes | All parts fit |
| C: Halves | ⚠ Tight | Need diagonal print |
| D: Panels | ✓ Yes | Top deck needs split |

### Ender 3 (220×220mm bed)

| Option | Compatible? | Notes |
|--------|-------------|-------|
| A: Al Tube | ✓ Yes | Corners fit |
| B: Hybrid | ✓ Yes | Tube sleeves need 200mm |
| C: Halves | ✗ No | Too small |
| D: Panels | ⚠ Tight | Side rails need diagonal |

### Large Format (300×300mm+)

| Option | Compatible? | Notes |
|--------|-------------|-------|
| All | ✓ Yes | Full flexibility |

---

## Material Recommendations

| Material | Use Case | Settings |
|----------|----------|----------|
| PETG | All structural parts | 240°C, 80°C bed, 3 walls |
| PLA | Low-stress brackets | 210°C, 60°C bed |
| ASA | Outdoor use | 260°C, 100°C bed, enclosure |
| CF-PETG | Maximum strength | 260°C, requires hardened nozzle |

### Infill Guide
| Part Type | Infill | Pattern |
|-----------|--------|---------|
| Corner brackets | 40% | Grid |
| Servo mounts | 50% | Grid |
| Rails/tubes | 30% | Gyroid |
| Electronics tray | 20% | Grid |

---

## Recommendation

**For Prusa MK4S:** Go with **Option B (Hybrid Modular)**
- All parts print easily
- Light and stiff
- ~$15 materials, ~8 hours print
- 6mm aluminum rod adds backbone

**For quick build:** Go with **Option A (Aluminum Tube)**
- Buy one 6ft tube, make 4 cuts
- Print corners overnight
- Strongest result

**For fully printed:** Go with **Option D (Panels)**
- More work but satisfying
- Good if you want enclosed electronics bay

---

## STL Files Needed

### Option A (Al Tube)
```
corner_bracket_left.stl   (×2, mirror for right)
corner_bracket_right.stl  (×2)
servo_mount.stl           (×4)
electronics_tray.stl      (×1)
lidar_mount.stl           (×1)
```

### Option B (Hybrid)
```
corner_module_FL.stl      (front-left)
corner_module_FR.stl      (front-right, mirrored)
corner_module_RL.stl      (rear-left)
corner_module_RR.stl      (rear-right, mirrored)
tube_sleeve_long.stl      (×2, for 500mm rails)
tube_sleeve_short.stl     (×2, for 350mm rails)
electronics_tray.stl      (×1)
```

### Option D (Panels)
```
top_deck_left.stl         (×1)
top_deck_right.stl        (×1)
side_rail.stl             (×2)
end_rail.stl              (×2)
corner_gusset.stl         (×4)
servo_mount.stl           (×4)
```

---

*Design these in Fusion 360, FreeCAD, or OnShape. Search Printables/Thingiverse for similar designs to adapt.*
