# 3D Hardware Rasterizer — Golden Reference & Hardware Spec

## Project Overview

This software rasterizer is the **golden reference** for our CSEE 4840 hardware rasterizer project. It implements the exact algorithm and data formats that the FPGA hardware will replicate. Every pixel this software produces is a test vector for hardware verification.

**What the final demo looks like:** A 3D object (sphere, torus, or teapot) displayed on a VGA monitor connected to the DE1-SoC, rotating in real time via keyboard input. Smooth animation, no flickering, per-face lighting giving visible 3D depth.

---

## Display Specs

| Parameter | Value |
|---|---|
| Internal resolution | 320×240 |
| VGA output | 640×480 (each pixel doubled 2×2) |
| Color format | RGB332 (3 bits red, 3 bits green, 2 bits blue) |
| Color depth | 8 bits per pixel, 256 possible colors |
| Z-buffer | 16-bit unsigned per pixel |
| Double buffering | Yes — one buffer displayed, one being written |
| Frame rate target | 30+ FPS interactive |

## Memory Budget (Cyclone V — 550KB M10K block RAM)

| Component | Size |
|---|---|
| Framebuffer A (display) | 320×240×8 bits = 75 KB |
| Framebuffer B (write) | 75 KB |
| Z-buffer | 320×240×16 bits = 150 KB |
| **Total** | **300 KB (55% of available)** |

Each of these is partitioned across N parallel pixel units (targeting 4), so each unit owns its own local memory bank with no contention.

## Target Models

| Model | Vertices | Faces | Purpose |
|---|---|---|---|
| Cube | 8 | 12 | Integration testing |
| Sphere (low) | 42 | 80 | Fallback demo |
| Sphere (med) | 162 | 320 | Primary demo target |
| Torus | 96 | 192 | Alternative demo |
| Teapot | 416 | 792 | Stretch goal |

---

## Algorithm: Pineda Edge Equation Rasterization

For each triangle with screen-space vertices v0, v1, v2, we define three edge functions:

```
Edge 0 (v1 → v2):  E0(x,y) = a0*x + b0*y + c0
Edge 1 (v2 → v0):  E1(x,y) = a1*x + b1*y + c1
Edge 2 (v0 → v1):  E2(x,y) = a2*x + b2*y + c2

where:
  a_i = (start_y - end_y)
  b_i = (end_x - start_x)
  c_i = (start_x * end_y - end_x * start_y)
```

A pixel at (x, y) is inside the triangle if and only if **all three edge functions are ≥ 0**.

The key property for hardware: when stepping one pixel right, the edge function updates by a single addition:

```
E_i(x+1, y) = E_i(x, y) + a_i    ← one addition
E_i(x, y+1) = E_i(x, y) + b_i    ← one addition
```

**No multiplication or division in the inner pixel loop.** Three additions and three sign checks per pixel. This is what makes it ideal for FPGA implementation.

### Depth Interpolation

Depth is interpolated incrementally using the same principle:

```
z(x+1, y) = z(x, y) + z_step_x   ← one addition
z(x, y+1) = z(x, y) + z_step_y   ← one addition
```

The step values `z_step_x` and `z_step_y` are precomputed in software from the barycentric relationship between edge functions and vertex depths.

### Early Z-Rejection

Before writing a pixel, read the existing Z-buffer value. If the new pixel is behind (depth ≥ stored depth), skip it immediately. This avoids wasted framebuffer writes on occluded geometry.

---

## Software/Hardware Split

### What runs on ARM (software — floating point, per-triangle)

1. **User input** — read keyboard, update rotation angles
2. **Matrix transforms** — model × view × projection applied to every vertex
3. **Perspective divide** — clip.x/clip.w, clip.y/clip.w → screen coordinates
4. **Per-face lighting** — face normal · light direction = brightness → RGB332 color
5. **Triangle setup** — compute edge coefficients, bounding box, depth step values
6. **FIFO submission** — pack triangle packet, write to hardware over Avalon bus

### What runs on FPGA (hardware — integer only, per-pixel)

1. **FIFO pull** — read precomputed triangle packet
2. **Distribute** — send triangle to all parallel pixel units
3. **Bounding box traversal** — iterate over pixel region
4. **Edge test** — three additions + three sign checks per pixel
5. **Early Z test** — compare interpolated depth against Z-buffer
6. **Framebuffer write** — write RGB332 color if pixel passes both tests
7. **VGA scan-out** — continuously read display buffer to monitor

---

## Triangle Packet Format (Software → Hardware FIFO)

This is the exact data that crosses the Avalon bus for each triangle:

```
struct TrianglePacket {
    // Edge equation coefficients (Q12.12 fixed point)
    int32_t a0, b0, c0;    // Edge 0: v1 → v2
    int32_t a1, b1, c1;    // Edge 1: v2 → v0
    int32_t a2, b2, c2;    // Edge 2: v0 → v1
    // c0/c1/c2 are initial values at (bbox_xmin+0.5, bbox_ymin+0.5)

    // Bounding box (integer screen coordinates)
    int bbox_xmin, bbox_ymin, bbox_xmax, bbox_ymax;

    // Depth interpolation (Q12.12 fixed point)
    int32_t z_at_origin;    // depth at (bbox_xmin, bbox_ymin)
    int32_t z_step_x;       // depth change per pixel in x
    int32_t z_step_y;       // depth change per pixel in y

    // Color
    uint8_t color;           // RGB332 flat color for this face

    // Culling
    bool front_facing;       // false = skip this triangle
};
```

**Total: ~20 values per triangle, approximately 80 bytes per packet.**

## Fixed-Point Format

All hardware arithmetic uses Q12.12 signed fixed point:
- 1 sign bit + 11 integer bits + 12 fractional bits = 24 bits (stored in 32-bit int)
- Range: -2048.0 to +2047.999
- Precision: 1/4096 ≈ 0.000244
- Conversion: `fixed = (int32_t)(float_value * 4096.0)`

The inner pixel loop uses only:
- `int32_t` addition (edge function stepping)
- `int32_t` comparison (sign check, Z-buffer compare)
- bit shift (convert fixed-point depth to 16-bit unsigned)

---

## Hardware Rasterizer Inner Loop (What the SystemVerilog Does)

This is the exact C code that the FPGA replicates. Every operation here maps to hardware:

```c
// Runs for each triangle pulled from FIFO
// ALL INTEGER ARITHMETIC — no float, no multiply, no divide

int32_t e0_row = pkt.c0, e1_row = pkt.c1, e2_row = pkt.c2;
int32_t z_row = pkt.z_at_origin;

for (int y = pkt.bbox_ymin; y <= pkt.bbox_ymax; y++) {
    int32_t e0 = e0_row, e1 = e1_row, e2 = e2_row;
    int32_t z = z_row;

    for (int x = pkt.bbox_xmin; x <= pkt.bbox_xmax; x++) {

        // INSIDE TEST: 3 sign checks
        if (e0 >= 0 && e1 >= 0 && e2 >= 0) {

            uint16_t depth = (uint16_t)(z >> 12);  // fixed → integer

            // EARLY Z: compare before writing
            if (depth < zbuffer[y][x]) {
                zbuffer[y][x] = depth;
                framebuffer[y][x] = pkt.color;
            }
        }

        // STEP X: 3 additions (+ 1 for depth)
        e0 += pkt.a0;
        e1 += pkt.a1;
        e2 += pkt.a2;
        z  += pkt.z_step_x;
    }

    // STEP Y: 3 additions (+ 1 for depth)
    e0_row += pkt.b0;
    e1_row += pkt.b1;
    e2_row += pkt.b2;
    z_row  += pkt.z_step_y;
}
```

**Per pixel: 4 additions + 3 comparisons + 1 shift + 1 memory read + 1 conditional write.**
No multiplier. No divider. This is the entire datapath.

---

## Hardware Architecture Summary

```
ARM Software
    │
    │  triangle packets (edge coeffs, bbox, color, depth)
    ▼
┌──────────────────────┐
│  Avalon Bus Interface │
└──────────┬───────────┘
           ▼
┌──────────────────────┐
│  Triangle FIFO       │ ← low water mark interrupt → ARM
│  (4-8 triangle deep) │
└──────────┬───────────┘
           ▼
┌──────────────────────┐
│  Distribute to units │
└──┬───────┬───────┬───┘
   ▼       ▼       ▼
┌──────┐┌──────┐┌──────┐
│Unit 0││Unit 1││Unit N│  Each unit:
│      ││      ││      │  - edge eval (3 adds)
│Own FB││Own FB││Own FB│  - early Z test
│Own ZB││Own ZB││Own ZB│  - own memory partition
└──┬───┘└──┬───┘└──┬───┘
   ▼       ▼       ▼
┌──────────────────────┐
│  Double Framebuffer  │ ← swap on frame complete
│  A (display) / B     │
└──────────┬───────────┘
           ▼
┌──────────────────────┐
│  VGA Controller      │
│  640×480, 2× doubled │
└──────────┬───────────┘
           ▼
       VGA Monitor
```

## Parallel Pixel Units

Target: **4 parallel units**, each owning a horizontal band of 320×60 pixels.

Per unit memory:
- Framebuffer partition: 320×60×8 bits = 18.75 KB
- Z-buffer partition: 320×60×16 bits = 37.5 KB

No memory contention — each unit reads/writes only its own local M10K blocks.

When a triangle spans multiple partitions, each unit independently clips to its own region (ignores pixels outside its y-range).

---

## Golden Reference Files

| File | Purpose |
|---|---|
| `rasterizer.cpp` | Static reference — outputs PPM image + test_vectors.txt |
| `demo.cpp` | Interactive SDL2 demo — real-time rotation with keyboard |
| `gen_models.cpp` | Generates OBJ test models |
| `sphere_med.obj` | 320-face sphere (primary demo target) |
| `sphere_lo.obj` | 80-face sphere (fallback) |
| `torus.obj` | 192-face torus |
| `teapot.obj` | 792-face teapot (stretch) |
| `test_vectors.txt` | Triangle packets + expected pixel output for HW verification |

### Build & Run

```bash
# Static reference (generates test vectors)
g++ -O2 -std=c++17 -o rasterizer rasterizer.cpp -lm
./rasterizer sphere_med.obj 25 45 0

# Interactive demo (requires SDL2: brew install sdl2)
g++ -O2 -std=c++17 -o demo demo.cpp -I/opt/homebrew/include/SDL2 \
    -L/opt/homebrew/lib -lSDL2 -lm
./demo

# Generate models
g++ -O2 -std=c++17 -o gen_models gen_models.cpp -lm
./gen_models
```

### Interactive Demo Controls

| Key | Action |
|---|---|
| Arrow keys | Rotate object |
| 1, 2, 3, 4 | Switch model (cube, sphere_lo, sphere_med, torus) |
| +/- | Zoom in/out |
| Space | Toggle auto-rotate |
| R | Reset view |
| Q / ESC | Quit |

---

## Test Vector Format

`test_vectors.txt` contains two sections:

**Triangle packets** (what software sends to FIFO):
```
TRI <idx> <a0> <b0> <c0> <a1> <b1> <c1> <a2> <b2> <c2>
    <xmin> <ymin> <xmax> <ymax> <z_origin> <z_step_x> <z_step_y>
    <color> <front_facing>
```

**Expected pixel outputs** (what hardware should produce):
```
PIX <x> <y> <depth> <color>
```

Hardware verification: feed the TRI packets into your SystemVerilog testbench, capture the pixel writes, and compare against the PIX lines. Any mismatch indicates a bug in the hardware implementation.
