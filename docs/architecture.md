# Hardware Rasterizer Architecture

Target: DE1-SoC (Cyclone V `5CSEMA5F31C6N`). Internal framebuffer is
256x240 RGB332, scanned out as 512x480 inside a 640x480 VGA frame with
64-pixel black bars on left and right.

## Block diagram

```
             +-------------------+
   HPS  -->  | avalon_interface  |  (pop / pop_ACK handshake)
             +---------+---------+
                       |
                       v
             +-------------------+
             | triangle_dispatcher|
             +---------+---------+
                       | broadcast packet (constants)
                       | + valid_out[0] -> pu[0].seed_valid_in
                       v
             +---+---+---+---+        ...        +---+
             | 0 | 1 | 2 | 3 |  systolic chain   |15 |
             +-+-+-+-+-+-+-+-+        ...        +-+-+
               |   |   |   |                       |
               |   v   v   v                       v
               +--> seed propagation (+a between adjacent PUs)
               |
               +-- per-PU FB (A/B) and Z lives inside each pixel_unit
                       |
                       v VGA scan-out reads
             +-------------------+
             | vga_framebuffer   |  -> VGA pins
             +-------------------+
```

## Pixel-unit chain

There are 16 pixel units, each owning a 16-column bank of the screen:
PU `i` is responsible for the screen columns whose low nibble equals
`i`. They run a Pineda edge-function rasterizer column-by-column down
the screen, walking the row by `+b` each cycle.

The seed `(e0, e1, e2, z)` for the first pixel of a triangle enters the
chain at PU0. On the same clock edge that PU0 latches the seed it also
forwards `(seed + a)` to PU1, which latches it one cycle later and
forwards `(seed + 2a)` to PU2, and so on. After 16 cycles every PU is
walking its column in lockstep with the others, one row apart on the
diagonal.

Steady-state throughput is therefore **16 pixels/cycle** (one per PU).
Fill latency is 16 cycles; for a triangle of bbox-height H the total
runtime is H + 16 + a couple of pipeline cycles for the read-modify-
write z-test.

### Multi-column iteration

After a PU finishes walking its first column, it jumps 16 columns to
the right (PU_ID, PU_ID + 16, PU_ID + 32, ...) and walks the next
column. This continues until the PU's column index exceeds bbox_xmax,
at which point it returns to IDLE. The systolic seed propagation only
seeds the FIRST column for each PU; subsequent columns are computed
internally using stored column-top edge values plus 16*a -- the live
e/z accumulators have walked +b for many cycles by end-of-column and
can't be reused, so the PU keeps a separate snapshot of (e0, e1, e2,
z) at the top of the current column. That snapshot, plus 16 left-shift
of a/z_step_x, gives the seed for the next column without any
contribution from the chain. The dispatcher still issues exactly one
broadcast per triangle.

## Memory layout

| Name              | Width | Depth | Per-PU M10K | Total M10K | Notes                       |
| ----------------- | ----- | ----- | ----------- | ---------- | --------------------------- |
| Color framebuffer | 8     | 4096  | 4 (8x1024)  | 4 * 16 * 2 = **128** | Double-buffered (A/B). |
| Z-buffer          | 16    | 4096  | 8 (16x512)  | 8 * 16 * 1 = **128** | Single-buffered.       |
| Triangle FIFO     | wide  | 64    | (separate)  | a few      | Outside the PU chain.       |

Each PU's framebuffer slice holds its 16-column bank: 16 cols x 240
rows = 3,840 entries, rounded to 4,096 (the next power of two).
Address layout inside a PU is `{x[7:4], y[7:0]}`, a 12-bit local
address. The high nibble of x picks the column bank inside the PU
(4-deep M10K chain), the low byte is the row.

The double-buffered color framebuffer toggles between `fb_a_mem` and
`fb_b_mem` once per VGA frame on `vga_framebuffer.frame_done`. The
rasterizer writes one buffer while the VGA scanout reads the other, so
swaps never tear mid-frame.

The z-buffer is single-buffered: the rasterizer must clear it once per
frame (or the host CPU does) so that "smaller depth wins" remains a
correct test.

### M10K inference

Every memory in `pixel_unit.sv` carries a Quartus synthesis attribute
that pins the implementation to the configuration we want:

```systemverilog
(* ramstyle = "M10K", max_depth = 1024 *) logic [7:0]  fb_a_mem [4096];
(* ramstyle = "M10K", max_depth = 1024 *) logic [7:0]  fb_b_mem [4096];
(* ramstyle = "M10K", max_depth = 512  *) logic [15:0] z_mem    [4096];
```

Without these the inference defaults are wasteful (Quartus tends to
fall back to LUT-RAM for small or oddly-sized arrays, or pick a 1-bit
wide M10K config that wastes most of the block). With them the
synthesis report should show **4 M10K per FB-buffer per PU** and
**8 M10K per Z-buffer per PU**, no LUTs spent on bank-decode.

## Address decode

The screen address is 16 bits: `{y[7:0], x[7:0]}`.

| Bits     | Selects                              |
| -------- | ------------------------------------ |
| `x[3:0]` | which PU (one of 16)                 |
| `x[7:4]` | column bank inside the PU (1 of 16)  |
| `y[7:0]` | row inside the column bank (1 of 240) |

There is no hand-coded bank chain inside the PU. Quartus chains the
4 (FB) or 8 (Z) M10Ks automatically based on the array size and the
`max_depth` attribute.

## Rasterizer dataflow per triangle

1. Software computes Pineda edge coefficients `(a_i, b_i)`, edge
   initial values `e_i_init` at `(bbox_xmin, bbox_ymin)`, depth
   constants `(z_at_origin, z_step_x, z_step_y)`, and a color, and
   pushes a packet through `avalon_interface` into `triangle_fifo`.
2. `triangle_dispatcher` waits until every PU is `ready` (chain fully
   drained), then pops the packet and broadcasts it. `valid_out[0]`
   pulses for one cycle to seed PU0.
3. PU0 latches the seed and constants, transitions IDLE -> ACTIVE,
   and on the same edge registers `(seed + a)` onto its
   `seed_*_out` lines and pulses `seed_valid_out`.
4. PU1 sees `seed_valid_in` one cycle later and does the same; the
   wave propagates down the chain.
5. Each PU walks `+b` per cycle until `cur_row == last_row`, at which
   point it returns to IDLE and reasserts `ready`.
6. Pixel writes inside each PU pipeline a synchronous z-buffer read
   on cycle N and a conditional write on cycle N+1, using the M10K's
   two ports.
7. Once all PUs are back to IDLE, the dispatcher pops the next
   triangle.

## VGA scan-out

`vga_framebuffer` runs the standard 640x480/60 timing. It exposes
`(fb_x, fb_y)` of the pixel currently being scanned (combinational
from `hcount/vcount`) plus an `in_fb_region` flag and a one-cycle
`frame_done` pulse at end-of-field.

`rasterizer_top` muxes the 16 PU read ports based on `fb_x[3:0]` and
feeds the byte back as `fb_pixel_color`. The PU read latency is 1
cycle but `fb_x` is held for 4 hcount cycles per pixel (2x pixel
doubling, plus 2 clock cycles per VGA pixel), so no extra lookahead is
needed -- the result lands well inside the same `fb_x` hold window.

Letterbox bars at `pixel_x in [0, 63]` and `[576, 639]` are gated to
black inside `rasterizer_top` before reaching the VGA RGB encoder.

## What is *not* changing

- `triangle_packet.svh` -- the wire format between dispatcher and PUs
  is unchanged.
- `triangle_fifo.sv` -- independent of PU count.
- `avalon_interface.sv` -- owned by Shlok.
- The HPS-side Avalon register map.
