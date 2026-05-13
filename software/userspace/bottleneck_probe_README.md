# bottleneck_probe quick run

From the repo root:

```sh
cd software
make bottleneck_probe
```

Software-only check, no FPGA or `/dev/rasterizer` needed:

```sh
./bottleneck_probe --dry-run --frames 60 --model 8
./bottleneck_probe --dry-run --frames 60 ../800.obj
```

On the FPGA, after the rasterizer driver is loaded:

```sh
./bottleneck_probe --frames 60 --model 8
./bottleneck_probe --frames 60 ../800.obj
```

If `/dev/rasterizer` says permission denied, ask someone with sudo to run:

```sh
sudo chmod 666 /dev/rasterizer
```

CSV columns:

```text
frame             frame index within this probe run
model             model name
faces             total faces in the input model
built             triangles that survived setup/culling and produced packets
culled            triangles rejected by setup_triangle() as degenerate, back-facing, or off-screen
render_ms         total CPU-side time to prepare and submit the frame's triangles
project_ms        vertex projection time only
setup_ms          triangle packet construction time only (does not include projection)
submit_ms         packet submission time only (MMIO or ioctl path)
present_ms        time from PRESENT until swap/clear completes
fps               end-to-end frames/second estimate using render_ms + present_ms
fifo_max          maximum FIFO occupancy observed by the probe, 64 means full
fifo_full_polls   how often the probe had to wait because the FIFO was full
eagain            ioctl fallback only: number of EAGAIN retries during submit
status_polls      total STATUS reads performed by the probe
bbox_avg_px       average triangle bounding-box area in pixels for submitted triangles
hw_cycle_est      rough bbox-based hardware workload estimate, not measured FPGA cycles
```

Interpretation notes:

```text
render_ms and present_ms are the most useful report-level throughput numbers.
project_ms + setup_ms separate CPU geometry work from packet submission overhead.
present_ms is not pure raster time; it also includes handoff/polling/swap-clear latency.
fifo_max is sampled by the probe and should be treated as approximate occupancy evidence.
hw_cycle_est is a workload proxy based on bbox height and 16-column bank coverage.
```
