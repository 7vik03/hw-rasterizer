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

Main columns to check:

```text
setup_ms          software triangle setup time
submit_ms         ioctl/Avalon submit time
present_ms        wait after PRESENT
fifo_max          max FIFO level seen, 64 means full
fifo_full_polls   how often software had to wait for FIFO space
hw_cycle_est      rough bbox-based hardware workload estimate
```
