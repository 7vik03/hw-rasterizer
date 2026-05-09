ifneq ($(KERNELRELEASE),)
# Module name must match the .c filename: avalon_kernel.c → avalon_kernel.o
obj-m := avalon_kernel.o

else

KERNEL_SOURCE := /usr/src/linux-headers-4.19.0
PWD           := $(shell pwd)

CC      := gcc
CFLAGS  := -Wall -O2 -I$(PWD)/kernel

.PHONY: default module userspace clean

default: module userspace

# Build the kernel module → avalon_kernel.ko
module:
	$(MAKE) -C $(KERNEL_SOURCE) M=$(PWD)/kernel modules

# Build all userspace programs
userspace: rasterizer_test demo

# Minimal driver validation tool (no geometry / model deps)
rasterizer_test: userspace/rasterizer_test.c kernel/avalon_kernel.h
	$(CC) $(CFLAGS) -o rasterizer_test userspace/rasterizer_test.c

# Full interactive demo
DEMO_SRCS := userspace/main.c userspace/geometry.c userspace/model.c \
             userspace/usbkeyboard.c
DEMO_HDRS := userspace/geometry.h userspace/model.h \
             userspace/usbkeyboard.h kernel/avalon_kernel.h

demo: $(DEMO_SRCS) $(DEMO_HDRS)
	$(CC) $(CFLAGS) -o demo $(DEMO_SRCS) -lm -lusb-1.0

# Standalone bottleneck profiler. Not part of the default userspace target
# so the working interactive demo path remains untouched.
bottleneck_probe: userspace/bottleneck_probe.c userspace/geometry.c \
                  userspace/model.c userspace/geometry.h \
                  userspace/model.h kernel/avalon_kernel.h
	$(CC) $(CFLAGS) -o bottleneck_probe userspace/bottleneck_probe.c \
	    userspace/geometry.c userspace/model.c -lm

# Compile the device tree blob → socfpga.dtb
socfpga.dtb: socfpga.dts
	dtc -O dtb -o socfpga.dtb socfpga.dts

# Clean everything
clean:
	$(MAKE) -C $(KERNEL_SOURCE) M=$(PWD)/kernel clean
	$(RM) rasterizer_test demo bottleneck_probe socfpga.dtb

endif
