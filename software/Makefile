ifneq ($(KERNELRELEASE),)
# Module name must match the .c filename: avalon_kernel.c → avalon_kernel.o
obj-m := avalon_kernel.o

else

KERNEL_SOURCE := /usr/src/linux
PWD           := $(shell pwd)

CC      := gcc
CFLAGS  := -Wall -O2 -I$(PWD)/kernel

.PHONY: default module userspace clean

default: module userspace

# Build the kernel module → avalon_kernel.ko
module:
	$(MAKE) -C $(KERNEL_SOURCE) SUBDIRS=$(PWD) modules

# Build all userspace programs
userspace: rasterizer_test demo

# Minimal driver validation tool (no geometry / model deps)
rasterizer_test: kernel/rasterizer_test.c kernel/avalon_kernel.h
	$(CC) $(CFLAGS) -o rasterizer_test kernel/rasterizer_test.c

# Full interactive demo
DEMO_SRCS := userspace/main.c userspace/geometry.c userspace/model.c \
             userspace/usbkeyboard.c
DEMO_HDRS := userspace/geometry.h userspace/model.h \
             userspace/usbkeyboard.h kernel/avalon_kernel.h

demo: $(DEMO_SRCS) $(DEMO_HDRS)
	$(CC) $(CFLAGS) -o demo $(DEMO_SRCS) -lm -lusb-1.0

# Compile the device tree blob → socfpga.dtb
socfpga.dtb: socfpga.dts
	dtc -O dtb -o socfpga.dtb socfpga.dts

# Clean everything
clean:
	$(MAKE) -C $(KERNEL_SOURCE) SUBDIRS=$(PWD) clean
	$(RM) rasterizer_test demo socfpga.dtb

endif
