ifneq ($(KERNELRELEASE),)
# Module name must match the .c filename: avalon_kernel.c → avalon_kernel.o
obj-m := avalon_kernel.o

else

KERNEL_SOURCE := /usr/src/linux
PWD           := $(shell pwd)

.PHONY: default module userspace clean

default: module userspace

# Build the kernel module → avalon_kernel.ko
module:
	$(MAKE) -C $(KERNEL_SOURCE) SUBDIRS=$(PWD) modules

# Build the userspace test program → rasterizer_test
userspace: rasterizer_test

rasterizer_test: rasterizer_test.c avalon_kernel.h
	gcc -Wall -o rasterizer_test rasterizer_test.c

# Compile the device tree blob → socfpga.dtb
socfpga.dtb: socfpga.dts
	dtc -O dtb -o socfpga.dtb socfpga.dts

# Clean everything
clean:
	$(MAKE) -C $(KERNEL_SOURCE) SUBDIRS=$(PWD) clean
	$(RM) rasterizer_test socfpga.dtb

endif
