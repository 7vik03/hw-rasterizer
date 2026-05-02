/*
 * rasterizer_test.c
 *
 * Minimal userspace validator for the rasterizer kernel driver.
 * Exercises every ioctl path so we can confirm the driver loads,
 * /dev/rasterizer is reachable, and copy_from/to_user works.
 *
 * NOTE: Without a real Avalon slave on the FPGA, the data values
 * read back from STATUS will be garbage (whatever the bus returns
 * for an unmapped or unconnected region — typically 0x0 or 0xFFFFFFFF).
 * That's fine — what we are validating here is the driver plumbing,
 * not the hardware behavior.
 *
 * Build:  gcc -Wall -o rasterizer_test rasterizer_test.c
 * Run:    sudo ./rasterizer_test
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include "avalon_kernel.h"

int main(void)
{
    int fd;
    rasterizer_arg_t ra;
    int ret;

    printf("=== rasterizer driver validation ===\n\n");

    /* ----- Test 1: open /dev/rasterizer ----- */
    printf("[1] opening /dev/rasterizer ... ");
    fd = open("/dev/rasterizer", O_RDWR);
    if (fd < 0) {
        printf("FAIL: %s\n", strerror(errno));
        printf("  Driver probably not loaded. Run:\n");
        printf("    sudo insmod avalon_kernel.ko\n");
        printf("  Then check dmesg | tail\n");
        return 1;
    }
    printf("OK (fd=%d)\n", fd);

    /* ----- Test 2: read STATUS ----- */
    printf("[2] ioctl(RASTERIZER_STATUS) ... ");
    memset(&ra, 0, sizeof(ra));
    ret = ioctl(fd, RASTERIZER_STATUS, &ra);
    if (ret != 0) {
        printf("FAIL: %s\n", strerror(errno));
    } else {
        printf("OK\n");
        printf("    fifo_level = %u\n", ra.status.fifo_level);
        printf("    fifo_full  = %u\n", ra.status.fifo_full);
        printf("    (values are bus garbage if no SV slave is connected)\n");
    }

    /* ----- Test 3: write CONTROL ----- */
    printf("[3] ioctl(RASTERIZER_SET_CONTROL) ... ");
    memset(&ra, 0, sizeof(ra));
    ra.control.irq_enable    = 0;
    ra.control.low_watermark = 16;
    ret = ioctl(fd, RASTERIZER_SET_CONTROL, &ra);
    if (ret != 0)
        printf("FAIL: %s\n", strerror(errno));
    else
        printf("OK (wrote irq_enable=0, low_watermark=16)\n");

    /* ----- Test 4: read CONTROL back from shadow ----- */
    printf("[4] ioctl(RASTERIZER_GET_CONTROL) ... ");
    memset(&ra, 0, sizeof(ra));
    ret = ioctl(fd, RASTERIZER_GET_CONTROL, &ra);
    if (ret != 0) {
        printf("FAIL: %s\n", strerror(errno));
    } else {
        printf("OK\n");
        printf("    irq_enable    = %u (expect 0)\n", ra.control.irq_enable);
        printf("    low_watermark = %u (expect 16)\n", ra.control.low_watermark);
        if (ra.control.irq_enable == 0 && ra.control.low_watermark == 16)
            printf("    PASS: shadow read matches what we wrote\n");
        else
            printf("    FAIL: shadow read did not match\n");
    }

    /* ----- Test 5: submit a fake triangle ----- */
    printf("[5] ioctl(RASTERIZER_SUBMIT) ... ");
    memset(&ra, 0, sizeof(ra));
    ra.packet.a0          = 0x11111111;
    ra.packet.b0          = 0x22222222;
    ra.packet.c0          = 0x33333333;
    ra.packet.flags_color = 0x000001E0;  /* front-facing red */
    ret = ioctl(fd, RASTERIZER_SUBMIT, &ra);
    if (ret != 0) {
        if (errno == EAGAIN)
            printf("EAGAIN (FIFO reported full — expected if STATUS returns 0xFF)\n");
        else
            printf("FAIL: %s\n", strerror(errno));
    } else {
        printf("OK (17 iowrite32s + 1 commit issued to bus)\n");
    }

    /* ----- Test 6: bad ioctl number ----- */
    printf("[6] ioctl(0xDEADBEEF) ... ");
    ret = ioctl(fd, 0xDEADBEEF, &ra);
    if (ret == 0)
        printf("FAIL: driver accepted unknown command\n");
    else
        printf("OK (rejected with %s)\n", strerror(errno));

    close(fd);
    printf("\n=== done ===\n");
    return 0;
}
