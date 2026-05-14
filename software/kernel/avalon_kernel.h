// avalon_kernel.h

#ifndef _RASTERIZER_H
#define _RASTERIZER_H

#include <linux/ioctl.h>
#include <linux/types.h>

typedef struct {
    __s32 a0, b0, c0;
    __s32 a1, b1, c1;
    __s32 a2, b2, c2;
    __s32 e0_init;
    __s32 e1_init;
    __s32 e2_init;
    __s32 z_origin;
    __s32 z_step_x;
    __s32 z_step_y;
    __u32 bbox_packed;
    __u32 flags_color;
} triangle_packet_t;


typedef struct {
    __u32 fifo_level;
    __u32 fifo_empty;
    __u32 fifo_full;
    __u32 swap_busy;
} rasterizer_status_t;


typedef struct {
    __u32 irq_enable;
    __u32 low_watermark;
} rasterizer_control_t;


typedef union {
    triangle_packet_t  packet;
    rasterizer_status_t  status;
    rasterizer_control_t control;
} rasterizer_arg_t;


#define RAST_PACKET_WORD_BASE   0x00
#define RAST_PACKET_NUM_WORDS   17
#define RAST_COMMIT_OFFSET      0x44
#define RAST_STATUS_OFFSET      0x48
#define RAST_CONTROL_OFFSET     0x4C
#define RAST_PRESENT_OFFSET     0x50


#define RAST_STATUS_LEVEL_MASK      0x3F
#define RAST_STATUS_EMPTY_BIT       (1<<6)
#define RAST_STATUS_FULL_BIT        (1<<7)
#define RAST_STATUS_SWAP_BUSY_BIT   (1<<8)


#define RAST_CTRL_IRQ_EN_BIT    (1<<0)
#define RAST_CTRL_WMARK_SHIFT   1
#define RAST_CTRL_WMARK_MASK    0x7F


#define RASTERIZER_MAGIC        'R'


#define RASTERIZER_SUBMIT       _IOW(RASTERIZER_MAGIC, 1, rasterizer_arg_t *)
#define RASTERIZER_STATUS       _IOR(RASTERIZER_MAGIC, 2, rasterizer_arg_t *)
#define RASTERIZER_SET_CONTROL  _IOW(RASTERIZER_MAGIC, 3, rasterizer_arg_t *)
#define RASTERIZER_GET_CONTROL  _IOR(RASTERIZER_MAGIC, 4, rasterizer_arg_t *)
#define RASTERIZER_PRESENT _IO(RASTERIZER_MAGIC, 5)
#endif
