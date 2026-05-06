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

//use this in the triangle packet calcuation module later to get the fifo status from the function read_status
typedef struct {
    __u32 fifo_level;   // bits [5:0] of STATUS
    __u32 fifo_empty;   // bit  [6]   of STATUS
    __u32 fifo_full;    // bit  [7]   of STATUS
} rasterizer_status_t;

//no tneeded rigth now
typedef struct {
    __u32 irq_enable;       //1 = enable FIFO low-watermark IRQ
    __u32 low_watermark;    //IRQ fires when fifo_level < this value
} rasterizer_control_t;


typedef union {
    triangle_packet_t  packet;    //used by RASTERIZER_SUBMIT
    rasterizer_status_t  status;    //used by RASTERIZER_STATUS
    rasterizer_control_t control;   //used by RASTERIZER_SET_CONTROL
} rasterizer_arg_t;

// Offset 0x00 - 0x40 : packet words 0-16  (write-only)
// Offset 0x44         : COMMIT register   (write-only, any value)
// Offset 0x48         : STATUS register   (read-only)
// Offset 0x4C         : CONTROL register  (read-write)
#define RAST_PACKET_WORD_BASE   0x00
#define RAST_PACKET_NUM_WORDS   17
#define RAST_COMMIT_OFFSET      0x44
#define RAST_STATUS_OFFSET      0x48
#define RAST_CONTROL_OFFSET     0x4C
#define RAST_CTRL_PRESENT_BIT (1<<0)
// bit masks for the STATUS and CONTROL registers
#define RAST_STATUS_LEVEL_MASK  0x3F    // bits [5:0] = fifo level
#define RAST_STATUS_EMPTY_BIT   (1<<6)  // bit  [6]   = fifo empty
#define RAST_STATUS_FULL_BIT    (1<<7)  // bit  [7]   = fifo full

// CONTROL register bit fields
#define RAST_CTRL_IRQ_EN_BIT    (1<<0)  // bit [0] = irq enable
#define RAST_CTRL_WMARK_SHIFT   1       // bits [7:1] = watermark
#define RAST_CTRL_WMARK_MASK    0x7F

// same as  vga_ball 
#define RASTERIZER_MAGIC        'R'

// macros for ioctl commands, used in the switch statement in the handler func of the kernel module
#define RASTERIZER_SUBMIT       _IOW(RASTERIZER_MAGIC, 1, rasterizer_arg_t *)
#define RASTERIZER_STATUS       _IOR(RASTERIZER_MAGIC, 2, rasterizer_arg_t *)
#define RASTERIZER_SET_CONTROL  _IOW(RASTERIZER_MAGIC, 3, rasterizer_arg_t *)
#define RASTERIZER_GET_CONTROL  _IOR(RASTERIZER_MAGIC, 4, rasterizer_arg_t *)
#define RASTERIZER_PRESENT _IO(RASTERIZER_MAGIC, 5)
#endif /* _RASTERIZER_H */
