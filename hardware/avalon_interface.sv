module avalon_interface (
    input  logic clk, rst,
   
    // From the AVALON bus
    input  logic [6:0]  avalon_address,
    input  logic        avalon_write,
    input  logic [31:0] avalon_writedata,
    output logic [31:0] avalon_readdata,
   
    //For the pops that your pixel unit or the dispatcher will do
    input  logic        pop,            // Dispatcher requests a packet
    output logic        pop_available,  // FIFO has valid data to give
    output triangle_packet_t pop_data,  // The packet (valid when pop_available=1)
    input  logic        pop_ACK,        // Dispatcher confirms receipt
   
    // To status registers
    output logic        fifo_full,
    output logic        fifo_empty,
    output logic [5:0]  fifo_level
);

/* 
If you want to pop a triangle you can send in a request with pop=1 once the data is ready and is valid the avalon interface will give a signal as pop_availavle=1, once your pixel unit sends in a pop-ACK signal the module will internally pop that item from the buffer and make pop_available as 0.

The mechanism is too rudimentary right now and requires 2 cycles just because of the handshake but we can change it later if we see performance issues, i feel this might slow down our hardware and might act as a bottleneck but once we have a working module both of us can sit together and figure out a way to make it all synchronous to be able to do it in 1 cycle

Below i just have a offset based register wise mapping which i will be using for myself while picking up data from the avalon bus might be useless to you but added it in just case you want to know about it.

Our module Receives writes from ARM over the Avalon-MM bus at address 0xFF200000 + offset(tyhis is the offsets of the registers below)

Offset - Register Name
0x00 - bbox_x
0x04 - bbox_y
0x08 - a0
0x0C - b0
0x10 - a1
0x14 - b1
0x18 - a2
0x1C - b2
0x20 - e0_init
0x24 - e1_init
0x28 - e2_init
0x2C - z_at_origin
0x30 - z_step_x
0x34 - z_step_y
0x38 - color_cull

Also has 2 status registers for the interface between HW and SW
0x3C - commit/status(i have yet not decided the bitwise mapping on this, will have to look at the avalon interface for this if concurrent RW permissions can be given to registers) */
