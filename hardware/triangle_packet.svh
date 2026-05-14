// triangle_packet.svh

`ifndef TRIANGLE_PACKET_SVH
`define TRIANGLE_PACKET_SVH

typedef struct packed {
    logic [9:0] bbox_xmin;
    logic [9:0] bbox_xmax;
    logic [8:0] bbox_ymin;
    logic [8:0] bbox_ymax;
    logic signed [31:0] a0, b0;
    logic signed [31:0] a1, b1;
    logic signed [31:0] a2, b2;
    logic signed [31:0] e0_init;
    logic signed [31:0] e1_init;
    logic signed [31:0] e2_init;
    logic signed [31:0] z_at_origin;
    logic signed [31:0] z_step_x;
    logic signed [31:0] z_step_y;
    logic [7:0] color;
    logic front_facing;
} triangle_packet_t;

`endif
