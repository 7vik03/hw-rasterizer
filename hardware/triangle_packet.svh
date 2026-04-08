`ifndef TRIANGLE_PACKET_SVH
`define TRIANGLE_PACKET_SVH

typedef struct packed {
    // bounding box (integer screen coords)
    logic [9:0]  bbox_xmin;   // 0-319
    logic [9:0]  bbox_xmax;
    logic [8:0]  bbox_ymin;   // 0-239
    logic [8:0]  bbox_ymax;

    // edge coefficients Q12.12 signed
    // E_i(x,y) = a_i*x + b_i*y + c_i
    // step x: E_i += a_i   step y: E_i += b_i
    logic signed [31:0] a0, b0, c0;  // edge 0: v1->v2
    logic signed [31:0] a1, b1, c1;  // edge 1: v2->v0
    logic signed [31:0] a2, b2, c2;  // edge 2: v0->v1

    // initial edge values at bounding box origin
    // evaluated at pixel center (bbox_xmin+0.5, bbox_ymin+0.5)
    logic signed [31:0] e0_init;
    logic signed [31:0] e1_init;
    logic signed [31:0] e2_init;

    // depth interpolation Q12.12 signed
    logic signed [31:0] z_at_origin; // depth at bbox origin
    logic signed [31:0] z_step_x;    // dZ per x step
    logic signed [31:0] z_step_y;    // dZ per y step

    // color and culling
    logic [7:0]  color;        // RGB332
    logic        front_facing; // 0 = cull, 1 = draw

} triangle_packet_t;

`endif