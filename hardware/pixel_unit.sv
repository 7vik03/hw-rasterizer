`include "triangle_packet.svh"

module pixel_unit (
    input logic clk,
    input logic rst,
    input logic valid_in,
    input triangle_packet_t packet,
    output logic pixel_valid,
    output logic [9:0] pixel_x,
    output logic [8:0] pixel_y,
    output logic [7:0] pixel_color,
    output logic [15:0] pixel_depth,
    output logic ready
);

    logic [9:0] x;
    logic [8:0] y;
    logic signed [31:0] e0, e1, e2;
    logic signed [31:0] e0_row, e1_row, e2_row;
    logic signed [31:0] z, z_row;
    logic active; //for if we are currently rasterizing something

    always_ff @(posedge clk) begin
        if (rst) begin
            active <= 0;
            ready <= 1;
            pixel_valid <= 0;
        end else if (active) begin
            //step x
            //check inside test
            //if inside: output pixel
            //if end of row: step y, reset x, update row edge values
            //if end of bbox: done, go idle
        end else if (valid_in && ready) begin
            //new triangle arrived
            //check front_facing, if not skip
            //load bbox, initialize e0_row/e1_row/e2_row from e*_init
            //initialize z_row from z_at_origin
            //set x = bbox_xmin, y = bbox_ymin
            //set active = 1, ready = 0
        end
    end

endmodule