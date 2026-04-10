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
    logic active;

    //TODO: latch packet fields into local regs when we accept a triangle
    //right now we read packet.a0 etc every cycle while active
    //if upstream changes packet (FIFO advances to next triangle) we get corrupted data

    always_ff @(posedge clk) begin
        if (rst) begin
            active <= 0;
            ready <= 1;
            pixel_valid <= 0;
        end else if (active) begin
            pixel_valid <= 0;

            if (e0 >= 0 && e1 >= 0 && e2 >= 0) begin
                pixel_valid <= 1;
                pixel_x <= x;
                pixel_y <= y;
                pixel_color <= packet.color;
                pixel_depth <= z[27:12];
            end

            e0 <= e0 + packet.a0;
            e1 <= e1 + packet.a1;
            e2 <= e2 + packet.a2;
            z <= z + packet.z_step_x;

            if (x == packet.bbox_xmax) begin
                x <= packet.bbox_xmin;
                y <= y + 1;
                e0_row <= e0_row + packet.b0;
                e1_row <= e1_row + packet.b1;
                e2_row <= e2_row + packet.b2;
                z_row <= z_row + packet.z_step_y;
                e0 <= e0_row + packet.b0;
                e1 <= e1_row + packet.b1;
                e2 <= e2_row + packet.b2;
                z <= z_row + packet.z_step_y;
            end else begin
                x <= x + 1;
            end

            if (y == packet.bbox_ymax && x == packet.bbox_xmax) begin
                active <= 0;
                ready <= 1;
                //remove pixel_valid <= 0 here bc it kills the
                //edge test on the final pixel (and 1x1 bbox)
                //should get cleared at top of active block so shoudl be good?
                //pixel_valid <= 0;
            end
        end else if (valid_in && ready) begin
            if (packet.front_facing) begin
                x <= packet.bbox_xmin;
                y <= packet.bbox_ymin;
                e0_row <= packet.e0_init;
                e1_row <= packet.e1_init;
                e2_row <= packet.e2_init;
                e0 <= packet.e0_init;
                e1 <= packet.e1_init;
                e2 <= packet.e2_init;
                z_row <= packet.z_at_origin;
                z <= packet.z_at_origin;
                active <= 1;
                ready <= 0;
            end
        end
    end


    //TODO: gonna need RAM for z buffer inside this unit
    //m10k has 1 cycle read latency
    //probably means adding an FSM
    //- compute addr from (x,y), read stored depth from z buffer
    //- compare read depth vs z[27:12], write if new depth < stored

    //TODO: y-range clipping for parallel pixel units
    //skip triangle entirely if clipped_ymin > clipped_ymax
    //also need to adjust e0_init/e1_init/e2_init/z_at_origin for skipped rows
    //(add b0 * (clipped_ymin - bbox_ymin) etc)

endmodule