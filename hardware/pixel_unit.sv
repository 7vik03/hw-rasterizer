`include "triangle_packet.svh"

module pixel_unit #(
    parameter int Y_MIN_CLIP = 0,
    parameter int Y_MAX_CLIP = 239
) (
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

    //DONE: latch packet fields into local regs when we accept a triangle
    //right now we read packet.a0 etc every cycle while active
    //if upstream changes packet (FIFO advances to next triangle) we get corrupted data

    //latched packet fields - copied on accept so upstream fifo can move on
    logic [9:0] lat_bbox_xmin, lat_bbox_xmax;
    logic [8:0] lat_clip_ymin, lat_clip_ymax;
    logic signed [31:0] lat_a0, lat_a1, lat_a2;
    logic signed [31:0] lat_b0, lat_b1, lat_b2;
    logic signed [31:0] lat_z_step_x, lat_z_step_y;
    logic [7:0] lat_color;

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
                pixel_color <= lat_color;
                pixel_depth <= z[27:12];
            end

            if (x == lat_bbox_xmax) begin
                x <= lat_bbox_xmin;
                y <= y + 1;
                e0_row <= e0_row + lat_b0;
                e1_row <= e1_row + lat_b1;
                e2_row <= e2_row + lat_b2;
                z_row <= z_row + lat_z_step_y;
                e0 <= e0_row + lat_b0;
                e1 <= e1_row + lat_b1;
                e2 <= e2_row + lat_b2;
                z <= z_row + lat_z_step_y;
            end else begin
                x <= x + 1;
                e0 <= e0 + lat_a0;
                e1 <= e1 + lat_a1;
                e2 <= e2 + lat_a2;
                z <= z + lat_z_step_x;
            end

            if (y == lat_clip_ymax && x == lat_bbox_xmax) begin
                active <= 0;
                ready  <= 1;
            end

        end else if (valid_in && ready) begin
           if (packet.front_facing) begin
                logic [8:0] clip_ymin, clip_ymax;
                logic signed [31:0] skip;
                clip_ymin = (packet.bbox_ymin > 9'(Y_MIN_CLIP)) ? packet.bbox_ymin : 9'(Y_MIN_CLIP);
                clip_ymax = (packet.bbox_ymax < 9'(Y_MAX_CLIP)) ? packet.bbox_ymax : 9'(Y_MAX_CLIP);
                skip = 32'(clip_ymin) - 32'(packet.bbox_ymin);

                if (clip_ymin <= clip_ymax) begin
                    lat_bbox_xmin <= packet.bbox_xmin;
                    lat_bbox_xmax <= packet.bbox_xmax;
                    lat_clip_ymin <= clip_ymin;
                    lat_clip_ymax <= clip_ymax;
                    lat_a0 <= packet.a0;
                    lat_a1 <= packet.a1;
                    lat_a2 <= packet.a2;
                    lat_b0 <= packet.b0;
                    lat_b1 <= packet.b1;
                    lat_b2 <= packet.b2;
                    lat_z_step_x <= packet.z_step_x;
                    lat_z_step_y <= packet.z_step_y;
                    lat_color <= packet.color;
                    x <= packet.bbox_xmin;
                    y <= clip_ymin;
                    e0_row <= packet.e0_init + packet.b0 * skip;
                    e1_row <= packet.e1_init + packet.b1 * skip;
                    e2_row <= packet.e2_init + packet.b2 * skip;
                    e0 <= packet.e0_init + packet.b0 * skip;
                    e1 <= packet.e1_init + packet.b1 * skip;
                    e2 <= packet.e2_init + packet.b2 * skip;
                    z_row <= packet.z_at_origin + packet.z_step_y * skip;
                    z <= packet.z_at_origin + packet.z_step_y * skip;
                    active <= 1;
                    ready <= 0;
                end
           end
        end
    end
    


    //TODO: gonna need RAM for z buffer inside this unit
    //m10k has 1 cycle read latency
    //probably means adding an FSM
    //- compute addr from (x,y), read stored depth from z buffer
    //- compare read depth vs z[27:12], write if new depth < stored

    //DONE: y-range clipping for parallel pixel units
    //skip triangle entirely if clipped_ymin > clipped_ymax
    //also need to adjust e0_init/e1_init/e2_init/z_at_origin for skipped rows
    //(add b0 * (clipped_ymin - bbox_ymin) etc)

endmodule