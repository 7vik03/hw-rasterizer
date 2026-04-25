`include "triangle_packet.svh"

module pixel_unit_pipeline #(
    parameter int Y_MIN_CLIP = 0,
    parameter int Y_MAX_CLIP = 239,
    parameter int STAGES = 4
) (
    input  logic clk,
    input  logic rst,
    input  logic valid_in,
    input  triangle_packet_t packet,
    output logic ready
);

    // ─────────────────────────────────────────────
    // Pipeline registers (one slot per stage)
    // ─────────────────────────────────────────────
    logic signed [31:0] e0_pipe    [0:STAGES-1];
    logic signed [31:0] e1_pipe    [0:STAGES-1];
    logic signed [31:0] e2_pipe    [0:STAGES-1];
    logic signed [31:0] z_pipe     [0:STAGES-1];
    logic        [9:0]  x_pipe     [0:STAGES-1];
    logic        [8:0]  y_pipe     [0:STAGES-1];
    logic        [7:0]  color_pipe [0:STAGES-1];
    logic               valid_pipe [0:STAGES-1];
    logic               inside_pipe[0:STAGES-1];

    // ─────────────────────────────────────────────
    // Latched packet fields (same as your current design)
    // ─────────────────────────────────────────────
    logic signed [31:0] lat_a0, lat_a1, lat_a2;
    logic signed [31:0] lat_b0, lat_b1, lat_b2;
    logic signed [31:0] lat_z_step_x, lat_z_step_y;
    logic        [9:0]  lat_bbox_xmin, lat_bbox_xmax;
    logic        [8:0]  lat_clip_ymin, lat_clip_ymax;
    logic        [7:0]  lat_color;

    // ─────────────────────────────────────────────
    // Row tracking — one row seeded per cycle
    // ─────────────────────────────────────────────
    logic signed [31:0] e0_row, e1_row, e2_row, z_row;
    logic        [8:0]  current_row;
    logic               active;

    // ─────────────────────────────────────────────
    // Z-buffer (one per stage — private M10K slice)
    // ─────────────────────────────────────────────
    localparam int Z_MSB = 15;
    localparam int Z_LSB = 0;

    logic [16:0] z_rd_addr [0:STAGES-1];
    logic [15:0] z_rd_data [0:STAGES-1];
    logic [16:0] z_wr_addr [0:STAGES-1];
    logic [15:0] z_wr_data [0:STAGES-1];
    logic        z_wr_en   [0:STAGES-1];

    // One Z-buffer slice per stage
    // Each owns every STAGES-th row starting at its stage index
    generate
        genvar s;
        for (s = 0; s < STAGES; s++) begin : zbuf_slice
            // rows owned: s, s+STAGES, s+2*STAGES...
            // size: ceil(240/STAGES) rows × 320 cols
            localparam int SLICE_ROWS = (240 + STAGES - 1) / STAGES;
            localparam int SLICE_SIZE = 320 * SLICE_ROWS;

            logic [15:0] z_mem [0:SLICE_SIZE-1];

            // Read port (registered — M10K 1-cycle latency)
            always_ff @(posedge clk)
                z_rd_data[s] <= z_mem[z_rd_addr[s]];

            // Write port
            always_ff @(posedge clk)
                if (z_wr_en[s])
                    z_mem[z_wr_addr[s]] <= z_wr_data[s];
        end
    endgenerate

    // ─────────────────────────────────────────────
    // Framebuffer write ports (one per stage)
    // Connected externally to per-stage FB instances
    // ─────────────────────────────────────────────
    output logic        fb_wr_en    [0:STAGES-1];
    output logic [9:0]  fb_wr_x     [0:STAGES-1];
    output logic [8:0]  fb_wr_y     [0:STAGES-1];
    output logic [7:0]  fb_wr_color [0:STAGES-1];

    // ─────────────────────────────────────────────
    // Stage 0: Accept triangle, seed new row each cycle
    // ─────────────────────────────────────────────
    always_ff @(posedge clk) begin
        if (rst) begin
            active      <= 0;
            ready       <= 1;
            valid_pipe[0] <= 0;
        end else if (active) begin
            // Seed stage 0 with current row start values
            e0_pipe[0]    <= e0_row;
            e1_pipe[0]    <= e1_row;
            e2_pipe[0]    <= e2_row;
            z_pipe[0]     <= z_row;
            x_pipe[0]     <= lat_bbox_xmin;
            y_pipe[0]     <= current_row;
            color_pipe[0] <= lat_color;
            valid_pipe[0] <= 1;

            // Advance to next row
            e0_row      <= e0_row + lat_b0;
            e1_row      <= e1_row + lat_b1;
            e2_row      <= e2_row + lat_b2;
            z_row       <= z_row  + lat_z_step_y;
            current_row <= current_row + 1;

            // Done when all rows seeded
            if (current_row == lat_clip_ymax) begin
                active        <= 0;
                ready         <= 1;
                valid_pipe[0] <= 0;
            end

        end else if (valid_in && ready && packet.front_facing) begin
            // Accept triangle — same skip logic as your current design
            logic [8:0] clip_ymin, clip_ymax;
            logic signed [9:0] skip;

            clip_ymin = (packet.bbox_ymin > 9'(Y_MIN_CLIP)) ?
                         packet.bbox_ymin : 9'(Y_MIN_CLIP);
            clip_ymax = (packet.bbox_ymax < 9'(Y_MAX_CLIP)) ?
                         packet.bbox_ymax : 9'(Y_MAX_CLIP);
            skip = $signed({1'b0, clip_ymin}) -
                   $signed({1'b0, packet.bbox_ymin});

            if (clip_ymin <= clip_ymax) begin
                // Latch packet fields
                lat_a0        <= packet.a0;
                lat_a1        <= packet.a1;
                lat_a2        <= packet.a2;
                lat_b0        <= packet.b0;
                lat_b1        <= packet.b1;
                lat_b2        <= packet.b2;
                lat_z_step_x  <= packet.z_step_x;
                lat_z_step_y  <= packet.z_step_y;
                lat_bbox_xmin <= packet.bbox_xmin;
                lat_bbox_xmax <= packet.bbox_xmax;
                lat_clip_ymin <= clip_ymin;
                lat_clip_ymax <= clip_ymax;
                lat_color     <= packet.color;

                // Seed first row with skip adjustment
                e0_row      <= packet.e0_init + packet.b0 * skip;
                e1_row      <= packet.e1_init + packet.b1 * skip;
                e2_row      <= packet.e2_init + packet.b2 * skip;
                z_row       <= packet.z_at_origin + packet.z_step_y * skip;
                current_row <= clip_ymin;

                active <= 1;
                ready  <= 0;
            end
        end
    end

    // ─────────────────────────────────────────────
    // Stages 1 to STAGES-1: shift pipeline forward
    // Each stage advances x by 1 (adds a coefficients)
    // ─────────────────────────────────────────────
    always_ff @(posedge clk) begin
        for (int i = 1; i < STAGES; i++) begin
            e0_pipe[i]    <= e0_pipe[i-1] + lat_a0;
            e1_pipe[i]    <= e1_pipe[i-1] + lat_a1;
            e2_pipe[i]    <= e2_pipe[i-1] + lat_a2;
            z_pipe[i]     <= z_pipe[i-1]  + lat_z_step_x;
            x_pipe[i]     <= x_pipe[i-1]  + 1;
            y_pipe[i]     <= y_pipe[i-1];
            color_pipe[i] <= color_pipe[i-1];
            valid_pipe[i] <= valid_pipe[i-1];
        end
    end

    // ─────────────────────────────────────────────
    // Inside check — combinational, per stage
    // ─────────────────────────────────────────────
    generate
        genvar g;
        for (g = 0; g < STAGES; g++) begin : inside_check
            assign inside_pipe[g] = valid_pipe[g]     &&
                                    (e0_pipe[g] >= 0) &&
                                    (e1_pipe[g] >= 0) &&
                                    (e2_pipe[g] >= 0);

            // Issue Z-buffer read for inside pixels
            assign z_rd_addr[g] = inside_pipe[g] ?
                                  (y_pipe[g] * 320 + x_pipe[g]) : '0;
        end
    endgenerate

    // ─────────────────────────────────────────────
    // Z compare and write — one cycle after Z read
    // Need one more set of pipeline registers to
    // hold (x, y, z, color, inside) while Z read resolves
    // ─────────────────────────────────────────────
    logic        [9:0]  x_zdelay     [0:STAGES-1];
    logic        [8:0]  y_zdelay     [0:STAGES-1];
    logic signed [31:0] z_zdelay     [0:STAGES-1];
    logic        [7:0]  color_zdelay [0:STAGES-1];
    logic               inside_zdelay[0:STAGES-1];

    always_ff @(posedge clk) begin
        for (int i = 0; i < STAGES; i++) begin
            x_zdelay[i]      <= x_pipe[i];
            y_zdelay[i]      <= y_pipe[i];
            z_zdelay[i]      <= z_pipe[i];
            color_zdelay[i]  <= color_pipe[i];
            inside_zdelay[i] <= inside_pipe[i];
        end
    end

    // Z compare and conditional write — per stage
    generate
        genvar z;
        for (z = 0; z < STAGES; z++) begin : z_compare
            logic z_pass;
            assign z_pass = inside_zdelay[z] &&
                           (z_zdelay[z][Z_MSB:Z_LSB] < z_rd_data[z]);

            // Z-buffer write
            assign z_wr_en[z]   = z_pass;
            assign z_wr_addr[z] = y_zdelay[z] * 320 + x_zdelay[z];
            assign z_wr_data[z] = z_zdelay[z][Z_MSB:Z_LSB];

            // Framebuffer write
            assign fb_wr_en[z]    = z_pass;
            assign fb_wr_x[z]     = x_zdelay[z];
            assign fb_wr_y[z]     = y_zdelay[z];
            assign fb_wr_color[z] = color_zdelay[z];
        end
    endgenerate

endmodule
