`include "triangle_packet.svh"

/* One node of the 16-PU systolic chain.

Each instance is hard-bound to one of 16 column banks of the 256x240
internal frame buffer: PU `PU_ID` owns screen columns whose lower 4 bits
equals PU_ID (cols PU_ID, PU_ID+16, PU_ID+32, ..., PU_ID+240). The
dispatcher is responsible for aligning bbox_xmin to a 16-pixel boundary
so that col_base_in[3:0] == PU_ID; with that invariant the PU's local
memory addresses naturally pack into a 12-bit space (4 col-bank bits |
8 row bits).

Seed handoff (the systolic part):
    cycle K   PU_i in IDLE, sees seed_valid_in. Latches accumulators and
              constants. On the same edge registers (seed + a) onto
              seed_*_out and pulses seed_valid_out high. This is what
              keeps the chain at a 1-cycle stagger: PU_(i+1) sees its
              seed at cycle K+1, one column to the right of PU_i.
    cycle K+1 PU_i is in ACTIVE for the first time, computes the inside
              test for (col_base, row_base), and walks +b every cycle
              thereafter until cur_row == last_row, then back to IDLE.

The (seed_in + a_in) computation lives on the same clock edge as the
IDLE->ACTIVE latch. Since the input is registered upstream and the
output is registered locally, the chain has exactly one 32-bit add
between flops -- no cascaded combinational adders across PUs.

Read-modify-write z-test is pipelined across two cycles using the M10K's
two ports: cycle N issues a synchronous read at addr_now and queues the
candidate (color, depth, row, inside-flag); cycle N+1 the read result is
available, the z compare runs, and the conditional write fires on the
other port.

Multi-column iteration: a PU is responsible for *every* screen column
whose low nibble equals PU_ID, not just one column per triangle. After
the first column finishes (cur_row == last_row) the PU jumps 16 columns
to the right, resets cur_row, and walks again. This continues until
col_base + 16 would step past last_col, at which point the PU returns
to IDLE. The systolic seed is only consumed for the first column;
subsequent columns are derived locally by adding 16*a to the column-top
edge values stored at the IDLE->ACTIVE latch.

Frame clear:
   S_INIT_CLEAR runs once after rst (DO_INIT_CLEAR=1) to put z_mem at
   16'hFFFF and BOTH color framebuffers at 8'h00, so the first VGA scan
   doesn't display BRAM power-up garbage. S_CLEAR is entered on each
   z_clear_start pulse (one per frame swap, driven by rasterizer_top)
   and writes z_mem and the *current back* color framebuffer (selected
   by fb_write_sel) to those same sentinels. Both states walk all
   Z_DEPTH addresses with a 12-bit clear_addr counter; ready stays low
   for the duration so the dispatcher holds off on new triangles.

   The z-test still uses `<=` so two pixels with identical depth both
   write (second one wins) -- harmless on coincident depths. */

module pixel_unit #(
    parameter int  PU_ID = 0,
    parameter bit  IS_LAST_PU = 1'b0,
    parameter int  FB_DEPTH = 4096,
    parameter int  Z_DEPTH = 4096,
    parameter int  Z_MSB = 27,
    parameter int  Z_LSB = 12,
    parameter bit  DO_INIT_CLEAR = 1'b1
) (
    input logic clk,
    input logic rst,
    output logic ready,
    input logic z_clear_start,
    input logic seed_valid_in,
    input logic signed [31:0] seed_e0_in,
    input logic signed [31:0] seed_e1_in,
    input logic signed [31:0] seed_e2_in,
    input logic signed [31:0] seed_z_in,
    input logic signed [31:0] a0_in, a1_in, a2_in, z_step_x_in,
    input logic signed [31:0] b0_in, b1_in, b2_in, z_step_y_in,
    input logic [7:0] color_in,
    input logic [7:0] col_base_in,
    input logic [7:0] row_base_in,
    input logic [7:0] last_row_in,
    input logic [7:0] last_col_in,  
    output logic seed_valid_out,
    output logic signed [31:0] seed_e0_out,
    output logic signed [31:0] seed_e1_out,
    output logic signed [31:0] seed_e2_out,
    output logic signed [31:0] seed_z_out,
    output logic p_write,
    output logic [7:0] p_col,
    output logic [7:0] p_row,
    output logic [7:0] p_color,
    output logic [15:0] p_depth,
    input logic [11:0] vga_r_addr,
    input logic vga_r_buf_sel,
    output logic [7:0] vga_r_data,
    input logic fb_write_sel
);

    typedef enum logic [1:0] {
        S_INIT_CLEAR,  
        S_IDLE,       
        S_ACTIVE,     
        S_CLEAR        
    } state_t;
    state_t state;

    logic [11:0] clear_addr;
    logic clear_done;
    assign clear_done = (clear_addr == 12'(Z_DEPTH - 1));
    logic signed [31:0] a0_q, a1_q, a2_q, z_step_x_q;
    logic signed [31:0] b0_q, b1_q, b2_q, z_step_y_q;
    logic [7:0] color_q, col_base_q, row_base_q, last_row_q;
    logic [7:0] last_col_q;

    logic signed [31:0] e0, e1, e2, z;
    logic [7:0] cur_row;
    logic signed [31:0] e0_col_top, e1_col_top, e2_col_top, z_col_top;

    assign ready = (state == S_IDLE);


    logic inside_now;
    assign inside_now = (state == S_ACTIVE) && (e0[31] == 1'b0) && (e1[31] == 1'b0) && (e2[31] == 1'b0);

    logic [11:0] addr_now;
    assign addr_now = {col_base_q[7:4], cur_row};

    logic q_valid;
    logic q_inside;
    logic [11:0] q_addr;
    logic [7:0] q_col, q_row, q_color;
    logic [15:0] q_z;


    (* ramstyle = "M10K", max_depth = 512 *)
    logic [15:0] z_mem [Z_DEPTH];
    logic [15:0] z_rd;

   
    (* ramstyle = "M10K", max_depth = 1024 *)
    logic [7:0] fb_a_mem [FB_DEPTH];
    (* ramstyle = "M10K", max_depth = 1024 *)
    logic [7:0] fb_b_mem [FB_DEPTH];

    logic [7:0] fb_a_rd, fb_b_rd;


    logic z_pass;
    assign z_pass = q_valid && q_inside && (q_z <= z_rd);

    always_ff @(posedge clk) begin
        seed_valid_out <= 1'b0;
        p_write <= 1'b0;
        q_valid <= 1'b0;

        if (rst) begin
            state <= DO_INIT_CLEAR ? S_INIT_CLEAR : S_IDLE;
            cur_row <= '0;
            clear_addr <= '0;
        end else begin
            unique case (state)
                S_INIT_CLEAR: begin
                    if (clear_done) begin
                        state <= S_IDLE;
                        clear_addr <= '0;
                    end else begin
                        clear_addr <= clear_addr + 12'd1;
                    end
                end

                S_IDLE: begin
                    if (z_clear_start) begin
                        state <= S_CLEAR;
                        clear_addr <= '0;
                    end else if (seed_valid_in) begin
                        e0 <= seed_e0_in;
                        e1 <= seed_e1_in;
                        e2 <= seed_e2_in;
                        z <= seed_z_in;
                        e0_col_top <= seed_e0_in;
                        e1_col_top <= seed_e1_in;
                        e2_col_top <= seed_e2_in;
                        z_col_top  <= seed_z_in;
                        a0_q <= a0_in;
                        a1_q <= a1_in;
                        a2_q <= a2_in;
                        z_step_x_q <= z_step_x_in;
                        b0_q <= b0_in;
                        b1_q <= b1_in;
                        b2_q <= b2_in;
                        z_step_y_q <= z_step_y_in;
                        color_q <= color_in;
                        col_base_q <= col_base_in;
                        row_base_q <= row_base_in;
                        last_row_q <= last_row_in;
                        last_col_q <= last_col_in;
                        cur_row <= row_base_in;
                        state <= S_ACTIVE;

                        if (!IS_LAST_PU) begin
                            seed_valid_out <= 1'b1;
                            seed_e0_out <= seed_e0_in + a0_in;
                            seed_e1_out <= seed_e1_in + a1_in;
                            seed_e2_out <= seed_e2_in + a2_in;
                            seed_z_out <= seed_z_in + z_step_x_in;
                        end
                    end
                end

                S_ACTIVE: begin
                    q_valid <= 1'b1;
                    q_inside <= inside_now;
                    q_addr <= addr_now;
                    q_col <= col_base_q;
                    q_row <= cur_row;
                    q_color <= color_q;
                    q_z <= z[Z_MSB:Z_LSB];
                    e0 <= e0 + b0_q;
                    e1 <= e1 + b1_q;
                    e2 <= e2 + b2_q;
                    z <= z + z_step_y_q;
                    cur_row <= cur_row + 8'd1;

                    if (cur_row == last_row_q) begin
                        if (({1'b0, col_base_q} + 9'd16) > {1'b0, last_col_q}) begin
                            state <= S_IDLE;
                        end else begin
                            e0 <= e0_col_top + (a0_q <<< 4);
                            e1 <= e1_col_top + (a1_q <<< 4);
                            e2 <= e2_col_top + (a2_q <<< 4);
                            z <= z_col_top + (z_step_x_q <<< 4);
                            e0_col_top <= e0_col_top + (a0_q <<< 4);
                            e1_col_top <= e1_col_top + (a1_q <<< 4);
                            e2_col_top <= e2_col_top + (a2_q <<< 4);
                            z_col_top <= z_col_top + (z_step_x_q <<< 4);
                            col_base_q <= col_base_q + 8'd16;
                            cur_row <= row_base_q;
                        end
                    end
                end

                S_CLEAR: begin
                    if (clear_done) begin
                        state <= S_IDLE;
                        clear_addr <= '0;
                    end else begin
                        clear_addr <= clear_addr + 12'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end

        z_rd <= z_mem[addr_now];

        if (state == S_INIT_CLEAR) begin
            z_mem [clear_addr] <= 16'hFFFF;
            fb_a_mem[clear_addr] <= 8'h00;
            fb_b_mem[clear_addr] <= 8'h00;
        end else if (state == S_CLEAR) begin
            z_mem[clear_addr] <= 16'hFFFF;
            if (fb_write_sel) fb_b_mem[clear_addr] <= 8'h00;
            else fb_a_mem[clear_addr] <= 8'h00;
        end else if (z_pass) begin
            z_mem[q_addr] <= q_z;
            if (fb_write_sel) fb_b_mem[q_addr] <= q_color;
            else fb_a_mem[q_addr] <= q_color;
            p_write <= 1'b1;
            p_col <= q_col;
            p_row <= q_row;
            p_color <= q_color;
            p_depth <= q_z;
        end

        fb_a_rd <= fb_a_mem[vga_r_addr];
        fb_b_rd <= fb_b_mem[vga_r_addr];
    end

    assign vga_r_data = vga_r_buf_sel ? fb_b_rd : fb_a_rd;

    // ---- simulation-only sanity check ----
    `ifndef SYNTHESIS
    always @(posedge clk) begin
        if (!rst && state == S_IDLE && seed_valid_in) begin
            assert (col_base_in[3:0] == PU_ID[3:0])
                else $error("pixel_unit PU_ID=%0d got col_base=%0d (low nibble %0d, expected %0d)",
                            PU_ID, col_base_in, col_base_in[3:0], PU_ID[3:0]);
        end
    end
    `endif

endmodule
