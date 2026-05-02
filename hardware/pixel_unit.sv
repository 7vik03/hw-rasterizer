`include "triangle_packet.svh"

// One node of the 16-PU systolic chain.
//
// Each instance is hard-bound to one of 16 column banks of the 256x240
// internal framebuffer: PU `PU_ID` owns screen columns whose low nibble
// equals PU_ID (cols PU_ID, PU_ID+16, PU_ID+32, ..., PU_ID+240). The
// dispatcher is responsible for aligning bbox_xmin to a 16-pixel boundary
// so that col_base_in[3:0] == PU_ID; with that invariant the PU's local
// memory addresses naturally pack into a 12-bit space (4 col-bank bits |
// 8 row bits).
//
// Seed handoff (the systolic part):
//   cycle K   PU_i in IDLE, sees seed_valid_in. Latches accumulators and
//             constants. *On the same edge* registers (seed + a) onto
//             seed_*_out and pulses seed_valid_out high. This is what
//             keeps the chain at a 1-cycle stagger: PU_(i+1) sees its
//             seed at cycle K+1, one column to the right of PU_i.
//   cycle K+1 PU_i is in ACTIVE for the first time, computes the inside
//             test for (col_base, row_base), and walks +b every cycle
//             thereafter until cur_row == last_row, then back to IDLE.
//
// The (seed_in + a_in) computation lives on the same clock edge as the
// IDLE->ACTIVE latch. Since the input is registered upstream and the
// output is registered locally, the chain has exactly one 32-bit add
// between flops -- no cascaded combinational adders across PUs.
//
// Read-modify-write z-test is pipelined across two cycles using the M10K's
// two ports: cycle N issues a synchronous read at addr_now and queues the
// candidate (color, depth, row, inside-flag); cycle N+1 the read result is
// available, the z compare runs, and the conditional write fires on the
// other port.
//
// Multi-column iteration: a PU is responsible for *every* screen column
// whose low nibble equals PU_ID, not just one column per triangle. After
// the first column finishes (cur_row == last_row) the PU jumps 16 columns
// to the right, resets cur_row, and walks again. This continues until
// col_base + 16 would step past last_col, at which point the PU returns
// to IDLE. The systolic seed is only consumed for the first column;
// subsequent columns are derived locally by adding 16*a to the column-top
// edge values stored at the IDLE->ACTIVE latch.
//
// Z-buffer init: z_mem powers up to 0. The z-test is `q_z <= z_rd`
// (less-than-OR-EQUAL) so the very first write to any address always
// succeeds even if z_rd reads 0. The minor cost is that two pixels with
// identical depth both write (the second one wins), which is fine for
// our use case -- z-fighting on coincident depths is invisible. This
// avoids needing an explicit z-buffer clear pass at the start of each
// frame.

module pixel_unit #(
    parameter int  PU_ID       = 0,
    parameter bit  IS_LAST_PU  = 1'b0,
    parameter int  FB_DEPTH    = 4096,
    parameter int  Z_DEPTH     = 4096,
    parameter int  Z_MSB       = 15,
    parameter int  Z_LSB       = 0
) (
    input  logic               clk,
    input  logic               rst,

    // high while in IDLE; the dispatcher uses an AND-reduction across all
    // 16 PUs to know the chain has fully drained before issuing the next
    // triangle.
    output logic               ready,

    // ---- systolic seed in (from previous PU, or from dispatcher for PU0) ----
    input  logic               seed_valid_in,
    input  logic signed [31:0] seed_e0_in,
    input  logic signed [31:0] seed_e1_in,
    input  logic signed [31:0] seed_e2_in,
    input  logic signed [31:0] seed_z_in,

    // ---- broadcast triangle constants (must be stable across the whole
    //      triangle; the dispatcher latches them so this is automatic) ----
    input  logic signed [31:0] a0_in, a1_in, a2_in, z_step_x_in,
    input  logic signed [31:0] b0_in, b1_in, b2_in, z_step_y_in,
    input  logic [7:0]         color_in,
    input  logic [7:0]         col_base_in,
    input  logic [7:0]         row_base_in,
    input  logic [7:0]         last_row_in,
    input  logic [7:0]         last_col_in,    // bbox_xmax; controls multi-col stop

    // ---- systolic seed out (to next PU; tied 0 when IS_LAST_PU) ----
    output logic               seed_valid_out,
    output logic signed [31:0] seed_e0_out,
    output logic signed [31:0] seed_e1_out,
    output logic signed [31:0] seed_e2_out,
    output logic signed [31:0] seed_z_out,

    // ---- observability of the actual write port (post z-test) ----
    output logic               p_write,
    output logic [7:0]         p_col,
    output logic [7:0]         p_row,
    output logic [7:0]         p_color,
    output logic [15:0]        p_depth,

    // ---- VGA read port: 12-bit local addr; vga_r_buf_sel picks which
    //      framebuffer is being scanned out (the one not being written). ----
    input  logic [11:0]        vga_r_addr,
    input  logic               vga_r_buf_sel,
    output logic [7:0]         vga_r_data,

    // 0 -> rasterizer writes go to FB_A; 1 -> writes go to FB_B
    input  logic               fb_write_sel
);

    typedef enum logic { S_IDLE, S_ACTIVE } state_t;
    state_t state;

    // ---- latched per-triangle constants ----
    logic signed [31:0] a0_q, a1_q, a2_q, z_step_x_q;
    logic signed [31:0] b0_q, b1_q, b2_q, z_step_y_q;
    logic [7:0]         color_q, col_base_q, row_base_q, last_row_q;
    logic [7:0]         last_col_q;

    // ---- walk accumulators, advanced once per ACTIVE cycle ----
    logic signed [31:0] e0, e1, e2, z;
    logic [7:0]         cur_row;

    // ---- column-top anchors: edge/depth values at (col_base, row_base) for
    //      the column currently being walked. When we jump 16 columns right
    //      we can't just add 16*a to live e0/e1/e2/z (those have walked +b
    //      for many cycles), so we keep these snapshots that only update
    //      when we move to a new column.
    logic signed [31:0] e0_col_top, e1_col_top, e2_col_top, z_col_top;

    assign ready = (state == S_IDLE);

    // Pineda inside-test: a pixel is in the triangle iff all three edge
    // functions are >= 0. We test the sign bit so the inference is one
    // 3-input AND of inverted MSBs, no full comparators.
    logic inside_now;
    assign inside_now = (state == S_ACTIVE)
                      && (e0[31] == 1'b0)
                      && (e1[31] == 1'b0)
                      && (e2[31] == 1'b0);

    // 12-bit local memory address: high nibble of screen-x picks the
    // column bank inside this PU; low byte is the row.
    logic [11:0] addr_now;
    assign addr_now = {col_base_q[7:4], cur_row};

    // ---- read pipeline registers (cycle N -> cycle N+1) ----
    logic        q_valid;
    logic        q_inside;
    logic [11:0] q_addr;
    logic [7:0]  q_col, q_row, q_color;
    logic [15:0] q_z;

    // ---- Z-buffer: 16-bit x 4096, single buffered ----
    // (* ramstyle = "M10K", max_depth = 512 *) forces Quartus to a 16x512
    // configuration, so the 4096-deep array is implemented as 8 chained
    // M10Ks instead of one wide LUT-RAM block.
    (* ramstyle = "M10K", max_depth = 512 *)
    logic [15:0] z_mem [Z_DEPTH];

    logic [15:0] z_rd;

    // ---- color framebuffers: 8-bit x 4096, double buffered ----
    // 8x1024 inference -> 4 chained M10Ks per buffer (8 total per PU FB pair).
    (* ramstyle = "M10K", max_depth = 1024 *)
    logic [7:0]  fb_a_mem [FB_DEPTH];
    (* ramstyle = "M10K", max_depth = 1024 *)
    logic [7:0]  fb_b_mem [FB_DEPTH];

    logic [7:0]  fb_a_rd, fb_b_rd;

    // smaller-depth wins (Q12.12 z, low-bits-up). Combinational so the
    // write decision lands on the same edge as the FB write.
    //
    // We use <= rather than < so the first write to any address always
    // succeeds despite z_mem powering up to 0. See the file header for
    // why this is safe.
    logic z_pass;
    assign z_pass = q_valid && q_inside && (q_z <= z_rd);

    always_ff @(posedge clk) begin
        // sequential defaults so we don't latch pulse signals
        seed_valid_out <= 1'b0;
        p_write        <= 1'b0;
        q_valid        <= 1'b0;

        if (rst) begin
            state   <= S_IDLE;
            cur_row <= '0;
        end else begin
            unique case (state)
                S_IDLE: begin
                    if (seed_valid_in) begin
                        // latch seed accumulators (live walk)
                        e0 <= seed_e0_in;
                        e1 <= seed_e1_in;
                        e2 <= seed_e2_in;
                        z  <= seed_z_in;

                        // also snapshot the column-top values; the live
                        // accumulators will diverge as we walk +b, so we
                        // need a clean reference point for the +16*a jump
                        // between columns.
                        e0_col_top <= seed_e0_in;
                        e1_col_top <= seed_e1_in;
                        e2_col_top <= seed_e2_in;
                        z_col_top  <= seed_z_in;

                        // latch triangle constants
                        a0_q       <= a0_in;
                        a1_q       <= a1_in;
                        a2_q       <= a2_in;
                        z_step_x_q <= z_step_x_in;
                        b0_q       <= b0_in;
                        b1_q       <= b1_in;
                        b2_q       <= b2_in;
                        z_step_y_q <= z_step_y_in;
                        color_q    <= color_in;
                        col_base_q <= col_base_in;
                        row_base_q <= row_base_in;
                        last_row_q <= last_row_in;
                        last_col_q <= last_col_in;

                        cur_row <= row_base_in;
                        state   <= S_ACTIVE;

                        // Forward (seed + a) to the next PU on the same
                        // edge as the latch. Both inputs are registered
                        // upstream so this is a single 32-bit add per PU,
                        // not a cascaded combinational chain.
                        if (!IS_LAST_PU) begin
                            seed_valid_out <= 1'b1;
                            seed_e0_out    <= seed_e0_in + a0_in;
                            seed_e1_out    <= seed_e1_in + a1_in;
                            seed_e2_out    <= seed_e2_in + a2_in;
                            seed_z_out     <= seed_z_in  + z_step_x_in;
                        end
                    end
                end

                S_ACTIVE: begin
                    // queue this cycle's candidate for next-cycle z-test
                    q_valid  <= 1'b1;
                    q_inside <= inside_now;
                    q_addr   <= addr_now;
                    q_col    <= col_base_q;
                    q_row    <= cur_row;
                    q_color  <= color_q;
                    q_z      <= z[Z_MSB:Z_LSB];

                    // walk one row down (default; may be overridden below
                    // on a column-jump cycle since later NBAs win)
                    e0      <= e0 + b0_q;
                    e1      <= e1 + b1_q;
                    e2      <= e2 + b2_q;
                    z       <= z  + z_step_y_q;
                    cur_row <= cur_row + 8'd1;

                    if (cur_row == last_row_q) begin
                        // End of current column. Stop AFTER walking
                        // col_base_q if jumping by +16 would land past
                        // last_col_q. The 9-bit compare prevents wrap
                        // when col_base_q is near 8'hFF.
                        //
                        // Example: PU0 with last_col=255 walks columns
                        // 0,16,...,240 then stops (since 240+16 > 255).
                        // PU0 with last_col=239 walks 0,16,...,224 then
                        // stops (since 224+16 > 239 and the next bank
                        // PU0 owns would be column 240, outside bbox).
                        if (({1'b0, col_base_q} + 9'd16) > {1'b0, last_col_q}) begin
                            state <= S_IDLE;
                        end else begin
                            // Jump 16 columns right. Reset row to top and
                            // recompute the live e/z from the column-top
                            // anchors plus 16*a. Both the live regs and the
                            // anchors advance, so the next jump (if any)
                            // builds on the new top values.
                            e0         <= e0_col_top + (a0_q       <<< 4);
                            e1         <= e1_col_top + (a1_q       <<< 4);
                            e2         <= e2_col_top + (a2_q       <<< 4);
                            z          <= z_col_top  + (z_step_x_q <<< 4);
                            e0_col_top <= e0_col_top + (a0_q       <<< 4);
                            e1_col_top <= e1_col_top + (a1_q       <<< 4);
                            e2_col_top <= e2_col_top + (a2_q       <<< 4);
                            z_col_top  <= z_col_top  + (z_step_x_q <<< 4);

                            col_base_q <= col_base_q + 8'd16;
                            cur_row    <= row_base_q;
                            // stay in S_ACTIVE
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end

        // ---- Z-buffer port A: synchronous read every cycle ----
        z_rd <= z_mem[addr_now];
        // ---- Z-buffer port B: conditional write at cycle N+1 ----
        if (z_pass) begin
            z_mem[q_addr] <= q_z;
        end

        // ---- color framebuffer write (same condition as z) ----
        if (z_pass) begin
            if (fb_write_sel) fb_b_mem[q_addr] <= q_color;
            else              fb_a_mem[q_addr] <= q_color;
            p_write <= 1'b1;
            p_col   <= q_col;
            p_row   <= q_row;
            p_color <= q_color;
            p_depth <= q_z;
        end

        // ---- VGA read ports (independent of rasterizer port) ----
        fb_a_rd <= fb_a_mem[vga_r_addr];
        fb_b_rd <= fb_b_mem[vga_r_addr];
    end

    assign vga_r_data = vga_r_buf_sel ? fb_b_rd : fb_a_rd;

    // ---- simulation-only sanity check ----
    // verify the dispatcher is feeding this PU a col_base whose low
    // nibble matches PU_ID. if this fires, the bbox_xmin alignment
    // invariant in software was violated and memory writes will land
    // in the wrong bank.
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