// systolic_chain_tb.sv
// Verifies the systolic seed handoff and steady-state pixel emission of a
// short 4-PU chain (a downscaled stand-in for the full 16-PU design).
//
// Coverage:
//   - Seed propagates from PU0 through PU3 with 1-cycle stagger.
//   - Each PU eventually emits exactly N_ROWS pixels for its column.
//   - Once the chain is full, all 4 PUs write in the same cycle on a
//     diagonal: in the cycle where PU(N-1) emits row 0, PU(N-2) is on
//     row 1, ..., PU0 is on row N-1.
//   - PU3 (IS_LAST_PU) never asserts seed_valid_out.

`timescale 1ns/1ps
`include "triangle_packet.svh"

module systolic_chain_tb;

    localparam int N_PU    = 4;
    localparam int N_ROWS  = 5;     // rows per PU
    localparam int FB_DEPTH = 4096;
    localparam int Z_DEPTH  = 4096;

    logic clk, rst;

    // ---------------- chain wiring ----------------
    logic               chain_seed_valid [N_PU];
    logic signed [31:0] chain_seed_e0    [N_PU];
    logic signed [31:0] chain_seed_e1    [N_PU];
    logic signed [31:0] chain_seed_e2    [N_PU];
    logic signed [31:0] chain_seed_z     [N_PU];

    logic               sv_out  [N_PU];
    logic signed [31:0] se0_out [N_PU];
    logic signed [31:0] se1_out [N_PU];
    logic signed [31:0] se2_out [N_PU];
    logic signed [31:0] sz_out  [N_PU];

    // ---------------- broadcast constants (driven by TB) ----------------
    logic signed [31:0] a0_b, a1_b, a2_b, z_step_x_b;
    logic signed [31:0] b0_b, b1_b, b2_b, z_step_y_b;
    logic [7:0]         color_b, row_base_b, last_row_b;
    logic [7:0]         col_base_for_pu [N_PU];

    // ---------------- TB-driven seed for PU0 ----------------
    logic               tb_seed_valid;
    logic signed [31:0] tb_seed_e0, tb_seed_e1, tb_seed_e2, tb_seed_z;

    assign chain_seed_valid[0] = tb_seed_valid;
    assign chain_seed_e0[0]    = tb_seed_e0;
    assign chain_seed_e1[0]    = tb_seed_e1;
    assign chain_seed_e2[0]    = tb_seed_e2;
    assign chain_seed_z[0]     = tb_seed_z;

    // ---------------- per-PU monitoring ----------------
    logic [N_PU-1:0] pu_ready;
    logic [N_PU-1:0] p_write;
    logic [7:0]      p_col   [N_PU];
    logic [7:0]      p_row   [N_PU];
    logic [7:0]      p_color [N_PU];
    logic [15:0]     p_depth [N_PU];

    // VGA tie-offs (not exercised in this TB)
    logic [11:0] vga_r_addr;
    logic        vga_r_buf_sel, fb_write_sel;
    assign vga_r_addr     = '0;
    assign vga_r_buf_sel  = 1'b0;
    assign fb_write_sel   = 1'b0;

    genvar gi;
    generate
        for (gi = 0; gi < N_PU; gi++) begin : g_pu
            // bbox_xmin = 0 (aligned), so col_base = pu_id matches mem bank
            assign col_base_for_pu[gi] = 8'(gi);

            pixel_unit #(
                .PU_ID      (gi),
                .IS_LAST_PU ((gi == N_PU - 1) ? 1'b1 : 1'b0),
                .FB_DEPTH   (FB_DEPTH),
                .Z_DEPTH    (Z_DEPTH)
            ) u_pu (
                .clk            (clk),
                .rst            (rst),
                .ready          (pu_ready[gi]),
                .seed_valid_in  (chain_seed_valid[gi]),
                .seed_e0_in     (chain_seed_e0[gi]),
                .seed_e1_in     (chain_seed_e1[gi]),
                .seed_e2_in     (chain_seed_e2[gi]),
                .seed_z_in      (chain_seed_z[gi]),
                .a0_in          (a0_b),
                .a1_in          (a1_b),
                .a2_in          (a2_b),
                .z_step_x_in    (z_step_x_b),
                .b0_in          (b0_b),
                .b1_in          (b1_b),
                .b2_in          (b2_b),
                .z_step_y_in    (z_step_y_b),
                .color_in       (color_b),
                .col_base_in    (col_base_for_pu[gi]),
                .row_base_in    (row_base_b),
                .last_row_in    (last_row_b),
                .seed_valid_out (sv_out[gi]),
                .seed_e0_out    (se0_out[gi]),
                .seed_e1_out    (se1_out[gi]),
                .seed_e2_out    (se2_out[gi]),
                .seed_z_out     (sz_out[gi]),
                .p_write        (p_write[gi]),
                .p_col          (p_col[gi]),
                .p_row          (p_row[gi]),
                .p_color        (p_color[gi]),
                .p_depth        (p_depth[gi]),
                .vga_r_addr     (vga_r_addr),
                .vga_r_buf_sel  (vga_r_buf_sel),
                .vga_r_data     (),
                .fb_write_sel   (fb_write_sel)
            );

            if (gi < N_PU - 1) begin : g_chain
                assign chain_seed_valid[gi + 1] = sv_out[gi];
                assign chain_seed_e0   [gi + 1] = se0_out[gi];
                assign chain_seed_e1   [gi + 1] = se1_out[gi];
                assign chain_seed_e2   [gi + 1] = se2_out[gi];
                assign chain_seed_z    [gi + 1] = sz_out[gi];
            end
        end
    endgenerate

    // 100 MHz
    initial clk = 0;
    always #5 clk = ~clk;

    int errors = 0;
    task automatic check(input bit cond, input string msg);
        if (!cond) begin
            $error("FAIL: %s", msg);
            errors++;
        end
    endtask

    // ---------------- collectors ----------------
    int cycle_num;
    int pix_count       [N_PU];
    int pu_first_cycle  [N_PU];
    int pu_last_cycle   [N_PU];

    initial begin
        cycle_num = 0;
        for (int u = 0; u < N_PU; u++) begin
            pix_count[u]      = 0;
            pu_first_cycle[u] = -1;
            pu_last_cycle[u]  = -1;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            cycle_num <= 0;
        end else begin
            cycle_num <= cycle_num + 1;
            for (int u = 0; u < N_PU; u++) begin
                if (p_write[u]) begin
                    if (pu_first_cycle[u] == -1)
                        pu_first_cycle[u] <= cycle_num;
                    pu_last_cycle[u] <= cycle_num;
                    pix_count[u] <= pix_count[u] + 1;
                end
            end
        end
    end

    // diagonal-pattern witness: the first cycle PU(N_PU-1) writes is the
    // cycle the chain is fully filled. At that moment we expect every PU
    // to be writing, with rows forming a diagonal (PU0=N_PU-1, PU1=N_PU-2,
    // ..., PU(N-1)=0).
    bit diagonal_checked;
    initial diagonal_checked = 1'b0;

    always @(posedge clk) begin
        if (!rst && p_write[N_PU - 1] && !diagonal_checked) begin
            diagonal_checked <= 1'b1;
            for (int u = 0; u < N_PU; u++) begin
                check(p_write[u] === 1'b1,
                      $sformatf("diagonal: PU%0d not writing on fill cycle", u));
                check(p_row[u] === 8'(N_PU - 1 - u),
                      $sformatf("diagonal: PU%0d row=%0d expected %0d",
                                u, p_row[u], N_PU - 1 - u));
                check(p_col[u] === 8'(u),
                      $sformatf("diagonal: PU%0d col=%0d expected %0d",
                                u, p_col[u], u));
            end
        end
    end

    // PU3 must never forward a seed (IS_LAST_PU = 1)
    always @(posedge clk) begin
        if (!rst && sv_out[N_PU - 1])
            $error("FAIL: PU(N-1) asserted seed_valid_out (IS_LAST_PU broken)");
    end

    // unroll the z-init across the 4 generate-block instances; runtime
    // indexing into generate arrays isn't portable across all simulators
    task automatic z_clear_all_pus_to_far();
        for (int i = 0; i < Z_DEPTH; i++) begin
            g_pu[0].u_pu.z_mem[i] = 16'hFFFF;
            g_pu[1].u_pu.z_mem[i] = 16'hFFFF;
            g_pu[2].u_pu.z_mem[i] = 16'hFFFF;
            g_pu[3].u_pu.z_mem[i] = 16'hFFFF;
        end
    endtask

    task automatic drive_idle();
        tb_seed_valid = 1'b0;
        tb_seed_e0    = '0;
        tb_seed_e1    = '0;
        tb_seed_e2    = '0;
        tb_seed_z     = '0;
        a0_b          = '0;
        a1_b          = '0;
        a2_b          = '0;
        z_step_x_b    = '0;
        b0_b          = '0;
        b1_b          = '0;
        b2_b          = '0;
        z_step_y_b    = '0;
        color_b       = '0;
        row_base_b    = '0;
        last_row_b    = '0;
    endtask

    initial begin
        drive_idle();
        rst = 1'b1;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        z_clear_all_pus_to_far();

        // ---- set broadcast constants ----
        // a values nonzero so we can verify e propagates with +a along chain.
        // b = 0 keeps every cycle inside the triangle (constant-positive edges).
        a0_b       <= 32'sd1;
        a1_b       <= 32'sd1;
        a2_b       <= 32'sd1;
        z_step_x_b <= 32'sd0;
        b0_b       <= 32'sd0;
        b1_b       <= 32'sd0;
        b2_b       <= 32'sd0;
        z_step_y_b <= 32'sd0;
        color_b    <= 8'h7E;
        row_base_b <= 8'd0;
        last_row_b <= 8'(N_ROWS - 1);

        tb_seed_e0 <= 32'sd100;
        tb_seed_e1 <= 32'sd200;
        tb_seed_e2 <= 32'sd300;
        tb_seed_z  <= 32'sh0040;
        @(posedge clk);

        // ---- pulse seed for one cycle ----
        tb_seed_valid <= 1'b1;
        @(posedge clk);
        tb_seed_valid <= 1'b0;

        // ---- wait for the chain to drain ----
        // worst case: fill (N_PU) + active rows (N_ROWS) + read pipeline (2)
        repeat (N_PU + N_ROWS + 16) @(posedge clk);

        for (int u = 0; u < N_PU; u++) begin
            check(pix_count[u] == N_ROWS,
                  $sformatf("PU%0d emitted %0d pixels, expected %0d",
                            u, pix_count[u], N_ROWS));
            check(pu_ready[u] === 1'b1,
                  $sformatf("PU%0d not back to ready after drain", u));
        end

        // 1-cycle stagger: PU(i+1)'s first emit is exactly one cycle after PU i's
        for (int u = 1; u < N_PU; u++) begin
            check(pu_first_cycle[u] - pu_first_cycle[u-1] == 1,
                  $sformatf("stagger PU%0d-PU%0d: first cycles %0d, %0d",
                            u-1, u, pu_first_cycle[u-1], pu_first_cycle[u]));
            check(pu_last_cycle[u] - pu_last_cycle[u-1] == 1,
                  $sformatf("drain PU%0d-PU%0d: last cycles %0d, %0d",
                            u-1, u, pu_last_cycle[u-1], pu_last_cycle[u]));
        end

        check(diagonal_checked,
              "diagonal cycle witness never fired (chain may not have filled)");

        if (errors == 0)
            $display("PASS systolic_chain_tb: %0d-PU chain, %0d rows each",
                     N_PU, N_ROWS);
        else
            $display("FAIL systolic_chain_tb: %0d errors", errors);
        $finish;
    end

    initial begin
        #200000;
        $error("systolic_chain_tb timed out");
        $finish;
    end

endmodule
