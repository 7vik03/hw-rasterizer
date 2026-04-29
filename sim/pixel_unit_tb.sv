// pixel_unit_tb.sv
// Directed tests for the systolic-chain pixel_unit.
//
// Tests one PU in isolation. Coverage:
//   1. Reset -> ready=1, no p_write, no seed forward.
//   2. Walk a 5-row column with constant-positive edges. Verifies:
//        - exactly 5 p_writes, in row-major order
//        - p_col == col_base, p_row == row_base+i, p_color, p_depth correct
//        - seed_valid_out fires for exactly one cycle on the first ACTIVE
//          cycle, with seed_e*_out == seed + a, seed_z_out == seed + z_step_x
//   3. Inside-test cull: e0_init negative -> no pixels written.
//   4. Z-test: pre-write a closer depth into z_mem; second triangle with a
//      farther depth must not overwrite (and must not emit p_write).
//   5. IS_LAST_PU=1 instance: seed_valid_out is held low even while ACTIVE.
//   6. Back-to-back triangles: latched constants don't leak between dispatches.

`timescale 1ns/1ps
`include "triangle_packet.svh"

module pixel_unit_tb;

    // pick a non-zero PU_ID so col_base lining up with low nibble is exercised
    localparam int PU_ID   = 4'd3;
    localparam int FB_DEPTH = 4096;
    localparam int Z_DEPTH  = 4096;

    logic               clk, rst;

    // primary DUT signals
    logic               seed_valid_in;
    logic signed [31:0] seed_e0_in, seed_e1_in, seed_e2_in, seed_z_in;
    logic signed [31:0] a0_in, a1_in, a2_in, z_step_x_in;
    logic signed [31:0] b0_in, b1_in, b2_in, z_step_y_in;
    logic [7:0]         color_in, col_base_in, row_base_in, last_row_in;

    logic               ready;
    logic               seed_valid_out;
    logic signed [31:0] seed_e0_out, seed_e1_out, seed_e2_out, seed_z_out;

    logic               p_write;
    logic [7:0]         p_col, p_row, p_color;
    logic [15:0]        p_depth;

    logic [11:0]        vga_r_addr;
    logic               vga_r_buf_sel, fb_write_sel;
    logic [7:0]         vga_r_data;

    pixel_unit #(
        .PU_ID(PU_ID),
        .IS_LAST_PU(1'b0),
        .FB_DEPTH(FB_DEPTH),
        .Z_DEPTH(Z_DEPTH)
    ) dut (
        .clk(clk),
        .rst(rst),
        .ready(ready),
        .seed_valid_in(seed_valid_in),
        .seed_e0_in(seed_e0_in),
        .seed_e1_in(seed_e1_in),
        .seed_e2_in(seed_e2_in),
        .seed_z_in(seed_z_in),
        .a0_in(a0_in), .a1_in(a1_in), .a2_in(a2_in),
        .z_step_x_in(z_step_x_in),
        .b0_in(b0_in), .b1_in(b1_in), .b2_in(b2_in),
        .z_step_y_in(z_step_y_in),
        .color_in(color_in),
        .col_base_in(col_base_in),
        .row_base_in(row_base_in),
        .last_row_in(last_row_in),
        .seed_valid_out(seed_valid_out),
        .seed_e0_out(seed_e0_out),
        .seed_e1_out(seed_e1_out),
        .seed_e2_out(seed_e2_out),
        .seed_z_out(seed_z_out),
        .p_write(p_write),
        .p_col(p_col), .p_row(p_row),
        .p_color(p_color), .p_depth(p_depth),
        .vga_r_addr(vga_r_addr),
        .vga_r_buf_sel(vga_r_buf_sel),
        .vga_r_data(vga_r_data),
        .fb_write_sel(fb_write_sel)
    );

    // a second DUT just to spot-check the IS_LAST_PU path -- same wires
    // tied off the same as the primary except seed_valid_in only drives it
    // through a small mux for one focused test.
    logic               last_seed_valid_in;
    logic               last_seed_valid_out;
    logic [31:0]        last_seed_e0_out;
    pixel_unit #(
        .PU_ID(PU_ID),
        .IS_LAST_PU(1'b1)
    ) dut_last (
        .clk(clk), .rst(rst),
        .ready(),
        .seed_valid_in(last_seed_valid_in),
        .seed_e0_in(seed_e0_in), .seed_e1_in(seed_e1_in),
        .seed_e2_in(seed_e2_in), .seed_z_in(seed_z_in),
        .a0_in(a0_in), .a1_in(a1_in), .a2_in(a2_in),
        .z_step_x_in(z_step_x_in),
        .b0_in(b0_in), .b1_in(b1_in), .b2_in(b2_in),
        .z_step_y_in(z_step_y_in),
        .color_in(color_in), .col_base_in(col_base_in),
        .row_base_in(row_base_in), .last_row_in(last_row_in),
        .seed_valid_out(last_seed_valid_out),
        .seed_e0_out(last_seed_e0_out),
        .seed_e1_out(), .seed_e2_out(), .seed_z_out(),
        .p_write(), .p_col(), .p_row(), .p_color(), .p_depth(),
        .vga_r_addr(12'h0), .vga_r_buf_sel(1'b0),
        .vga_r_data(),
        .fb_write_sel(1'b0)
    );

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

    // pixel collector
    int           pix_n;
    logic [7:0]   pix_col   [0:511];
    logic [7:0]   pix_row   [0:511];
    logic [7:0]   pix_color [0:511];
    logic [15:0]  pix_depth [0:511];

    always_ff @(posedge clk) begin
        if (rst) begin
            pix_n <= 0;
        end else if (p_write) begin
            pix_col[pix_n]   <= p_col;
            pix_row[pix_n]   <= p_row;
            pix_color[pix_n] <= p_color;
            pix_depth[pix_n] <= p_depth;
            pix_n            <= pix_n + 1;
        end
    end

    // -- helpers --
    task automatic z_clear_to_far();
        for (int i = 0; i < Z_DEPTH; i++) dut.z_mem[i] = 16'hFFFF;
    endtask

    task automatic z_set_all(input logic [15:0] val);
        for (int i = 0; i < Z_DEPTH; i++) dut.z_mem[i] = val;
    endtask

    task automatic drive_idle();
        seed_valid_in       = 1'b0;
        last_seed_valid_in  = 1'b0;
        seed_e0_in = '0; seed_e1_in = '0; seed_e2_in = '0; seed_z_in = '0;
        a0_in = '0; a1_in = '0; a2_in = '0; z_step_x_in = '0;
        b0_in = '0; b1_in = '0; b2_in = '0; z_step_y_in = '0;
        color_in    = '0;
        col_base_in = '0;
        row_base_in = '0;
        last_row_in = '0;
        vga_r_addr     = '0;
        vga_r_buf_sel  = 1'b0;
        fb_write_sel   = 1'b0;
    endtask

    // pulse a seed for one cycle. Caller is responsible for setting the
    // const inputs before calling. After the pulse cycle this drops
    // seed_valid_in but leaves constants alone (mimics the dispatcher,
    // which holds packet_out latched while the chain drains).
    task automatic pulse_seed_main();
        @(posedge clk);
        seed_valid_in <= 1'b1;
        @(posedge clk);
        seed_valid_in <= 1'b0;
    endtask

    task automatic pulse_seed_last();
        @(posedge clk);
        last_seed_valid_in <= 1'b1;
        @(posedge clk);
        last_seed_valid_in <= 1'b0;
    endtask

    initial begin
        drive_idle();
        rst = 1'b1;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        // ----------------------------------------------------------------
        // Test 1: reset state
        // ----------------------------------------------------------------
        $display("--- Test 1: reset ---");
        check(ready,             "ready high after reset");
        check(!p_write,          "p_write low at idle");
        check(!seed_valid_out,   "seed_valid_out low at idle");

        // ----------------------------------------------------------------
        // Test 2: 5-row column with all-positive edges and constant z
        // ----------------------------------------------------------------
        $display("--- Test 2: 5-row column, all-positive edges ---");
        z_clear_to_far();
        pix_n = 0;

        seed_e0_in <= 32'sd1000;
        seed_e1_in <= 32'sd2000;
        seed_e2_in <= 32'sd3000;
        seed_z_in  <= 32'sh0100;
        a0_in      <= 32'sd10;
        a1_in      <= 32'sd20;
        a2_in      <= 32'sd30;
        z_step_x_in<= 32'sd1;
        b0_in      <= 32'sd0;
        b1_in      <= 32'sd0;
        b2_in      <= 32'sd0;
        z_step_y_in<= 32'sd0;
        color_in   <= 8'hA5;
        // col_base_in[3:0] must equal PU_ID by construction
        col_base_in<= 8'h13;     // low nibble = 3 = PU_ID, bank = 1
        row_base_in<= 8'd0;
        last_row_in<= 8'd4;
        @(posedge clk);

        // pulse seed_valid_in for one cycle
        seed_valid_in <= 1'b1;
        @(posedge clk);
        seed_valid_in <= 1'b0;

        // The cycle right after the seed is sampled, the PU is in ACTIVE
        // for the first time and must be driving seed_valid_out high with
        // (seed + a) on the e_out lines.
        check(seed_valid_out === 1'b1,
              "seed_valid_out high on first ACTIVE cycle");
        check(seed_e0_out === 32'sd1010,
              $sformatf("seed_e0_out=%0d expected 1010", seed_e0_out));
        check(seed_e1_out === 32'sd2020,
              $sformatf("seed_e1_out=%0d expected 2020", seed_e1_out));
        check(seed_e2_out === 32'sd3030,
              $sformatf("seed_e2_out=%0d expected 3030", seed_e2_out));
        check(seed_z_out  === 32'sh0101,
              $sformatf("seed_z_out=%h expected 0101", seed_z_out));
        check(!ready, "ready low while ACTIVE");

        // one more cycle; seed_valid_out must drop
        @(posedge clk);
        check(!seed_valid_out, "seed_valid_out is a one-cycle pulse");

        // wait for column to drain (5 ACTIVE cycles + 2 cycles of read pipeline)
        repeat (12) @(posedge clk);

        check(ready, "ready high after column finishes");
        check(pix_n == 5, $sformatf("expected 5 pixels, got %0d", pix_n));
        for (int i = 0; i < 5 && i < pix_n; i++) begin
            check(pix_col[i]   === 8'h13,
                  $sformatf("pix[%0d].col=%h expected 13", i, pix_col[i]));
            check(pix_row[i]   === 8'(i),
                  $sformatf("pix[%0d].row=%0d expected %0d", i, pix_row[i], i));
            check(pix_color[i] === 8'hA5,
                  $sformatf("pix[%0d].color=%h expected A5", i, pix_color[i]));
            check(pix_depth[i] === 16'h0100,
                  $sformatf("pix[%0d].depth=%h expected 0100", i, pix_depth[i]));
        end

        // ----------------------------------------------------------------
        // Test 3: inside-test cull -- e0 negative throughout
        // ----------------------------------------------------------------
        $display("--- Test 3: negative edge culls all pixels ---");
        z_clear_to_far();
        pix_n = 0;

        seed_e0_in <= -32'sd1;
        seed_e1_in <= 32'sd1;
        seed_e2_in <= 32'sd1;
        seed_z_in  <= 32'sh0100;
        b0_in <= 0; b1_in <= 0; b2_in <= 0; z_step_y_in <= 0;
        last_row_in <= 8'd3;
        row_base_in <= 8'd0;
        col_base_in <= 8'h03;
        @(posedge clk);

        seed_valid_in <= 1'b1;
        @(posedge clk);
        seed_valid_in <= 1'b0;

        repeat (10) @(posedge clk);
        check(pix_n == 0,
              $sformatf("inside cull: expected 0 pixels, got %0d", pix_n));

        // ----------------------------------------------------------------
        // Test 4: z-test rejects a farther triangle
        //   pre-write z=0x0050 at the address the PU will hit; new z=0x0100
        //   is greater (farther) so the write must not occur
        // ----------------------------------------------------------------
        $display("--- Test 4: z-test culls farther fragment ---");
        z_set_all(16'h0050);
        pix_n = 0;

        seed_e0_in <= 32'sd1; seed_e1_in <= 32'sd1; seed_e2_in <= 32'sd1;
        seed_z_in  <= 32'sh0100;
        b0_in <= 0; b1_in <= 0; b2_in <= 0; z_step_y_in <= 0;
        last_row_in <= 8'd3;
        row_base_in <= 8'd0;
        col_base_in <= 8'h03;
        @(posedge clk);

        seed_valid_in <= 1'b1;
        @(posedge clk);
        seed_valid_in <= 1'b0;

        repeat (10) @(posedge clk);
        check(pix_n == 0,
              $sformatf("z-test: expected 0 pixels, got %0d", pix_n));

        // sanity: same scenario but with z=0x0010 (closer) draws all 4
        $display("--- Test 4b: z-test admits closer fragment ---");
        z_set_all(16'h0050);
        pix_n = 0;

        seed_z_in <= 32'sh0010;
        @(posedge clk);
        seed_valid_in <= 1'b1;
        @(posedge clk);
        seed_valid_in <= 1'b0;

        repeat (10) @(posedge clk);
        check(pix_n == 4,
              $sformatf("z-test admit: expected 4 pixels, got %0d", pix_n));

        // ----------------------------------------------------------------
        // Test 5: IS_LAST_PU = 1 instance must never assert seed_valid_out
        // ----------------------------------------------------------------
        $display("--- Test 5: IS_LAST_PU instance does not forward ---");
        // last DUT shares the const wires; reuse the all-positive setup
        seed_e0_in <= 32'sd1; seed_e1_in <= 32'sd1; seed_e2_in <= 32'sd1;
        seed_z_in  <= 32'sh0100;
        last_row_in <= 8'd2;
        @(posedge clk);

        last_seed_valid_in <= 1'b1;
        @(posedge clk);
        last_seed_valid_in <= 1'b0;
        // first ACTIVE cycle on the last-PU instance
        check(last_seed_valid_out === 1'b0,
              "IS_LAST_PU: seed_valid_out is tied low on first ACTIVE cycle");
        repeat (10) @(posedge clk);

        // ----------------------------------------------------------------
        // Test 6: back-to-back triangles, second one uses different params
        // ----------------------------------------------------------------
        $display("--- Test 6: back-to-back ---");
        z_clear_to_far();
        pix_n = 0;

        // first triangle: 3 rows, color 0x11
        seed_e0_in <= 32'sd1; seed_e1_in <= 32'sd1; seed_e2_in <= 32'sd1;
        seed_z_in  <= 32'sh0080;
        b0_in <= 0; b1_in <= 0; b2_in <= 0; z_step_y_in <= 0;
        a0_in <= 32'sd5; a1_in <= 32'sd5; a2_in <= 32'sd5; z_step_x_in <= 32'sd0;
        color_in    <= 8'h11;
        col_base_in <= 8'h03;
        row_base_in <= 8'd0;
        last_row_in <= 8'd2;
        @(posedge clk);
        seed_valid_in <= 1'b1;
        @(posedge clk);
        seed_valid_in <= 1'b0;

        // wait for it to fully drain
        wait (ready);
        repeat (4) @(posedge clk);

        // second triangle: 4 rows starting at row 100, color 0x22
        color_in    <= 8'h22;
        col_base_in <= 8'h13;
        row_base_in <= 8'd100;
        last_row_in <= 8'd103;
        seed_z_in   <= 32'sh0040;
        @(posedge clk);
        seed_valid_in <= 1'b1;
        @(posedge clk);
        seed_valid_in <= 1'b0;

        repeat (12) @(posedge clk);

        check(pix_n == 3 + 4,
              $sformatf("back-to-back: expected 7 pixels total, got %0d", pix_n));
        // first 3 pixels: color 0x11, col 0x03, rows 0..2
        for (int i = 0; i < 3 && i < pix_n; i++) begin
            check(pix_color[i] === 8'h11,
                  $sformatf("first-tri pix[%0d] color", i));
            check(pix_col[i]   === 8'h03,
                  $sformatf("first-tri pix[%0d] col", i));
            check(pix_row[i]   === 8'(i),
                  $sformatf("first-tri pix[%0d] row", i));
        end
        // next 4 pixels: color 0x22, col 0x13, rows 100..103
        for (int i = 0; i < 4 && (i + 3) < pix_n; i++) begin
            check(pix_color[i+3] === 8'h22,
                  $sformatf("second-tri pix[%0d] color", i));
            check(pix_col[i+3]   === 8'h13,
                  $sformatf("second-tri pix[%0d] col", i));
            check(pix_row[i+3]   === 8'(100 + i),
                  $sformatf("second-tri pix[%0d] row", i));
        end

        if (errors == 0)
            $display("PASS pixel_unit_tb");
        else
            $display("FAIL pixel_unit_tb: %0d errors", errors);
        $finish;
    end

    // safety net
    initial begin
        #200000;
        $error("pixel_unit_tb timed out");
        $finish;
    end

endmodule
