// pixel_unit_tb.sv

`timescale 1ns/1ps
`include "triangle_packet.svh"

module pixel_unit_tb;

    localparam int PU_ID    = 4'd3;
    localparam int FB_DEPTH = 4096;
    localparam int Z_DEPTH  = 4096;

    logic clk, rst;

    logic seed_valid_in;
    logic signed [31:0] seed_e0_in, seed_e1_in, seed_e2_in, seed_z_in;
    logic signed [31:0] a0_in, a1_in, a2_in, z_step_x_in;
    logic signed [31:0] b0_in, b1_in, b2_in, z_step_y_in;
    logic [7:0] color_in, col_base_in, row_base_in, last_row_in, last_col_in;

    logic ready;
    logic seed_valid_out;
    logic signed [31:0] seed_e0_out, seed_e1_out, seed_e2_out, seed_z_out;

    logic p_write;
    logic [7:0] p_col, p_row, p_color;
    logic [15:0] p_depth;

    logic [11:0] vga_r_addr;
    logic vga_r_buf_sel, fb_write_sel;
    logic [7:0] vga_r_data;

    pixel_unit #(
        .PU_ID(PU_ID),
        .IS_LAST_PU(1'b0),
        .FB_DEPTH(FB_DEPTH),
        .Z_DEPTH(Z_DEPTH),


        .DO_INIT_CLEAR(1'b0)
    ) dut (
        .clk(clk),
        .rst(rst),
        .ready(ready),
        .z_clear_start(1'b0),

        .seed_valid_in(seed_valid_in),
        .seed_e0_in(seed_e0_in),
        .seed_e1_in(seed_e1_in),
        .seed_e2_in(seed_e2_in),
        .seed_z_in(seed_z_in),

        .a0_in(a0_in),
        .a1_in(a1_in),
        .a2_in(a2_in),
        .z_step_x_in(z_step_x_in),

        .b0_in(b0_in),
        .b1_in(b1_in),
        .b2_in(b2_in),
        .z_step_y_in(z_step_y_in),

        .color_in(color_in),
        .col_base_in(col_base_in),
        .row_base_in(row_base_in),
        .last_row_in(last_row_in),
        .last_col_in(last_col_in),

        .seed_valid_out(seed_valid_out),
        .seed_e0_out(seed_e0_out),
        .seed_e1_out(seed_e1_out),
        .seed_e2_out(seed_e2_out),
        .seed_z_out(seed_z_out),

        .p_write(p_write),
        .p_col(p_col),
        .p_row(p_row),
        .p_color(p_color),
        .p_depth(p_depth),

        .vga_r_addr(vga_r_addr),
        .vga_r_buf_sel(vga_r_buf_sel),
        .vga_r_data(vga_r_data),
        .fb_write_sel(fb_write_sel)
    );

    logic last_seed_valid_in;
    logic last_seed_valid_out;
    logic [31:0] last_seed_e0_out;

    pixel_unit #(
        .PU_ID(PU_ID),
        .IS_LAST_PU(1'b1),
        .FB_DEPTH(FB_DEPTH),
        .Z_DEPTH(Z_DEPTH),
        .DO_INIT_CLEAR(1'b0)
    ) dut_last (
        .clk(clk),
        .rst(rst),
        .ready(),
        .z_clear_start(1'b0),

        .seed_valid_in(last_seed_valid_in),
        .seed_e0_in(seed_e0_in),
        .seed_e1_in(seed_e1_in),
        .seed_e2_in(seed_e2_in),
        .seed_z_in(seed_z_in),

        .a0_in(a0_in),
        .a1_in(a1_in),
        .a2_in(a2_in),
        .z_step_x_in(z_step_x_in),

        .b0_in(b0_in),
        .b1_in(b1_in),
        .b2_in(b2_in),
        .z_step_y_in(z_step_y_in),

        .color_in(color_in),
        .col_base_in(col_base_in),
        .row_base_in(row_base_in),
        .last_row_in(last_row_in),
        .last_col_in(last_col_in),

        .seed_valid_out(last_seed_valid_out),
        .seed_e0_out(last_seed_e0_out),
        .seed_e1_out(),
        .seed_e2_out(),
        .seed_z_out(),

        .p_write(),
        .p_col(),
        .p_row(),
        .p_color(),
        .p_depth(),

        .vga_r_addr(12'h000),
        .vga_r_buf_sel(1'b0),
        .vga_r_data(),
        .fb_write_sel(1'b0)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    int errors = 0;

    task automatic check(input bit cond, input string msg);
        if (!cond) begin
            $error("FAIL: %s", msg);
            errors++;
        end
    endtask

    int pix_n;
    logic [7:0]  pix_col   [0:511];
    logic [7:0]  pix_row   [0:511];
    logic [7:0]  pix_color [0:511];
    logic [15:0] pix_depth [0:511];

    always_ff @(posedge clk) begin
        if (rst) begin
            pix_n <= 0;
        end else if (p_write) begin
            $display("[%0t] p_write: col=%0d row=%0d color=%h depth=%h idx=%0d",
                     $time, p_col, p_row, p_color, p_depth, pix_n);
            pix_col[pix_n]   <= p_col;
            pix_row[pix_n]   <= p_row;
            pix_color[pix_n] <= p_color;
            pix_depth[pix_n] <= p_depth;
            pix_n            <= pix_n + 1;
        end
    end

    task automatic z_clear_to_far();
        for (int i = 0; i < Z_DEPTH; i++) begin
            dut.z_mem[i] = 16'hFFFF;
        end
    endtask

    task automatic z_set_all(input logic [15:0] val);
        for (int i = 0; i < Z_DEPTH; i++) begin
            dut.z_mem[i] = val;
        end
    endtask

    task automatic fb_clear();
        for (int i = 0; i < FB_DEPTH; i++) begin
            dut.fb_a_mem[i] = 8'h00;
            dut.fb_b_mem[i] = 8'h00;
        end
    endtask

    task automatic clear_collector();
        begin
            pix_n = 0;
            #1;
        end
    endtask

    task automatic drive_idle();
        begin
            seed_valid_in      = 1'b0;
            last_seed_valid_in = 1'b0;

            seed_e0_in = '0;
            seed_e1_in = '0;
            seed_e2_in = '0;
            seed_z_in  = '0;

            a0_in = '0;
            a1_in = '0;
            a2_in = '0;
            z_step_x_in = '0;

            b0_in = '0;
            b1_in = '0;
            b2_in = '0;
            z_step_y_in = '0;

            color_in    = '0;
            col_base_in = '0;
            row_base_in = '0;
            last_row_in = '0;
            last_col_in = '0;

            vga_r_addr    = '0;
            vga_r_buf_sel = 1'b0;
            fb_write_sel  = 1'b0;
        end
    endtask

    task automatic launch_main();
        begin
            @(negedge clk);
            seed_valid_in = 1'b1;

            @(posedge clk);
            #1;

            seed_valid_in = 1'b0;

            check(ready === 1'b0, "ready low immediately after seed accepted");
        end
    endtask

    task automatic launch_last();
        begin
            @(negedge clk);
            last_seed_valid_in = 1'b1;

            @(posedge clk);
            #1;

            last_seed_valid_in = 1'b0;
        end
    endtask

    task automatic wait_done();
        begin
            wait (ready === 1'b1);
            repeat (4) @(posedge clk);
            #1;
        end
    endtask

    initial begin
        drive_idle();
        fb_clear();
        z_clear_to_far();

        rst = 1'b1;
        repeat (4) @(posedge clk);
        #1;
        rst = 1'b0;
        repeat (2) @(posedge clk);
        #1;

        $display("--- Test 1: reset ---");
        check(ready === 1'b1, "ready high after reset");
        check(p_write === 1'b0, "p_write low at idle");
        check(seed_valid_out === 1'b0, "seed_valid_out low at idle");


        $display("--- Test 2: 5-row column, all-positive edges ---");
        wait_done();
        z_clear_to_far();
        clear_collector();

        seed_e0_in = 32'sd1000;
        seed_e1_in = 32'sd2000;
        seed_e2_in = 32'sd3000;
        seed_z_in  = 32'sh0100;

        a0_in = 32'sd10;
        a1_in = 32'sd20;
        a2_in = 32'sd30;
        z_step_x_in = 32'sd1;

        b0_in = 32'sd0;
        b1_in = 32'sd0;
        b2_in = 32'sd0;
        z_step_y_in = 32'sd0;

        color_in    = 8'hA5;
        col_base_in = 8'h13;
        row_base_in = 8'd0;
        last_row_in = 8'd4;
        last_col_in = 8'h13;

        launch_main();

        check(seed_valid_out === 1'b1, "seed_valid_out high on accept cycle");
        check(seed_e0_out === 32'sd1010, $sformatf("seed_e0_out=%0d expected 1010", seed_e0_out));
        check(seed_e1_out === 32'sd2020, $sformatf("seed_e1_out=%0d expected 2020", seed_e1_out));
        check(seed_e2_out === 32'sd3030, $sformatf("seed_e2_out=%0d expected 3030", seed_e2_out));
        check(seed_z_out  === 32'sh0101, $sformatf("seed_z_out=%h expected 0101", seed_z_out));

        @(posedge clk);
        #1;
        check(seed_valid_out === 1'b0, "seed_valid_out one-cycle pulse");

        wait_done();

        check(pix_n == 5, $sformatf("expected 5 pixels, got %0d", pix_n));
        for (int i = 0; i < 5 && i < pix_n; i++) begin
            check(pix_col[i]   === 8'h13, $sformatf("pix[%0d].col=%h expected 13", i, pix_col[i]));
            check(pix_row[i]   === 8'(i), $sformatf("pix[%0d].row=%0d expected %0d", i, pix_row[i], i));
            check(pix_color[i] === 8'hA5, $sformatf("pix[%0d].color=%h expected A5", i, pix_color[i]));
            check(pix_depth[i] === 16'h0100, $sformatf("pix[%0d].depth=%h expected 0100", i, pix_depth[i]));
        end


        $display("--- Test 3: negative edge culls all pixels ---");
        wait_done();
        z_clear_to_far();
        clear_collector();

        seed_e0_in = -32'sd1;
        seed_e1_in = 32'sd1;
        seed_e2_in = 32'sd1;
        seed_z_in  = 32'sh0100;

        a0_in = 32'sd0;
        a1_in = 32'sd0;
        a2_in = 32'sd0;
        z_step_x_in = 32'sd0;

        b0_in = 32'sd0;
        b1_in = 32'sd0;
        b2_in = 32'sd0;
        z_step_y_in = 32'sd0;

        color_in    = 8'h55;
        col_base_in = 8'h03;
        row_base_in = 8'd0;
        last_row_in = 8'd3;
        last_col_in = 8'h03;

        launch_main();
        wait_done();

        check(pix_n == 0, $sformatf("inside cull: expected 0 pixels, got %0d", pix_n));


        $display("--- Test 4: z-test culls farther fragment ---");
        wait_done();
        z_set_all(16'h0050);
        clear_collector();

        seed_e0_in = 32'sd1;
        seed_e1_in = 32'sd1;
        seed_e2_in = 32'sd1;
        seed_z_in  = 32'sh0100;

        a0_in = 32'sd0;
        a1_in = 32'sd0;
        a2_in = 32'sd0;
        z_step_x_in = 32'sd0;

        b0_in = 32'sd0;
        b1_in = 32'sd0;
        b2_in = 32'sd0;
        z_step_y_in = 32'sd0;

        color_in    = 8'h66;
        col_base_in = 8'h03;
        row_base_in = 8'd0;
        last_row_in = 8'd3;
        last_col_in = 8'h03;

        launch_main();
        wait_done();

        check(pix_n == 0, $sformatf("z-test reject: expected 0 pixels, got %0d", pix_n));


        $display("--- Test 4b: z-test admits closer fragment ---");
        wait_done();
        z_set_all(16'h0050);
        clear_collector();

        seed_e0_in = 32'sd1;
        seed_e1_in = 32'sd1;
        seed_e2_in = 32'sd1;
        seed_z_in  = 32'sh0010;

        color_in    = 8'h77;
        col_base_in = 8'h03;
        row_base_in = 8'd0;
        last_row_in = 8'd3;
        last_col_in = 8'h03;

        launch_main();
        wait_done();

        check(pix_n == 4, $sformatf("z-test admit: expected 4 pixels, got %0d", pix_n));


        $display("--- Test 5: IS_LAST_PU instance does not forward ---");
        wait_done();

        seed_e0_in = 32'sd1;
        seed_e1_in = 32'sd1;
        seed_e2_in = 32'sd1;
        seed_z_in  = 32'sh0100;

        a0_in = 32'sd5;
        a1_in = 32'sd5;
        a2_in = 32'sd5;
        z_step_x_in = 32'sd5;

        b0_in = 32'sd0;
        b1_in = 32'sd0;
        b2_in = 32'sd0;
        z_step_y_in = 32'sd0;

        color_in    = 8'h88;
        col_base_in = 8'h03;
        row_base_in = 8'd0;
        last_row_in = 8'd2;
        last_col_in = 8'h03;

        launch_last();

        check(last_seed_valid_out === 1'b0,
              "IS_LAST_PU: seed_valid_out stays low on accept cycle");

        repeat (10) @(posedge clk);
        #1;


        $display("--- Test 6: back-to-back ---");
        wait_done();
        z_clear_to_far();
        clear_collector();

        seed_e0_in = 32'sd1;
        seed_e1_in = 32'sd1;
        seed_e2_in = 32'sd1;
        seed_z_in  = 32'sh0080;

        a0_in = 32'sd5;
        a1_in = 32'sd5;
        a2_in = 32'sd5;
        z_step_x_in = 32'sd0;

        b0_in = 32'sd0;
        b1_in = 32'sd0;
        b2_in = 32'sd0;
        z_step_y_in = 32'sd0;

        color_in    = 8'h11;
        col_base_in = 8'h03;
        row_base_in = 8'd0;
        last_row_in = 8'd2;
        last_col_in = 8'h03;

        launch_main();
        wait_done();

        color_in    = 8'h22;
        col_base_in = 8'h13;
        row_base_in = 8'd100;
        last_row_in = 8'd103;
        last_col_in = 8'h13;
        seed_z_in   = 32'sh0040;

        launch_main();
        wait_done();

        check(pix_n == 7, $sformatf("back-to-back: expected 7 pixels total, got %0d", pix_n));

        for (int i = 0; i < 3 && i < pix_n; i++) begin
            check(pix_color[i] === 8'h11, $sformatf("first-tri pix[%0d] color", i));
            check(pix_col[i]   === 8'h03, $sformatf("first-tri pix[%0d] col", i));
            check(pix_row[i]   === 8'(i), $sformatf("first-tri pix[%0d] row", i));
        end

        for (int i = 0; i < 4 && (i + 3) < pix_n; i++) begin
            check(pix_color[i+3] === 8'h22, $sformatf("second-tri pix[%0d] color", i));
            check(pix_col[i+3]   === 8'h13, $sformatf("second-tri pix[%0d] col", i));
            check(pix_row[i+3]   === 8'(100 + i), $sformatf("second-tri pix[%0d] row", i));
        end


        $display("--- Test 7: multi-column iteration (3 cols at stride 16) ---");
        wait_done();
        z_clear_to_far();
        clear_collector();

        seed_e0_in = 32'sd100;
        seed_e1_in = 32'sd200;
        seed_e2_in = 32'sd300;
        seed_z_in  = 32'sh0040;

        a0_in = 32'sd2;
        a1_in = 32'sd3;
        a2_in = 32'sd5;
        z_step_x_in = 32'sd0;

        b0_in = 32'sd0;
        b1_in = 32'sd0;
        b2_in = 32'sd0;
        z_step_y_in = 32'sd0;

        color_in    = 8'h7C;
        col_base_in = 8'h03;
        row_base_in = 8'd0;
        last_row_in = 8'd4;
        last_col_in = 8'd47;

        launch_main();
        wait_done();

        $display("Test 7 result: pix_n=%0d col_base_q=%0d e0_col_top=%0d e1_col_top=%0d e2_col_top=%0d",
                 pix_n, dut.col_base_q, dut.e0_col_top, dut.e1_col_top, dut.e2_col_top);

        for (int i = 0; i < pix_n && i < 20; i++) begin
            $display("  pix[%0d]: col=%0d row=%0d color=%h depth=%h",
                     i, pix_col[i], pix_row[i], pix_color[i], pix_depth[i]);
        end

        check(pix_n == 15, $sformatf("multi-col: expected 15 pixels, got %0d", pix_n));

        begin
            logic [7:0] expected_cols [0:2];
            expected_cols[0] = 8'd3;
            expected_cols[1] = 8'd19;
            expected_cols[2] = 8'd35;

            for (int c = 0; c < 3; c++) begin
                for (int r = 0; r < 5; r++) begin
                    int idx;
                    idx = c * 5 + r;

                    if (idx < pix_n) begin
                        check(pix_col[idx] === expected_cols[c],
                              $sformatf("multi-col pix[%0d] col=%0d expected %0d",
                                        idx, pix_col[idx], expected_cols[c]));
                        check(pix_row[idx] === 8'(r),
                              $sformatf("multi-col pix[%0d] row=%0d expected %0d",
                                        idx, pix_row[idx], r));
                        check(pix_color[idx] === 8'h7C,
                              $sformatf("multi-col pix[%0d] color expected 7C", idx));
                    end
                end
            end
        end

        check(dut.col_base_q === 8'd35,
              $sformatf("multi-col: dut.col_base_q=%0d expected 35", dut.col_base_q));

        check(dut.e0_col_top === 32'sd100 + 32'sd32 * 32'sd2,
              $sformatf("multi-col: e0_col_top=%0d expected %0d",
                        dut.e0_col_top, 100 + 32*2));

        check(dut.e1_col_top === 32'sd200 + 32'sd32 * 32'sd3,
              $sformatf("multi-col: e1_col_top=%0d expected %0d",
                        dut.e1_col_top, 200 + 32*3));

        check(dut.e2_col_top === 32'sd300 + 32'sd32 * 32'sd5,
              $sformatf("multi-col: e2_col_top=%0d expected %0d",
                        dut.e2_col_top, 300 + 32*5));

        if (errors == 0)
            $display("PASS pixel_unit_tb");
        else
            $display("FAIL pixel_unit_tb: %0d errors", errors);

        $finish;
    end

    initial begin
        #200000;
        $error("pixel_unit_tb timed out");
        $finish;
    end

endmodule
