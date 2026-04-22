`timescale 1ns/1ps

// Exercises write/read of an 8-bit partition with Y_MIN=0 and of a 16-bit
// partition with Y_MIN=120 to make sure the partition offset subtraction
// does what we expect. Also confirms the 1-cycle registered read latency.

module framebuffer_tb;

    localparam int SCREEN_W = 320;

    // 8-bit top half DUT
    logic              wclk_a;
    logic              w_en_a;
    logic [16:0]       w_addr_a;
    logic [7:0]        w_data_a;
    logic              rclk_a;
    logic [16:0]       r_addr_a;
    logic [7:0]        r_data_a;

    framebuffer #(.Y_MIN(0), .Y_MAX(119), .DATA_W(8)) dut_a (
        .w_clk (wclk_a),
        .w_en  (w_en_a),
        .w_addr(w_addr_a),
        .w_data(w_data_a),
        .r_clk (rclk_a),
        .r_addr(r_addr_a),
        .r_data(r_data_a)
    );

    // 16-bit bottom half DUT (stands in for a z buffer)
    logic              wclk_b;
    logic              w_en_b;
    logic [16:0]       w_addr_b;
    logic [15:0]       w_data_b;
    logic              rclk_b;
    logic [16:0]       r_addr_b;
    logic [15:0]       r_data_b;

    framebuffer #(.Y_MIN(120), .Y_MAX(239), .DATA_W(16)) dut_b (
        .w_clk (wclk_b),
        .w_en  (w_en_b),
        .w_addr(w_addr_b),
        .w_data(w_data_b),
        .r_clk (rclk_b),
        .r_addr(r_addr_b),
        .r_data(r_data_b)
    );

    // single shared clock for simplicity; module still supports split clocks
    logic clk;
    assign wclk_a = clk;
    assign rclk_a = clk;
    assign wclk_b = clk;
    assign rclk_b = clk;

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    int errors = 0;
    task automatic check(input bit cond, input string msg);
        if (!cond) begin
            $error("FAIL: %s", msg);
            errors++;
        end
    endtask

    // helper: drive signals synchronously after a posedge and wait to the
    // next negedge to sample, so we're clear of the DUT's NBA region
    task automatic write_a(input int y, input int x, input logic [7:0] d);
        @(posedge clk);
        w_en_a   <= 1'b1;
        w_addr_a <= 17'(y * SCREEN_W + x);
        w_data_a <= d;
        @(posedge clk);
        w_en_a <= 1'b0;
    endtask

    task automatic read_a(input int y, input int x, output logic [7:0] d);
        @(posedge clk);
        r_addr_a <= 17'(y * SCREEN_W + x);
        @(posedge clk);  // at this edge, r_data_a <= mem[addr] is scheduled
        @(negedge clk);  // past NBA region, safe to sample
        d = r_data_a;
    endtask

    task automatic write_b(input int y, input int x, input logic [15:0] d);
        @(posedge clk);
        w_en_b   <= 1'b1;
        w_addr_b <= 17'(y * SCREEN_W + x);
        w_data_b <= d;
        @(posedge clk);
        w_en_b <= 1'b0;
    endtask

    task automatic read_b(input int y, input int x, output logic [15:0] d);
        @(posedge clk);
        r_addr_b <= 17'(y * SCREEN_W + x);
        @(posedge clk);
        @(negedge clk);
        d = r_data_b;
    endtask

    initial begin
        logic [7:0]  got8;
        logic [15:0] got16;

        w_en_a = 0; w_en_b = 0;
        w_addr_a = 0; w_data_a = 0; r_addr_a = 0;
        w_addr_b = 0; w_data_b = 0; r_addr_b = 0;

        repeat (3) @(posedge clk);

        // top partition: write a few pixels, read back
        write_a(0,   0,   8'hA5);
        write_a(10,  42,  8'h3C);
        write_a(119, 319, 8'hFF);  // last legal pixel in partition

        read_a(0,   0,   got8);
        check(got8 === 8'hA5, "top partition (0,0) readback");

        read_a(10,  42,  got8);
        check(got8 === 8'h3C, "top partition (10,42) readback");

        read_a(119, 319, got8);
        check(got8 === 8'hFF, "top partition (119,319) readback");

        // bottom partition tests the Y_MIN=120 offset subtraction
        write_b(120, 0,    16'hBEEF);
        write_b(180, 100,  16'h1234);
        write_b(239, 319,  16'hFFFF);

        read_b(120, 0,    got16);
        check(got16 === 16'hBEEF, "bottom partition (120,0) readback");

        read_b(180, 100,  got16);
        check(got16 === 16'h1234, "bottom partition (180,100) readback");

        read_b(239, 319,  got16);
        check(got16 === 16'hFFFF, "bottom partition (239,319) readback");

        if (errors == 0) $display("PASS framebuffer_tb");
        else             $display("FAIL framebuffer_tb: %0d errors", errors);
        $finish;
    end

    initial begin
        #20000;
        $error("framebuffer_tb timeout");
        $finish;
    end

endmodule
