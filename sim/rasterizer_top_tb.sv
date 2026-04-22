`timescale 1ns/1ps

// Smoke test only: instantiates rasterizer_top, toggles reset, idles the
// Avalon bus, and confirms nothing flags an $error during a few thousand
// cycles. A behavioral end-to-end test is blocked on Shlok's
// avalon_interface being real rather than a stub.

module rasterizer_top_tb;

    logic        clk;
    logic        rst;
    logic [6:0]  avalon_address;
    logic        avalon_write;
    logic [31:0] avalon_writedata;
    logic [31:0] avalon_readdata;
    logic [7:0]  VGA_R, VGA_G, VGA_B;
    logic        VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_n, VGA_SYNC_n;

    rasterizer_top dut (
        .clk              (clk),
        .rst              (rst),
        .avalon_address   (avalon_address),
        .avalon_write     (avalon_write),
        .avalon_writedata (avalon_writedata),
        .avalon_readdata  (avalon_readdata),
        .VGA_R            (VGA_R),
        .VGA_G            (VGA_G),
        .VGA_B            (VGA_B),
        .VGA_CLK          (VGA_CLK),
        .VGA_HS           (VGA_HS),
        .VGA_VS           (VGA_VS),
        .VGA_BLANK_n      (VGA_BLANK_n),
        .VGA_SYNC_n       (VGA_SYNC_n)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        rst              = 1;
        avalon_address   = '0;
        avalon_write     = 1'b0;
        avalon_writedata = '0;
        repeat (5) @(posedge clk);
        rst = 0;
        // let the design settle, VGA counters run through some lines
        repeat (2000) @(posedge clk);
        $display("PASS rasterizer_top_tb elaborated + idled");
        $finish;
    end

    initial begin
        #100000;
        $error("rasterizer_top_tb timeout");
        $finish;
    end

endmodule
