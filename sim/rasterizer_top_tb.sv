// rasterizer_top_tb.sv

`timescale 1ns/1ps


module rasterizer_top_tb;

    logic        clk;
    logic        rst;
    logic [6:0]  avs_address;
    logic        avs_write;
    logic [31:0] avs_writedata;
    logic [31:0] avs_readdata;
    logic        avs_waitrequest;
    logic [7:0]  VGA_R, VGA_G, VGA_B;
    logic        VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_n, VGA_SYNC_n;

    rasterizer_top dut (
        .clk              (clk),
        .rst              (rst),
        .avs_address      (avs_address),
        .avs_write        (avs_write),
        .avs_writedata    (avs_writedata),
        .avs_readdata     (avs_readdata),
        .avs_waitrequest  (avs_waitrequest),
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
        rst           = 1;
        avs_address   = '0;
        avs_write     = 1'b0;
        avs_writedata = '0;
        repeat (5) @(posedge clk);
        rst = 0;

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
