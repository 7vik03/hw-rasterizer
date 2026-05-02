`timescale 1ns/1ps

module vga_framebuffer_tb;

    logic clk, reset;

    logic [7:0] pu_vga_data [0:15];

    logic [11:0] vga_r_addr;
    logic        vga_r_buf_sel;
    logic        fb_write_sel;

    logic [7:0] VGA_R, VGA_G, VGA_B;
    logic VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_n, VGA_SYNC_n;

    vga_framebuffer dut (
        .clk(clk),
        .reset(reset),
        .pu_vga_data(pu_vga_data),
        .vga_r_addr(vga_r_addr),
        .vga_r_buf_sel(vga_r_buf_sel),
        .fb_write_sel(fb_write_sel),
        .VGA_R(VGA_R),
        .VGA_G(VGA_G),
        .VGA_B(VGA_B),
        .VGA_CLK(VGA_CLK),
        .VGA_HS(VGA_HS),
        .VGA_VS(VGA_VS),
        .VGA_BLANK_n(VGA_BLANK_n),
        .VGA_SYNC_n(VGA_SYNC_n)
    );

    // 50 MHz clock
    initial clk = 0;
    always #10 clk = ~clk;

    int errors = 0;

    task check(input bit cond, input string msg);
        if (!cond) begin
            $error("FAIL: %s", msg);
            errors++;
        end
    endtask

    function automatic [23:0] rgb332_to_rgb888(input logic [7:0] c);
        rgb332_to_rgb888 = {
            {c[7:5], c[7:5], c[7:6]},
            {c[4:2], c[4:2], c[4:3]},
            {c[1:0], c[1:0], c[1:0], c[1:0]}
        };
    endfunction

    task drive_pattern();
        for (int i = 0; i < 16; i++) begin
            pu_vga_data[i] = {i[3:0], i[3:0]};
        end
    endtask

    // Wait until DUT hits specific pixel
    task wait_pixel(input int h, input int v);
        wait (dut.hcount == h && dut.vcount == v);
        #1;
    endtask

    // Check one framebuffer pixel
    task check_pixel(input int fb_x, input int fb_y);
        int screen_x;
        int h, v;
        int pu;
        logic [11:0] addr;
        logic [23:0] expected_rgb;

        begin
            screen_x = 64 + 2*fb_x;
            h = 2 * screen_x;
            v = 2 * fb_y;

            wait_pixel(h, v);

            pu   = fb_x[3:0];
            addr = {fb_x[7:4], fb_y};

            check(vga_r_addr == addr,
                  $sformatf("addr mismatch got=%h expected=%h", vga_r_addr, addr));

            @(posedge clk);
            #1;

            expected_rgb = rgb332_to_rgb888({pu[3:0], pu[3:0]});

            check({VGA_R, VGA_G, VGA_B} == expected_rgb,
                  $sformatf("RGB mismatch PU=%0d got=%h expected=%h",
                            pu, {VGA_R, VGA_G, VGA_B}, expected_rgb));
        end
    endtask

    task check_black(input int sx, input int sy);
        int h, v;
        begin
            h = 2*sx;
            v = sy;

            wait_pixel(h, v);

            @(posedge clk);
            #1;

            check({VGA_R, VGA_G, VGA_B} == 24'h0,
                  "black bar not black");
        end
    endtask

    initial begin
        drive_pattern();

        fb_write_sel = 0;

        reset = 1;
        repeat (5) @(posedge clk);
        reset = 0;

        repeat (10) @(posedge clk);

        $display("--- buffer select ---");
        check(vga_r_buf_sel == 1, "buf sel wrong");

        $display("--- black bars ---");
        check_black(10, 20);
        check_black(600, 20);

        $display("--- pixels ---");
        check_pixel(0, 0);
        check_pixel(1, 0);
        check_pixel(15, 0);
        check_pixel(37, 5);
        check_pixel(255, 239);

        if (errors == 0)
            $display("PASS vga_framebuffer_tb");
        else
            $display("FAIL vga_framebuffer_tb: %0d errors", errors);

        $finish;
    end

endmodule
