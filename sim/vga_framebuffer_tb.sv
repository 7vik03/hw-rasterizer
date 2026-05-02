`timescale 1ns/1ps

module vga_framebuffer_tb;

    logic clk, reset;

    logic [7:0] pu_vga_data [0:15];

    logic [11:0] vga_r_addr;
    logic        vga_r_buf_sel;
    logic        fb_write_sel;
    logic        frame_done;

    logic [7:0] VGA_R, VGA_G, VGA_B;
    logic VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_n, VGA_SYNC_n;

    vga_framebuffer dut (
        .clk(clk),
        .reset(reset),

        .pu_vga_data(pu_vga_data),

        .vga_r_addr(vga_r_addr),
        .vga_r_buf_sel(vga_r_buf_sel),

        .fb_write_sel(fb_write_sel),
        .frame_done(frame_done),

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
    initial clk = 1'b0;
    always #10 clk = ~clk;

    int errors = 0;

    task automatic check(input bit cond, input string msg);
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

    task automatic drive_pu_pattern();
        for (int i = 0; i < 16; i++) begin
            // unique fake color per PU
            pu_vga_data[i] = {i[3:0], i[3:0]};
        end
    endtask

    task automatic wait_hv(input int h, input int v);
        begin
            wait (dut.hcount == h && dut.vcount == v);
            #1;
        end
    endtask

    task automatic check_pixel(input int fb_x, input int fb_y);
        int screen_x;
        int h_target;
        int v_target;
        int expected_pu;
        logic [11:0] expected_addr;
        logic [7:0]  expected_color;
        logic [23:0] expected_rgb;

        begin
            // Internal 256x240 image is centered:
            // screen_x = 64 + 2*fb_x
            screen_x = 64 + (2 * fb_x);

            // New vga_counters use hcount = 0..799 directly,
            // not doubled 0..1599.
            h_target = screen_x;

            // Vertical is still 2x scale: fb_y = vcount[9:1]
            v_target = 2 * fb_y;

            wait_hv(h_target, v_target);

            expected_pu    = fb_x[3:0];
            expected_addr  = {fb_x[7:4], fb_y[7:0]};
            expected_color = {expected_pu[3:0], expected_pu[3:0]};
            expected_rgb   = rgb332_to_rgb888(expected_color);

            check(vga_r_addr === expected_addr,
                  $sformatf("addr fb_x=%0d fb_y=%0d got=%h expected=%h",
                            fb_x, fb_y, vga_r_addr, expected_addr));

            // one cycle later, delayed PU select should choose correct color
            @(posedge clk);
            #1;

            check({VGA_R, VGA_G, VGA_B} === expected_rgb,
                  $sformatf("RGB fb_x=%0d fb_y=%0d PU=%0d got=%h expected=%h",
                            fb_x, fb_y, expected_pu,
                            {VGA_R, VGA_G, VGA_B}, expected_rgb));

            $display("PASS pixel fb_x=%0d fb_y=%0d PU=%0d addr=%h RGB=%h",
                     fb_x, fb_y, expected_pu, expected_addr, expected_rgb);
        end
    endtask

    task automatic check_black(input int screen_x, input int screen_y);
        begin
            wait_hv(screen_x, screen_y);

            @(posedge clk);
            #1;

            check({VGA_R, VGA_G, VGA_B} === 24'h000000,
                  $sformatf("black region screen_x=%0d screen_y=%0d got RGB=%h",
                            screen_x, screen_y, {VGA_R, VGA_G, VGA_B}));

            $display("PASS black screen_x=%0d screen_y=%0d", screen_x, screen_y);
        end
    endtask

    initial begin
        drive_pu_pattern();

        fb_write_sel = 1'b0;

        reset = 1'b1;
        repeat (5) @(posedge clk);
        #1;
        reset = 1'b0;

        repeat (10) @(posedge clk);
        #1;

        $display("--- Test 1: buffer select ---");
        check(vga_r_buf_sel === 1'b1,
              "vga_r_buf_sel should equal ~fb_write_sel when fb_write_sel=0");

        fb_write_sel = 1'b1;
        #1;
        check(vga_r_buf_sel === 1'b0,
              "vga_r_buf_sel should equal ~fb_write_sel when fb_write_sel=1");

        fb_write_sel = 1'b0;
        #1;

        $display("--- Test 2: black bars ---");
        check_black(10, 20);
        check_black(600, 20);

        $display("--- Test 3: active framebuffer pixels ---");
        check_pixel(0,   0);
        check_pixel(1,   0);
        check_pixel(15,  0);
        check_pixel(16,  0);
        check_pixel(37,  5);
        check_pixel(255, 239);

        $display("--- Test 4: frame_done pulse ---");
        wait (frame_done === 1'b1);
        #1;
        check(frame_done === 1'b1, "frame_done should pulse high at end of frame");

        @(posedge clk);
        #1;
        check(frame_done === 1'b0, "frame_done should be one-cycle pulse");

        if (errors == 0)
            $display("PASS vga_framebuffer_tb");
        else
            $display("FAIL vga_framebuffer_tb: %0d errors", errors);

        $finish;
    end

    initial begin
        #50_000_000;
        $error("vga_framebuffer_tb timed out");
        $finish;
    end

endmodule
