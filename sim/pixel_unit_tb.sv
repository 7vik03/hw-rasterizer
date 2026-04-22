// pixel_unit_tb.sv
// Directed tests for pixel_unit.
//
// What this covers:
//   1. Reset -> ready=1, no pixel_valid
//   2. "Solid bbox" packet (all edge functions always >= 0) rasterizes every
//      pixel in the bbox in row-major order with the packet color
//   3. Back-facing packet is dropped, ready stays high, no pixels emitted
//   4. Packet whose bbox is fully above Y_MAX_CLIP is dropped (y-clip)
//   5. Packet whose bbox straddles Y_MAX_CLIP only emits the in-range rows
//   6. pixel_valid does not latch past the final accepted pixel (no phantom)

`timescale 1ns/1ps
`include "triangle_packet.svh"

module pixel_unit_tb;

    // shrink the clip window so test 4/5 are easy to reason about
    localparam int Y_MIN_CLIP = 2;
    localparam int Y_MAX_CLIP = 5;

    logic clk;
    logic rst;
    logic valid_in;
    triangle_packet_t packet;

    logic        pixel_valid;
    logic [9:0]  pixel_x;
    logic [8:0]  pixel_y;
    logic [7:0]  pixel_color;
    logic [15:0] pixel_depth;
    logic        ready;

    pixel_unit #(
        .Y_MIN_CLIP(Y_MIN_CLIP),
        .Y_MAX_CLIP(Y_MAX_CLIP)
    ) dut (
        .clk(clk),
        .rst(rst),
        .valid_in(valid_in),
        .packet(packet),
        .pixel_valid(pixel_valid),
        .pixel_x(pixel_x),
        .pixel_y(pixel_y),
        .pixel_color(pixel_color),
        .pixel_depth(pixel_depth),
        .ready(ready)
    );

    // 100 MHz
    initial clk = 0;
    always #5 clk = ~clk;

    // pixel collector
    int pixel_count;
    logic [9:0]  last_x;
    logic [8:0]  last_y;
    logic [7:0]  last_color;

    always_ff @(posedge clk) begin
        if (rst) begin
            pixel_count <= 0;
        end else if (pixel_valid) begin
            pixel_count <= pixel_count + 1;
            last_x     <= pixel_x;
            last_y     <= pixel_y;
            last_color <= pixel_color;
            $display("  [%0t] px (%0d,%0d) color=0x%02h depth=0x%04h",
                     $time, pixel_x, pixel_y, pixel_color, pixel_depth);
        end
    end

    // build a packet whose edge functions are constant-positive, so every
    // pixel inside the bbox is drawn. keeps the test independent of Q12.12
    // edge-math correctness; we're exercising the traversal + latching.
    function automatic triangle_packet_t make_solid_packet(
        input logic [9:0] xmin,
        input logic [9:0] xmax,
        input logic [8:0] ymin,
        input logic [8:0] ymax,
        input logic [7:0] color,
        input logic       front_facing
    );
        triangle_packet_t p;
        p = '0;
        p.bbox_xmin    = xmin;
        p.bbox_xmax    = xmax;
        p.bbox_ymin    = ymin;
        p.bbox_ymax    = ymax;
        // a_i = b_i = 0, e_i_init = 1 -> edges stay positive everywhere
        p.a0 = 0; p.a1 = 0; p.a2 = 0;
        p.b0 = 0; p.b1 = 0; p.b2 = 0;
        p.e0_init = 32'sd1;
        p.e1_init = 32'sd1;
        p.e2_init = 32'sd1;
        // depth: constant 0.5 in Q12.12 = 0x00000800 at origin, no slope
        p.z_at_origin = 32'sh00000800;
        p.z_step_x    = 0;
        p.z_step_y    = 0;
        p.color        = color;
        p.front_facing = front_facing;
        return p;
    endfunction

    // drive one packet: wait for ready, hold valid_in for 1 cycle
    task automatic send_packet(input triangle_packet_t p);
        @(posedge clk);
        while (!ready) @(posedge clk);
        valid_in <= 1'b1;
        packet   <= p;
        @(posedge clk);
        valid_in <= 1'b0;
        packet   <= '0;
    endtask

    // wait until DUT reports ready again (end of rasterization) + 2 cycles
    // of grace so we can catch any phantom pixel_valid after the last pixel
    task automatic wait_done();
        @(posedge clk);
        while (!ready) @(posedge clk);
        @(posedge clk);
        @(posedge clk);
    endtask

    task automatic expect_count(input int exp, input string label);
        if (pixel_count !== exp) begin
            $error("FAIL [%s]: expected %0d pixels, got %0d", label, exp, pixel_count);
        end else begin
            $display("PASS [%s]: %0d pixels", label, pixel_count);
        end
    endtask

    initial begin
        triangle_packet_t p;
        int exp;

        valid_in = 0;
        packet   = '0;
        rst      = 1;
        repeat (4) @(posedge clk);
        rst = 0;
        @(posedge clk);

        if (!ready) $error("FAIL: ready should be high after reset");
        if (pixel_valid) $error("FAIL: pixel_valid should be low after reset");

        // -----------------------------------------------------------------
        // Test 2: solid bbox, fully inside clip window
        //   x in [3,6]  (4 cols)
        //   y in [2,4]  (3 rows, inside [Y_MIN_CLIP, Y_MAX_CLIP] = [2,5])
        //   -> 12 pixels
        // -----------------------------------------------------------------
        $display("--- Test 2: solid 4x3 bbox inside clip window ---");
        pixel_count = 0;
        p = make_solid_packet(10'd3, 10'd6, 9'd2, 9'd4, 8'hA5, 1'b1);
        send_packet(p);
        wait_done();
        expect_count(12, "solid 4x3");
        // first pixel should be top-left of bbox, last should be bottom-right
        if (last_x !== 10'd6 || last_y !== 9'd4)
            $error("FAIL [solid 4x3]: last pixel was (%0d,%0d), expected (6,4)", last_x, last_y);
        if (last_color !== 8'hA5)
            $error("FAIL [solid 4x3]: color was 0x%02h, expected 0xA5", last_color);

        // -----------------------------------------------------------------
        // Test 3: back-facing -> dropped
        // -----------------------------------------------------------------
        $display("--- Test 3: back-facing packet dropped ---");
        pixel_count = 0;
        p = make_solid_packet(10'd3, 10'd6, 9'd2, 9'd4, 8'h11, 1'b0);
        send_packet(p);
        // back-face: DUT keeps ready=1, never goes active, so wait_done returns fast
        repeat (4) @(posedge clk);
        expect_count(0, "back-face cull");
        if (!ready) $error("FAIL [back-face]: ready should stay high");

        // -----------------------------------------------------------------
        // Test 4: fully above Y_MAX_CLIP -> dropped by y-clip
        //   bbox y in [10,20], clip window ends at 5 -> clip_ymin > clip_ymax
        // -----------------------------------------------------------------
        $display("--- Test 4: bbox fully outside clip window ---");
        pixel_count = 0;
        p = make_solid_packet(10'd3, 10'd6, 9'd10, 9'd20, 8'h22, 1'b1);
        send_packet(p);
        repeat (4) @(posedge clk);
        expect_count(0, "y-clip drop");
        if (!ready) $error("FAIL [y-clip drop]: ready should stay high");

        // -----------------------------------------------------------------
        // Test 5: bbox straddles Y_MAX_CLIP
        //   bbox y in [4,9], clip [2,5] -> clipped to y in [4,5] (2 rows)
        //   x in [3,5] (3 cols)
        //   -> 6 pixels
        // -----------------------------------------------------------------
        $display("--- Test 5: bbox clipped on bottom ---");
        pixel_count = 0;
        p = make_solid_packet(10'd3, 10'd5, 9'd4, 9'd9, 8'h33, 1'b1);
        send_packet(p);
        wait_done();
        expect_count(6, "y-clip partial");
        if (last_y !== 9'd5)
            $error("FAIL [y-clip partial]: last row was %0d, expected 5", last_y);

        // -----------------------------------------------------------------
        // Test 6: no phantom pixel — after Test 5 we let the collector run
        //   a few more cycles; pixel_count should NOT tick up
        // -----------------------------------------------------------------
        begin
            int snapshot;
            snapshot = pixel_count;
            repeat (8) @(posedge clk);
            if (pixel_count !== snapshot)
                $error("FAIL [phantom]: pixel_count grew from %0d to %0d after done",
                       snapshot, pixel_count);
            else
                $display("PASS [phantom]: no extra pixel_valid after done");
        end

        // -----------------------------------------------------------------
        // Test 7: back-to-back packets — send two solid bboxes in a row,
        // make sure the second one latches its own params (not stale ones)
        // -----------------------------------------------------------------
        $display("--- Test 7: back-to-back packets ---");
        pixel_count = 0;
        p = make_solid_packet(10'd0, 10'd1, 9'd2, 9'd3, 8'h44, 1'b1); // 2x2 = 4
        send_packet(p);
        wait_done();
        // change packet signals immediately; DUT should have latched the old one
        p = make_solid_packet(10'd10, 10'd12, 9'd3, 9'd5, 8'h55, 1'b1); // 3x3 = 9
        send_packet(p);
        wait_done();
        expect_count(4 + 9, "back-to-back");
        if (last_color !== 8'h55)
            $error("FAIL [back-to-back]: last color 0x%02h, expected 0x55", last_color);

        $display("--- done ---");
        $finish;
    end

    // safety net
    initial begin
        #200000;
        $error("FAIL: testbench timed out");
        $finish;
    end

endmodule
