module vga_framebuffer (
    input  logic        clk,
    input  logic        reset,

    // One returned VGA pixel from each of the 16 pixel units
    input  logic [7:0]  pu_vga_data [0:15],

    // Broadcast to all pixel units
    output logic [11:0] vga_r_addr,
    output logic        vga_r_buf_sel,

    // Current rasterizer write buffer
    input  logic        fb_write_sel,

    // VGA pins
    output logic [7:0]  VGA_R, VGA_G, VGA_B,
    output logic        VGA_CLK, VGA_HS, VGA_VS,
    output logic        VGA_BLANK_n,
    output logic        VGA_SYNC_n
);

    logic [10:0] hcount;
    logic [9:0]  vcount;

    vga_counters counters (
        .clk50(clk),
        .reset(reset),
        .hcount(hcount),
        .vcount(vcount),
        .VGA_CLK(VGA_CLK),
        .VGA_HS(VGA_HS),
        .VGA_VS(VGA_VS),
        .VGA_BLANK_n(VGA_BLANK_n),
        .VGA_SYNC_n(VGA_SYNC_n)
    );

    // 640x480 VGA screen
    // Internal framebuffer is 256x240, scaled 2x to 512x480.
    // Centered horizontally, so active region is screen x = 64..575.

    logic [9:0] screen_x;
    logic [7:0] fb_x;
    logic [7:0] fb_y;
    logic       in_fb_region;

    assign screen_x     = hcount[10:1];       // 0..639
    assign in_fb_region = (screen_x >= 10'd64) && (screen_x < 10'd576);

    assign fb_x = hcount[10:2] - 8'd32;       // 0..255 inside region
    assign fb_y = vcount[9:1];                // 0..239

    // Address used INSIDE each PU:
    //   fb_x[7:4] = local column-bank index inside that PU, 0..15
    //   fb_y      = row, 0..239
    //
    // The bottom 4 bits of fb_x pick WHICH PU owns the pixel.
    assign vga_r_addr = {fb_x[7:4], fb_y};

    // VGA reads front buffer, rasterizer writes back buffer
    assign vga_r_buf_sel = ~fb_write_sel;

    // Delay PU select by 1 cycle because pixel_unit VGA read is registered.
    logic [3:0] pu_sel_q;
    logic       in_fb_region_q;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            pu_sel_q       <= 4'd0;
            in_fb_region_q <= 1'b0;
        end else begin
            pu_sel_q       <= fb_x[3:0];
            in_fb_region_q <= in_fb_region;
        end
    end

    logic [7:0] pixelcolor;
    assign pixelcolor = pu_vga_data[pu_sel_q];

    always_comb begin
        VGA_R = 8'h00;
        VGA_G = 8'h00;
        VGA_B = 8'h00;

        if (VGA_BLANK_n && in_fb_region_q) begin
            VGA_R = {pixelcolor[7:5], pixelcolor[7:5], pixelcolor[7:6]};
            VGA_G = {pixelcolor[4:2], pixelcolor[4:2], pixelcolor[4:3]};
            VGA_B = {pixelcolor[1:0], pixelcolor[1:0],
                     pixelcolor[1:0], pixelcolor[1:0]};
        end
    end

endmodule
