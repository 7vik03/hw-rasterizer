/*
 * VGA timing generator + scan-out for the 256x240 internal framebuffer.
 *
 * Outputs the (fb_x, fb_y) currently being scanned along with an
 * in_fb_region flag; the rasterizer reads its per-PU framebuffers based
 * on those addresses and feeds back fb_pixel_color one cycle later
 * (PU read latency). The internal 256-wide image is pixel-doubled
 * horizontally to 512 and vertically to 480, then letterboxed inside
 * 640x480 with 64-pixel black bars on left and right (no vertical
 * letterboxing because 240*2 already fills 480).
 *
 * Address timing: hcount[10:1] is the VGA pixel column (0..639) and is
 * held for 2 hcount cycles per pixel; fb_x = (pixel_x - 64) / 2 is
 * therefore held for 4 hcount cycles. The 1-cycle PU read latency lands
 * comfortably within that 4-cycle hold, so no explicit lookahead is
 * needed.
 */

module vga_framebuffer (
    input  logic       clk,
    input  logic       reset,

    // ---- per-cycle scan-out address into the rasterizer's PU memories ----
    output logic [7:0] fb_x,           // 0..255 inside the image region
    output logic [7:0] fb_y,           // 0..239
    output logic       in_fb_region,   // 1 inside the 256x240 image area
    output logic       frame_done,     // 1-cycle end-of-field pulse

    // ---- pixel from the rasterizer (already gated to 0 outside region) ----
    input  logic [7:0] fb_pixel_color, // RGB332

    // ---- VGA pins ----
    output logic [7:0] VGA_R, VGA_G, VGA_B,
    output logic       VGA_CLK,
    output logic       VGA_HS,
    output logic       VGA_VS,
    output logic       VGA_BLANK_n,
    output logic       VGA_SYNC_n
);

    logic [10:0] hcount;
    logic [9:0]  vcount;
    logic        endOfLine;
    logic        endOfField;

    vga_counters counters (
        .clk50      (clk),
        .reset      (reset),
        .hcount     (hcount),
        .vcount     (vcount),
        .VGA_CLK    (VGA_CLK),
        .VGA_HS     (VGA_HS),
        .VGA_VS     (VGA_VS),
        .VGA_BLANK_n(VGA_BLANK_n),
        .VGA_SYNC_n (VGA_SYNC_n),
        .endOfLine  (endOfLine),
        .endOfField (endOfField)
    );

    // hcount[10:1] = VGA pixel column (0..639); vcount[9:1] = row (0..479).
    logic [9:0] pixel_x;
    logic [8:0] pixel_y;
    assign pixel_x = hcount[10:1];
    assign pixel_y = vcount[9:1];

    // image region: pixel_x in [64, 575] (512-wide doubled image),
    // pixel_y in [0, 479] (240 rows doubled). The bars at 0..63 and
    // 576..639 stay black (rasterizer drives 0 there anyway).
    logic in_h_image;
    logic in_v_image;
    assign in_h_image  = (pixel_x >= 10'd64) && (pixel_x < 10'd576);
    assign in_v_image  = (pixel_y < 9'd480);
    assign in_fb_region = in_h_image && in_v_image;

    // map screen-space (pixel_x, pixel_y) -> internal fb (fb_x, fb_y).
    // /2 in each axis is the pixel-doubling factor.
    logic [9:0] off_x;
    assign off_x = pixel_x - 10'd64;
    assign fb_x  = off_x[8:1];
    assign fb_y  = pixel_y[8:1];

    // end-of-field pulse, registered so consumers see a clean one-cycle
    // pulse aligned with a clock edge.
    always_ff @(posedge clk) begin
        if (reset) frame_done <= 1'b0;
        else       frame_done <= endOfField & endOfLine;
    end

    // RGB332 -> 24-bit RGB. Outside VGA_BLANK_n we drive 0 so the
    // sync pulses on the VGA pins are correct. Inside the active region
    // the rasterizer has already zeroed fb_pixel_color in the letterbox
    // bars, so we don't need to gate on in_fb_region here.
    always_comb begin
        {VGA_R, VGA_G, VGA_B} = 24'h0;
        if (VGA_BLANK_n) begin
            VGA_R = {fb_pixel_color[7:5], fb_pixel_color[7:5], fb_pixel_color[7:6]};
            VGA_G = {fb_pixel_color[4:2], fb_pixel_color[4:2], fb_pixel_color[4:3]};
            VGA_B = {fb_pixel_color[1:0], fb_pixel_color[1:0],
                     fb_pixel_color[1:0], fb_pixel_color[1:0]};
        end
    end

endmodule

module vga_counters (
    input  logic        clk50, reset,
    output logic [10:0] hcount,        // hcount[10:1] is pixel column
    output logic [9:0]  vcount,        // vcount[9:1] is pixel row
    output logic        VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_n, VGA_SYNC_n,
    output logic        endOfLine,
    output logic        endOfField
);

    // 640 x 480 VGA timing for a 50 MHz clock: one pixel every other cycle
    parameter HACTIVE      = 11'd 1280,
              HFRONT_PORCH = 11'd 32,
              HSYNC        = 11'd 192,
              HBACK_PORCH  = 11'd 96,
              HTOTAL       = HACTIVE + HFRONT_PORCH + HSYNC + HBACK_PORCH; // 1600

    parameter VACTIVE      = 10'd 480,
              VFRONT_PORCH = 10'd 10,
              VSYNC        = 10'd 2,
              VBACK_PORCH  = 10'd 33,
              VTOTAL       = VACTIVE + VFRONT_PORCH + VSYNC + VBACK_PORCH; // 525

    always_ff @(posedge clk50 or posedge reset)
        if (reset)          hcount <= 0;
        else if (endOfLine) hcount <= 0;
        else                hcount <= hcount + 11'd 1;

    assign endOfLine = hcount == HTOTAL - 1;

    always_ff @(posedge clk50 or posedge reset)
        if (reset)          vcount <= 0;
        else if (endOfLine)
            if (endOfField) vcount <= 0;
            else            vcount <= vcount + 10'd 1;

    assign endOfField = vcount == VTOTAL - 1;

    // Horizontal sync: same pattern as the original 640x480 timing.
    assign VGA_HS = !( (hcount[10:8] == 3'b101) &
                       !(hcount[7:5] == 3'b111));
    assign VGA_VS = !( vcount[9:1] == (VACTIVE + VFRONT_PORCH) / 2);

    assign VGA_SYNC_n = 1'b0;

    assign VGA_BLANK_n = !( hcount[10] & (hcount[9] | hcount[8]) ) &
                         !( vcount[9] | (vcount[8:5] == 4'b1111) );

    assign VGA_CLK = hcount[0]; // 25 MHz pixel clock

endmodule
