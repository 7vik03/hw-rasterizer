`include "triangle_packet.svh"

// Top-level integration: HPS-side Avalon-MM decoder feeds the triangle FIFO,
// dispatcher fans packets to two pixel_units (top and bottom halves), each
// writes into its own color and depth partition, double-buffered A/B, and
// VGA reads whichever buffer isn't being written.

module rasterizer_top (
    input  logic        clk,
    input  logic        rst,

    // Avalon-MM slave from HPS
    input  logic [6:0]  avalon_address,
    input  logic        avalon_write,
    input  logic [31:0] avalon_writedata,
    output logic [31:0] avalon_readdata,

    // VGA pins
    output logic [7:0]  VGA_R,
    output logic [7:0]  VGA_G,
    output logic [7:0]  VGA_B,
    output logic        VGA_CLK,
    output logic        VGA_HS,
    output logic        VGA_VS,
    output logic        VGA_BLANK_n,
    output logic        VGA_SYNC_n
);

    localparam int N_UNITS = 2;

    // Shlok's avalon_interface presents the pop-side handshake directly;
    // triangle_fifo is the buffer underneath. Current avalon_interface.sv
    // signature doesn't expose a push port, so for now we assume the FIFO
    // lives inside avalon_interface. The standalone triangle_fifo module
    // stays unused at this level.
    // TODO: confirm with Shlok whether triangle_fifo should be lifted out
    // (deeper buffering) or kept internal.

    logic             disp_pop;
    logic             disp_pop_available;
    triangle_packet_t disp_pop_data;
    logic             disp_pop_ACK;
    logic             fifo_full;
    logic             fifo_empty;
    logic [5:0]       fifo_level;

    avalon_interface u_avalon (
        .clk              (clk),
        .rst              (rst),
        .avalon_address   (avalon_address),
        .avalon_write     (avalon_write),
        .avalon_writedata (avalon_writedata),
        .avalon_readdata  (avalon_readdata),
        .pop              (disp_pop),
        .pop_available    (disp_pop_available),
        .pop_data         (disp_pop_data),
        .pop_ACK          (disp_pop_ACK),
        .fifo_full        (fifo_full),
        .fifo_empty       (fifo_empty),
        .fifo_level       (fifo_level)
    );

    logic [N_UNITS-1:0] pu_valid_in;
    triangle_packet_t   pu_packet;
    logic [N_UNITS-1:0] pu_ready;

    triangle_dispatcher #(.N(N_UNITS)) u_dispatcher (
        .clk           (clk),
        .rst           (rst),
        .pop           (disp_pop),
        .pop_available (disp_pop_available),
        .pop_data      (disp_pop_data),
        .pop_ACK       (disp_pop_ACK),
        .valid_out     (pu_valid_in),
        .packet_out    (pu_packet),
        .ready_in      (pu_ready)
    );

    // per-partition pixel outputs: index 0 is top half, 1 is bottom half
    logic [N_UNITS-1:0] pu_pix_valid;
    logic [9:0]         pu_pix_x     [N_UNITS];
    logic [8:0]         pu_pix_y     [N_UNITS];
    logic [7:0]         pu_pix_color [N_UNITS];
    logic [15:0]        pu_pix_depth [N_UNITS];

    pixel_unit #(.Y_MIN_CLIP(0), .Y_MAX_CLIP(119)) u_pixel_top (
        .clk         (clk),
        .rst         (rst),
        .valid_in    (pu_valid_in[0]),
        .packet      (pu_packet),
        .pixel_valid (pu_pix_valid[0]),
        .pixel_x     (pu_pix_x[0]),
        .pixel_y     (pu_pix_y[0]),
        .pixel_color (pu_pix_color[0]),
        .pixel_depth (pu_pix_depth[0]),
        .ready       (pu_ready[0])
    );

    pixel_unit #(.Y_MIN_CLIP(120), .Y_MAX_CLIP(239)) u_pixel_bot (
        .clk         (clk),
        .rst         (rst),
        .valid_in    (pu_valid_in[1]),
        .packet      (pu_packet),
        .pixel_valid (pu_pix_valid[1]),
        .pixel_x     (pu_pix_x[1]),
        .pixel_y     (pu_pix_y[1]),
        .pixel_color (pu_pix_color[1]),
        .pixel_depth (pu_pix_depth[1]),
        .ready       (pu_ready[1])
    );

    // full-screen linear write addr per partition, y*320 + x.
    // 320 = 256 + 64 so Quartus turns this into two shifts plus an add.
    logic [16:0] w_addr [N_UNITS];
    genvar gi;
    generate
        for (gi = 0; gi < N_UNITS; gi++) begin : g_waddr
            assign w_addr[gi] = 17'(pu_pix_y[gi]) * 17'd320
                              + 17'(pu_pix_x[gi]);
        end
    endgenerate

    // frame_parity = "currently writing to buffer B". Toggles at the end of
    // each VGA frame so the buffer VGA just finished reading becomes the
    // next write target. vga_framebuffer doesn't yet expose frame_done;
    // tie low for now so parity stays at 0 (writes always hit A).
    // TODO: Srika to add a frame_done output (end-of-field pulse) and wire it.
    logic frame_done;
    assign frame_done = 1'b0;

    logic frame_parity;
    always_ff @(posedge clk) begin
        if (rst)             frame_parity <= 1'b0;
        else if (frame_done) frame_parity <= ~frame_parity;
    end

    // per-partition write enables, split A vs B by frame_parity
    logic [N_UNITS-1:0] fb_a_w_en;
    logic [N_UNITS-1:0] fb_b_w_en;
    generate
        for (gi = 0; gi < N_UNITS; gi++) begin : g_wen
            assign fb_a_w_en[gi] = pu_pix_valid[gi] & ~frame_parity;
            assign fb_b_w_en[gi] = pu_pix_valid[gi] &  frame_parity;
        end
    endgenerate

    // VGA read side: vga_framebuffer currently owns its own internal memory
    // and doesn't accept external pixel data. These nets stand in until
    // Srika extends the interface with (r_addr, r_data) and frame_done.
    // TODO: mux r_data across the 4 color buffers based on VGA y and
    // ~frame_parity (read the buffer VGA is displaying, not the one being
    // written) and route the 4 z buffers similarly if they're ever read.
    logic [16:0] vga_r_addr;
    logic [7:0]  fb_a_top_rdata, fb_a_bot_rdata;
    logic [7:0]  fb_b_top_rdata, fb_b_bot_rdata;
    assign vga_r_addr = 17'd0;

    framebuffer #(.Y_MIN(0),   .Y_MAX(119), .DATA_W(8)) u_fb_a_top (
        .w_clk  (clk),
        .w_en   (fb_a_w_en[0]),
        .w_addr (w_addr[0]),
        .w_data (pu_pix_color[0]),
        .r_clk  (clk),
        .r_addr (vga_r_addr),
        .r_data (fb_a_top_rdata)
    );

    framebuffer #(.Y_MIN(120), .Y_MAX(239), .DATA_W(8)) u_fb_a_bot (
        .w_clk  (clk),
        .w_en   (fb_a_w_en[1]),
        .w_addr (w_addr[1]),
        .w_data (pu_pix_color[1]),
        .r_clk  (clk),
        .r_addr (vga_r_addr),
        .r_data (fb_a_bot_rdata)
    );

    framebuffer #(.Y_MIN(0),   .Y_MAX(119), .DATA_W(8)) u_fb_b_top (
        .w_clk  (clk),
        .w_en   (fb_b_w_en[0]),
        .w_addr (w_addr[0]),
        .w_data (pu_pix_color[0]),
        .r_clk  (clk),
        .r_addr (vga_r_addr),
        .r_data (fb_b_top_rdata)
    );

    framebuffer #(.Y_MIN(120), .Y_MAX(239), .DATA_W(8)) u_fb_b_bot (
        .w_clk  (clk),
        .w_en   (fb_b_w_en[1]),
        .w_addr (w_addr[1]),
        .w_data (pu_pix_color[1]),
        .r_clk  (clk),
        .r_addr (vga_r_addr),
        .r_data (fb_b_bot_rdata)
    );

    // Depth buffers: same partitioning, 16 bit data. pixel_unit currently
    // has no z-buffer read port (see its own TODO), so depth test isn't
    // wired; we just capture pixel_depth on valid. r_data is unused.
    // TODO: when pixel_unit gains a z-read FSM, expose r_addr/r_data here.
    logic [15:0] zb_a_top_rdata, zb_a_bot_rdata;
    logic [15:0] zb_b_top_rdata, zb_b_bot_rdata;

    framebuffer #(.Y_MIN(0),   .Y_MAX(119), .DATA_W(16)) u_zb_a_top (
        .w_clk  (clk),
        .w_en   (fb_a_w_en[0]),
        .w_addr (w_addr[0]),
        .w_data (pu_pix_depth[0]),
        .r_clk  (clk),
        .r_addr (vga_r_addr),
        .r_data (zb_a_top_rdata)
    );

    framebuffer #(.Y_MIN(120), .Y_MAX(239), .DATA_W(16)) u_zb_a_bot (
        .w_clk  (clk),
        .w_en   (fb_a_w_en[1]),
        .w_addr (w_addr[1]),
        .w_data (pu_pix_depth[1]),
        .r_clk  (clk),
        .r_addr (vga_r_addr),
        .r_data (zb_a_bot_rdata)
    );

    framebuffer #(.Y_MIN(0),   .Y_MAX(119), .DATA_W(16)) u_zb_b_top (
        .w_clk  (clk),
        .w_en   (fb_b_w_en[0]),
        .w_addr (w_addr[0]),
        .w_data (pu_pix_depth[0]),
        .r_clk  (clk),
        .r_addr (vga_r_addr),
        .r_data (zb_b_top_rdata)
    );

    framebuffer #(.Y_MIN(120), .Y_MAX(239), .DATA_W(16)) u_zb_b_bot (
        .w_clk  (clk),
        .w_en   (fb_b_w_en[1]),
        .w_addr (w_addr[1]),
        .w_data (pu_pix_depth[1]),
        .r_clk  (clk),
        .r_addr (vga_r_addr),
        .r_data (zb_b_bot_rdata)
    );

    vga_framebuffer u_vga (
        .clk         (clk),
        .reset       (rst),
        .VGA_R       (VGA_R),
        .VGA_G       (VGA_G),
        .VGA_B       (VGA_B),
        .VGA_CLK     (VGA_CLK),
        .VGA_HS      (VGA_HS),
        .VGA_VS      (VGA_VS),
        .VGA_BLANK_n (VGA_BLANK_n),
        .VGA_SYNC_n  (VGA_SYNC_n)
    );

endmodule
