`include "triangle_packet.svh"

// Top-level integration of the 16-PU systolic rasterizer.
//
//   HPS (Avalon-MM)         (existing avalon_interface, owned by Shlok)
//        |
//        v  pop / pop_ACK
//   triangle_dispatcher --(packet broadcast)--> {pu[0]..pu[15]}
//        |
//        +-- valid_out[0] --> pu[0].seed_valid_in (the systolic seed start)
//
//   pu[i].seed_valid_out --> pu[i+1].seed_valid_in            (chain)
//   pu[i].seed_e?_out    --> pu[i+1].seed_e?_in
//   pu[i].seed_z_out     --> pu[i+1].seed_z_in
//
// Each PU owns one of 16 column banks (low nibble of screen-x). Their
// internal framebuffers are the actual pixel storage; this top-level no
// longer instantiates an external framebuffer module.
//
// VGA read path:
//   vga_framebuffer hands back the (fb_x, fb_y) being scanned this cycle
//   and a flag indicating the active 256x240 region (vs the 64-pixel
//   letterbox bars). All 16 PUs see the same vga_r_addr; the high nibble
//   of fb_x picks the column bank inside one PU, the low nibble picks
//   *which* PU's read result to mux out. PU read latency is 1 cycle, so
//   the mux select is registered for one cycle to align.

module rasterizer_top (
    input  logic        clk,
    input  logic        rst,

    // Avalon-MM slave from HPS. Ports use the standard `avs_*` prefix so
    // Platform Designer's Component Editor auto-detects them as one
    // avalon_slave interface; `avs_waitrequest` is required by the
    // template even when we never actually stall.
    input  logic [6:0]  avs_address,
    input  logic        avs_write,
    input  logic [31:0] avs_writedata,
    output logic [31:0] avs_readdata,
    output logic        avs_waitrequest,

    // VGA pins (conduit, exported to top-level pads)
    output logic [7:0]  VGA_R,
    output logic [7:0]  VGA_G,
    output logic [7:0]  VGA_B,
    output logic        VGA_CLK,
    output logic        VGA_HS,
    output logic        VGA_VS,
    output logic        VGA_BLANK_n,
    output logic        VGA_SYNC_n
);

    // Reads are combinational and writes are accepted in one cycle
    // (COMMIT silently drops if the FIFO is full -- software polls
    // STATUS to back off), so the bus never needs to be stalled.
    assign avs_waitrequest = 1'b0;

    localparam int N_PU = 16;

    // ---------------- Avalon / FIFO / dispatcher handshake ----------------
    logic             disp_pop;
    logic             disp_pop_available;
    triangle_packet_t disp_pop_data;
    logic             disp_pop_ACK;
    logic             fifo_full;
    logic             fifo_empty;
    logic [5:0]       fifo_level;

    // Internal avalon_interface keeps its private `avalon_*` port names;
    // the top-level avs_* signals just feed straight into them.
    avalon_interface u_avalon (
        .clk              (clk),
        .rst              (rst),
        .avalon_address   (avs_address),
        .avalon_write     (avs_write),
        .avalon_writedata (avs_writedata),
        .avalon_readdata  (avs_readdata),
        .pop              (disp_pop),
        .pop_available    (disp_pop_available),
        .pop_data         (disp_pop_data),
        .pop_ACK          (disp_pop_ACK),
        .fifo_full        (fifo_full),
        .fifo_empty       (fifo_empty),
        .fifo_level       (fifo_level)
    );

    logic [N_PU-1:0]  pu_valid_seed;
    triangle_packet_t pu_packet;
    logic [N_PU-1:0]  pu_ready;

    triangle_dispatcher #(.N_PU(N_PU)) u_dispatcher (
        .clk           (clk),
        .rst           (rst),
        .pop           (disp_pop),
        .pop_available (disp_pop_available),
        .pop_data      (disp_pop_data),
        .pop_ACK       (disp_pop_ACK),
        .valid_out     (pu_valid_seed),
        .packet_out    (pu_packet),
        .ready_in      (pu_ready)
    );

    // The dispatcher broadcasts valid_out across N_PU bits, but in the
    // systolic layout only valid_out[0] is consumed -- it kicks PU0 off
    // and the chain fans the seed forward. The other bits dangle.

    // all-ready signal from the chain. We use this both to gate the
    // double-buffer swap (so we never swap mid-rasterization) and as
    // observability. Same wire the dispatcher already AND-reduces
    // internally; replicated here to keep the buffer-swap logic local.
    logic chain_idle;
    assign chain_idle = &pu_ready;

    // ---------------- 16 PUs in a systolic chain ----------------
    logic               chain_seed_valid [N_PU];
    logic signed [31:0] chain_seed_e0    [N_PU];
    logic signed [31:0] chain_seed_e1    [N_PU];
    logic signed [31:0] chain_seed_e2    [N_PU];
    logic signed [31:0] chain_seed_z     [N_PU];

    // PU0's seed comes from the dispatcher (broadcast packet + the
    // valid_out[0] pulse). Subsequent PUs' seeds are forwarded inside
    // the generate block.
    assign chain_seed_valid[0] = pu_valid_seed[0];
    assign chain_seed_e0[0]    = pu_packet.e0_init;
    assign chain_seed_e1[0]    = pu_packet.e1_init;
    assign chain_seed_e2[0]    = pu_packet.e2_init;
    assign chain_seed_z [0]    = pu_packet.z_at_origin;

    // VGA-side read fan-out
    logic [11:0] vga_r_addr;
    logic        vga_r_buf_sel;
    logic        fb_write_sel;
    logic [7:0]  pu_vga_rdata [N_PU];

    // observability: each PU exposes its committed pixel writes; we don't
    // wire these to anything in the top, but they're useful in the
    // chain testbench. Aggregate so synthesis doesn't strip them.
    logic [N_PU-1:0] pu_p_write;
    logic [7:0]      pu_p_col   [N_PU];
    logic [7:0]      pu_p_row   [N_PU];
    logic [7:0]      pu_p_color [N_PU];
    logic [15:0]     pu_p_depth [N_PU];

    genvar gi;
    generate
        for (gi = 0; gi < N_PU; gi++) begin : g_pu
            // outgoing seed nets, ignored on the last PU
            logic               sv_out;
            logic signed [31:0] se0_out, se1_out, se2_out, sz_out;

            // col_base_in[3:0] must equal PU_ID for memory bank coherence;
            // achieved when bbox_xmin is 16-aligned (software invariant).
            logic [7:0] col_base_for_pu;
            assign col_base_for_pu = pu_packet.bbox_xmin[7:0] + 8'(gi);

            pixel_unit #(
                .PU_ID      (gi),
                .IS_LAST_PU ((gi == N_PU - 1) ? 1'b1 : 1'b0)
            ) u_pu (
                .clk            (clk),
                .rst            (rst),
                .ready          (pu_ready[gi]),

                .seed_valid_in  (chain_seed_valid[gi]),
                .seed_e0_in     (chain_seed_e0[gi]),
                .seed_e1_in     (chain_seed_e1[gi]),
                .seed_e2_in     (chain_seed_e2[gi]),
                .seed_z_in      (chain_seed_z[gi]),

                .a0_in          (pu_packet.a0),
                .a1_in          (pu_packet.a1),
                .a2_in          (pu_packet.a2),
                .z_step_x_in    (pu_packet.z_step_x),
                .b0_in          (pu_packet.b0),
                .b1_in          (pu_packet.b1),
                .b2_in          (pu_packet.b2),
                .z_step_y_in    (pu_packet.z_step_y),
                .color_in       (pu_packet.color),
                .col_base_in    (col_base_for_pu),
                .row_base_in    (pu_packet.bbox_ymin[7:0]),
                .last_row_in    (pu_packet.bbox_ymax[7:0]),
                .last_col_in    (pu_packet.bbox_xmax[7:0]),

                .seed_valid_out (sv_out),
                .seed_e0_out    (se0_out),
                .seed_e1_out    (se1_out),
                .seed_e2_out    (se2_out),
                .seed_z_out     (sz_out),

                .p_write        (pu_p_write[gi]),
                .p_col          (pu_p_col[gi]),
                .p_row          (pu_p_row[gi]),
                .p_color        (pu_p_color[gi]),
                .p_depth        (pu_p_depth[gi]),

                .vga_r_addr     (vga_r_addr),
                .vga_r_buf_sel  (vga_r_buf_sel),
                .vga_r_data     (pu_vga_rdata[gi]),

                .fb_write_sel   (fb_write_sel)
            );

            // tie off unused outputs of the last PU so synthesis doesn't
            // complain about the dangling sv_out / se*_out nets
            if (gi == N_PU - 1) begin : g_chain_tail
                // intentionally unconnected: sv_out, se0_out, se1_out,
                // se2_out, sz_out -- they're only meaningful when there
                // is a downstream PU
            end else begin : g_chain
                assign chain_seed_valid[gi + 1] = sv_out;
                assign chain_seed_e0   [gi + 1] = se0_out;
                assign chain_seed_e1   [gi + 1] = se1_out;
                assign chain_seed_e2   [gi + 1] = se2_out;
                assign chain_seed_z    [gi + 1] = sz_out;
            end
        end
    endgenerate

    // ---------------- Frame parity / double-buffer toggle ----------------
    // Toggle which framebuffer the rasterizer writes to (and which one
    // VGA scans out) at the end of each frame. Two conditions must hold
    // at the same time:
    //   1. frame_done is pulsing  (VGA finished scanning a frame)
    //   2. chain_idle is high     (no PU is mid-rasterization)
    // If a triangle is still in flight when frame_done fires, we hold
    // the swap pending until the chain drains. swap_pending captures
    // that case: the swap will fire on the first cycle where chain_idle
    // is high after the pulse.
    //
    // This avoids tearing where the rasterizer would write to the buffer
    // VGA just started reading.
    logic frame_done;
    logic swap_pending;

    always_ff @(posedge clk) begin
        if (rst) begin
            fb_write_sel <= 1'b0;
            swap_pending <= 1'b0;
        end else begin
            // latch the request when frame_done pulses
            if (frame_done) swap_pending <= 1'b1;

            // perform the swap as soon as both: a request is pending AND
            // the chain has drained. clear the pending flag in the same
            // cycle we swap.
            if (swap_pending && chain_idle) begin
                fb_write_sel <= ~fb_write_sel;
                swap_pending <= 1'b0;
            end
        end
    end
    vga_framebuffer u_vga (
        .clk          (clk),
        .reset        (rst),
        .pu_vga_data  (pu_vga_rdata),
        .vga_r_addr   (vga_r_addr),
        .vga_r_buf_sel(vga_r_buf_sel),
        .fb_write_sel (fb_write_sel),
        .frame_done   (frame_done),
        .VGA_R        (VGA_R),
        .VGA_G        (VGA_G),
        .VGA_B        (VGA_B),
        .VGA_CLK      (VGA_CLK),
        .VGA_HS       (VGA_HS),
        .VGA_VS       (VGA_VS),
        .VGA_BLANK_n  (VGA_BLANK_n),
        .VGA_SYNC_n   (VGA_SYNC_n)
    );

endmodule
