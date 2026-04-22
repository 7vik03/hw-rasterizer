// One partition of a 320x240 framebuffer. Each instance stores rows
// [Y_MIN, Y_MAX] so rasterizer_top can shard the screen across pixel units.
// Write and read ports each use their own clock and form a simple dual-port
// pattern Quartus maps to M10K (registered read, 1 cycle latency).
//
// Data width is parameterized so the same module backs the color buffer
// (8-bit, RGB332) and the depth buffer (16-bit).

module framebuffer #(
    parameter int Y_MIN  = 0,
    parameter int Y_MAX  = 119,
    parameter int DATA_W = 8
) (
    // write port (pixel unit)
    input  logic              w_clk,
    input  logic              w_en,
    input  logic [16:0]       w_addr,   // full-screen y*320 + x
    input  logic [DATA_W-1:0] w_data,

    // read port (vga)
    input  logic              r_clk,
    input  logic [16:0]       r_addr,
    output logic [DATA_W-1:0] r_data
);

    localparam int SCREEN_W = 320;
    localparam int ROWS     = Y_MAX - Y_MIN + 1;
    localparam int DEPTH    = ROWS * SCREEN_W;
    localparam int BASE     = Y_MIN * SCREEN_W;
    // guard $clog2 against a 1-row partition collapsing to 0 width
    localparam int LOCAL_W  = (DEPTH <= 1) ? 1 : $clog2(DEPTH);

    logic [DATA_W-1:0]  mem [DEPTH];
    logic [LOCAL_W-1:0] w_local;
    logic [LOCAL_W-1:0] r_local;

    // caller promises addresses fall in [BASE, BASE+DEPTH); truncation is
    // the partition-local index. For Y_MIN=0 this reduces to wire.
    assign w_local = (w_addr - BASE[16:0]);
    assign r_local = (r_addr - BASE[16:0]);

    always_ff @(posedge w_clk) begin
        if (w_en) mem[w_local] <= w_data;
    end

    // registered read keeps the pattern SDP so Quartus infers M10K
    always_ff @(posedge r_clk) begin
        r_data <= mem[r_local];
    end

endmodule
