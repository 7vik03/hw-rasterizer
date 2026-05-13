`include "triangle_packet.svh"

// avalon_interface
// ----------------
// HPS-side Avalon-MM slave + triangle staging + triangle FIFO.
//
// HPS writes 17 x 32-bit words into a staging array (one word per address)
// then writes COMMIT to push the assembled packet into the FIFO.
//
// Downstream side (dispatcher) sees a 2-cycle pop handshake:
//   pop           -> dispatcher requesting a packet (level)
//   pop_available -> we have data sitting on pop_data (level, == !empty)
//   pop_data      -> packet at FIFO head (combinationally visible)
//   pop_ACK       -> dispatcher latched it, advance the FIFO read pointer
//
// Address map (word addresses, avalon_address[6:0]):
//   0x00 - 0x10  packet words 0..16     (W)
//   0x11         COMMIT                 (W, any data)
//   0x12         STATUS                 (R) {swap_busy, full, empty, level[5:0]}
//   0x13         CONTROL                (R/W) reserved for future IRQ work
//   0x14         PRESENT                (W, any data) -- self-clearing pulse
//
// The C-side memory layout (rasterizer_packet_t in avalon_kernel.h) defines
// what each staging word means:
//
//   word  0 : a0           word  9 : e0_init
//   word  1 : b0           word 10 : e1_init
//   word  2 : c0   *       word 11 : e2_init
//   word  3 : a1           word 12 : z_at_origin
//   word  4 : b1           word 13 : z_step_x
//   word  5 : c1   *       word 14 : z_step_y
//   word  6 : a2           word 15 : bbox_packed   { ymax, ymin, xmax, xmin }
//   word  7 : b2           word 16 : flags_color   { ..., front_facing, color }
//   word  8 : c2   *
//
// (* c0/c1/c2 are not used by hardware; the e?_init values already encode
//    them at the bbox origin. They are accepted from software so the kernel
//    struct stays a clean mirror of the math, but they are dropped during
//    packing.)

module avalon_interface (
    input logic clk, rst,
    input logic [6:0] avalon_address,
    input logic avalon_write,
    input logic [31:0] avalon_writedata,
    output logic [31:0] avalon_readdata,

    input logic pop,
    output logic pop_available,
    output triangle_packet_t pop_data,
    input logic pop_ACK,
    output logic fifo_full,
    output logic fifo_empty,
    output logic [5:0] fifo_level,
    output logic present_req,
    input logic swap_busy
);

    // -----------------------------------------------------------------
    // Address map constants (must match avalon_kernel.h in software)
    // -----------------------------------------------------------------
    localparam logic [6:0] ADDR_PACKET_LO = 7'h00;
    localparam logic [6:0] ADDR_PACKET_HI = 7'h10;
    localparam logic [6:0] ADDR_COMMIT = 7'h11;
    localparam logic [6:0] ADDR_STATUS = 7'h12;
    localparam logic [6:0] ADDR_CONTROL = 7'h13;
    localparam logic [6:0] ADDR_PRESENT = 7'h14;

    // -----------------------------------------------------------------
    // Staging registers: 17 x 32-bit, indexed by avalon_address[4:0]
    // -----------------------------------------------------------------
    logic [31:0] stage [17];

    // CONTROL register shadow (irq_enable, low_watermark) - reserved for
    // future interrupt support, currently inert.
    logic [31:0] control_reg;

    // FIFO push pulse, asserted for one cycle when COMMIT is written
    logic fifo_push;
    triangle_packet_t fifo_push_data;

    // -----------------------------------------------------------------
    // Pack the staging array into a triangle_packet_t.
    // Done combinationally so the FIFO sees the right data on the
    // commit cycle without an extra pipeline stage.
    //
    // bbox_packed layout (matches software make_test_packet):
    //   bits [31:24] ymax
    //   bits [23:16] ymin
    //   bits [15: 8] xmax
    //   bits [ 7: 0] xmin
    //
    // flags_color layout:
    //   bit  [8]    front_facing
    //   bits [7:0]  color (RGB332)
    // -----------------------------------------------------------------
    always_comb begin
        fifo_push_data = '0;

        // bounding box - widen the 8-bit C fields to the SV struct widths
        fifo_push_data.bbox_xmin = {2'b00, stage[15][ 7: 0]};
        fifo_push_data.bbox_xmax = {2'b00, stage[15][15: 8]};
        fifo_push_data.bbox_ymin = {1'b0, stage[15][23:16]};
        fifo_push_data.bbox_ymax = {1'b0, stage[15][31:24]};

        // edge coefficients (a, b only; c is not used downstream)
        fifo_push_data.a0 = stage[0];
        fifo_push_data.b0 = stage[1];
        fifo_push_data.a1 = stage[3];
        fifo_push_data.b1 = stage[4];
        fifo_push_data.a2 = stage[6];
        fifo_push_data.b2 = stage[7];

        // initial edge values at the bbox origin
        fifo_push_data.e0_init = stage[ 9];
        fifo_push_data.e1_init = stage[10];
        fifo_push_data.e2_init = stage[11];

        // depth interpolation
        fifo_push_data.z_at_origin = stage[12];
        fifo_push_data.z_step_x = stage[13];
        fifo_push_data.z_step_y = stage[14];

        // color and culling flag
        fifo_push_data.color = stage[16][ 7:0];
        fifo_push_data.front_facing = stage[16][ 8];
    end

    // -----------------------------------------------------------------
    // Avalon write logic
    //   - packet word write : latch into stage[]
    //   - COMMIT            : pulse fifo_push for one cycle
    //   - CONTROL           : update shadow register
    // -----------------------------------------------------------------
    always_ff @(posedge clk) begin
        // default: commit pulse is one cycle wide
        fifo_push <= 1'b0;
        present_req<=1'b0;

        if (rst) begin
            for (int i = 0; i < 17; i++) stage[i] <= '0;
            control_reg<='0;
            present_req<=1'b0;
        end
        else if (avalon_write) begin
            if (avalon_address >= ADDR_PACKET_LO &&
                avalon_address<=ADDR_PACKET_HI) begin
                stage[avalon_address[4:0]]<=avalon_writedata;
            end
            else if (avalon_address == ADDR_COMMIT) begin
                // Drop the commit silently if the FIFO is full. Software
                // is expected to read STATUS first and back off; this is
                // a safety net so a buggy pusher doesn't wedge the bus.
                if (!fifo_full) 
                    fifo_push<=1'b1;
            end
            else if (avalon_address == ADDR_CONTROL) begin
                // CONTROL is purely irq_enable + low_watermark. PRESENT
                // used to share this register on bit 8, which clobbered
                // the persistent fields on every present write; it now
                // has its own ADDR_PRESENT (below) with no register.
                control_reg<=avalon_writedata;
            end
            else if (avalon_address == ADDR_PRESENT) begin
                // Self-clearing command pulse: any write fires
                // present_req for one cycle. No backing register, so
                // CONTROL state is preserved across presents.
                present_req<=1'b1;
            end
        end
    end

    // -----------------------------------------------------------------
    // Avalon read logic - combinational, single-cycle response
    // -----------------------------------------------------------------
    always_comb begin
        avalon_readdata = 32'd0;
        case (avalon_address)
            ADDR_STATUS: begin
                avalon_readdata = {23'd0, swap_busy, fifo_full, fifo_empty, fifo_level};
            end
            ADDR_CONTROL: begin
                avalon_readdata = control_reg;
            end
            default: avalon_readdata = 32'd0;
        endcase
    end

    // -----------------------------------------------------------------
    // Triangle FIFO instance
    //
    // The dispatcher's pop_ACK is what advances the FIFO's read pointer.
    // Its `pop` line is just a request flag - we don't pop until
    // pop_ACK arrives.
    // -----------------------------------------------------------------
    triangle_packet_t fifo_head;

    triangle_fifo #(.DEPTH(64)) u_fifo (
        .clk (clk), .rst (rst), .push (fifo_push), .push_data (fifo_push_data),
        .pop (pop_ACK), .pop_data (fifo_head), .full (fifo_full), .empty (fifo_empty),
        .level (fifo_level)
    );

    // -----------------------------------------------------------------
    // Dispatcher-facing handshake
    //   pop_available is high whenever the FIFO has data sitting at the
    //   head ready to be latched. The actual advance happens on pop_ACK.
    // -----------------------------------------------------------------
    assign pop_available = !fifo_empty;
    assign pop_data = fifo_head;

    // `pop` is an input we observe but don't directly act on; the FIFO
    // pop happens on pop_ACK from the dispatcher. Reference it so lint
    // tools don't complain about an unused port.
    logic _pop_unused;
    assign _pop_unused = pop;

endmodule
