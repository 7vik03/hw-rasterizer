`include "triangle_packet.svh"

// Pops one triangle packet from the FIFO and broadcasts it to N_PU pixel
// units. Only issues a new packet when *every* pixel unit reports ready,
// so the whole systolic chain has fully drained before the next triangle
// arrives. The broadcast `packet_out` stays latched between dispatches,
// so each PU still sees the right constants when its own seed_valid_in
// fires (the chain may not finish latching for ~2*N_PU cycles after
// seed_valid_out[0] pulses).
//
// fifo side uses Shlok's 2-cycle handshake:
//   pop asserted while downstream is ready -> wait for pop_available ->
//   latch pop_data -> pulse pop_ACK the cycle after.
//
// In the systolic 16-PU layout only valid_out[0] is consumed (it drives
// PU0's seed_valid_in; the chain forwards from there). The vector form
// is preserved so older 2-PU testbenches still compile, and so the
// dispatcher itself is oblivious to whether downstream is broadcast or
// systolic.

module triangle_dispatcher #(
    parameter int N_PU = 16
) (
    input  logic             clk,
    input  logic             rst,

    output logic             pop,
    input  logic             pop_available,
    input  triangle_packet_t pop_data,
    output logic             pop_ACK,

    output logic [N_PU-1:0]  valid_out,
    output triangle_packet_t packet_out,
    input  logic [N_PU-1:0]  ready_in,

    // High whenever rasterizer_top is mid-swap or mid-clear. Holds us
    // in WAIT (no pop, no BCAST transition) so the FIFO advances and
    // PU0 receives a seed only once the new back buffer is cleared.
    // Tie 1'b0 in testbenches that don't model the swap path.
    input  logic             block_dispatch
);

    typedef enum logic [1:0] {
        WAIT,     // request a packet and wait for pop_available
        BCAST,    // drive pop_ACK + valid_out for one cycle
        COOLDOWN  // bubble so ready_in can drop before the next WAIT sample
    } state_t;

    state_t           state;
    triangle_packet_t latched;
    logic             all_ready;

    // gating on every PU being ready means the next triangle only goes
    // out after the slowest column (typically PU_(N_PU-1) at the tail of
    // the chain) has fully drained back to IDLE.
    assign all_ready  = &ready_in;
    assign packet_out = latched;

    always_ff @(posedge clk) begin
        // default every cycle so no inferred latches and no stale pulses
        pop       <= 1'b0;
        pop_ACK   <= 1'b0;
        valid_out <= '0;

        if (rst) begin
            state   <= WAIT;
            latched <= '0;
        end else begin
            case (state)
                WAIT: begin
                    if (all_ready && !block_dispatch) pop <= 1'b1;
                    // Also gate the BCAST transition on !block_dispatch
                    // so a pop request that was already in flight when
                    // the block went high doesn't sneak a triangle past.
                    if (pop && pop_available && !block_dispatch) begin
                        latched <= pop_data;
                        state   <= BCAST;
                    end
                end
                BCAST: begin
                    pop_ACK   <= 1'b1;
                    valid_out <= '1;
                    state     <= COOLDOWN;
                end
                COOLDOWN: begin
                    state <= WAIT;
                end
                default: state <= WAIT;
            endcase
        end
    end

endmodule
