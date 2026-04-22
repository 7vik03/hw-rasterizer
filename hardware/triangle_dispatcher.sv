`include "triangle_packet.svh"

// pops one triangle packet from the FIFO and broadcasts it to N pixel units.
// only issues a new packet when every pixel unit reports ready; this keeps
// all units in lockstep so each gets every triangle (they self-partition via
// per-instance Y_MIN_CLIP / Y_MAX_CLIP).
//
// fifo side uses Shlok's 2-cycle handshake:
//   pop asserted while downstream is ready -> wait for pop_available ->
//   latch pop_data -> pulse pop_ACK the cycle after.

module triangle_dispatcher #(
    parameter int N = 2
) (
    input  logic             clk,
    input  logic             rst,

    output logic             pop,
    input  logic             pop_available,
    input  triangle_packet_t pop_data,
    output logic             pop_ACK,

    output logic [N-1:0]     valid_out,
    output triangle_packet_t packet_out,
    input  logic [N-1:0]     ready_in
);

    typedef enum logic [1:0] {
        WAIT,     // request a packet and wait for pop_available
        BCAST,    // drive pop_ACK + valid_out for one cycle
        COOLDOWN  // bubble so ready_in can drop before the next WAIT sample
    } state_t;

    state_t           state;
    triangle_packet_t latched;
    logic             all_ready;

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
                    if (all_ready) pop <= 1'b1;
                    // gate the latch on pop being registered high so we honor
                    // the request-before-grant ordering, not just !empty
                    if (pop && pop_available) begin
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
