// triangle_dispatcher.sv

`include "triangle_packet.svh"


module triangle_dispatcher #( parameter int N_PU = 16) (
    input logic clk, rst,
    output logic pop,
    input logic pop_available,
    input triangle_packet_t pop_data,
    output logic pop_ACK,

    output logic [N_PU-1:0] valid_out,
    output triangle_packet_t packet_out,
    input logic [N_PU-1:0] ready_in,
    input logic block_dispatch
);

    typedef enum logic [1:0] {
        WAIT,
        BCAST,
        COOLDOWN
    } state_t;

    state_t state;
    triangle_packet_t latched;
    logic all_ready;
    assign all_ready =&ready_in;
    assign packet_out =latched;

    always_ff @(posedge clk) begin
        pop <= 1'b0;
        pop_ACK <= 1'b0;
        valid_out<= '0;

        if (rst) begin
            state <= WAIT;
            latched <= '0;
        end else begin
            case (state)
                WAIT: begin
                    if (all_ready && !block_dispatch) pop <= 1'b1;

                    if (pop && pop_available && !block_dispatch)
                        begin
                        latched <= pop_data;
                        state <= BCAST;
                        end
                end
                BCAST: begin
                    pop_ACK <= 1'b1;
                    valid_out<='1;
                    state <= COOLDOWN;
                end
                COOLDOWN: begin
                    state<= WAIT;
                end

                default: state <= WAIT;
            endcase
        end
    end
endmodule
