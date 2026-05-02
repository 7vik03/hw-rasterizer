`include "triangle_packet.svh"

module avalon_interface (
    input  logic            clk,
    input  logic            rst,

    input  logic [6:0]      avalon_address,
    input  logic            avalon_write,
    input  logic [31:0]     avalon_writedata,
    output logic [31:0]     avalon_readdata,

    input  logic            pop,
    output logic            pop_available,
    output triangle_packet_t pop_data,
    input  logic            pop_ACK,

    output logic            fifo_full,
    output logic            fifo_empty,
    output logic [5:0]      fifo_level
);

    // Temporary smoke-test stub until the real Avalon/FIFO path lands.
    assign pop_available = 1'b0;
    assign pop_data      = '0;
    assign fifo_full     = 1'b0;
    assign fifo_empty    = 1'b1;
    assign fifo_level    = '0;

    always_comb begin
        avalon_readdata = 32'd0;
        if (avalon_address == 7'h0F) begin
            avalon_readdata[0]   = fifo_full;
            avalon_readdata[1]   = fifo_empty;
            avalon_readdata[7:2] = fifo_level;
        end

    end

endmodule
