`include "triangle_packet.svh"

// single-clock FIFO of triangle packets.
// array-backed so Quartus infers M10K; no show-ahead tricks, head is
// always visible combinationally via mem[rd_ptr].

module triangle_fifo #(parameter int DEPTH = 64) (
    input logic clk, rst, push,
    input triangle_packet_t push_data,
    input logic pop,
    output triangle_packet_t pop_data,

    output logic full, empty,
    // 6 bits matches Shlok's avalon_interface stat register;
    output logic [5:0] level
);
    localparam int ADDR_W =$clog2(DEPTH);
    localparam int CNT_W = $clog2(DEPTH + 1);
    triangle_packet_t mem [DEPTH];

    logic [ADDR_W-1:0] wr_ptr, rd_ptr;
    logic [CNT_W-1:0] count;
    
    assign empty = (count == '0);
    assign full =(count == DEPTH[CNT_W-1:0]);
    assign level = count[5:0];
    assign pop_data = mem[rd_ptr];

    always_ff @(posedge clk) begin
        if (rst) begin
            wr_ptr<= '0;
            rd_ptr<= '0;
            count<= '0;
        end 
        else begin
            unique case ({push && !full, pop && !empty})
                2'b10: begin
                    mem[wr_ptr]<= push_data;
                    wr_ptr<= wr_ptr + 1'b1;
                    count <= count + 1'b1;
                end
                2'b01: begin
                    rd_ptr<= rd_ptr + 1'b1;
                    count<= count - 1'b1;
                end
                2'b11: begin
                    mem[wr_ptr]<= push_data;
                    wr_ptr <= wr_ptr + 1'b1;
                    rd_ptr <= rd_ptr + 1'b1;
                end
                2'b00: begin
                end
        endcase
            
        end
    end
endmodule
