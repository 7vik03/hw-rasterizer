// avalon_interface.sv

`include "triangle_packet.svh"


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


    localparam logic [6:0] ADDR_PACKET_LO = 7'h00;
    localparam logic [6:0] ADDR_PACKET_HI = 7'h10;
    localparam logic [6:0] ADDR_COMMIT = 7'h11;
    localparam logic [6:0] ADDR_STATUS = 7'h12;
    localparam logic [6:0] ADDR_CONTROL = 7'h13;
    localparam logic [6:0] ADDR_PRESENT = 7'h14;


    logic [31:0] stage [17];


    logic [31:0] control_reg;


    logic fifo_push;
    triangle_packet_t fifo_push_data;


    always_comb begin
        fifo_push_data = '0;


        fifo_push_data.bbox_xmin = {2'b00, stage[15][ 7: 0]};
        fifo_push_data.bbox_xmax = {2'b00, stage[15][15: 8]};
        fifo_push_data.bbox_ymin = {1'b0, stage[15][23:16]};
        fifo_push_data.bbox_ymax = {1'b0, stage[15][31:24]};


        fifo_push_data.a0 = stage[0];
        fifo_push_data.b0 = stage[1];
        fifo_push_data.a1 = stage[3];
        fifo_push_data.b1 = stage[4];
        fifo_push_data.a2 = stage[6];
        fifo_push_data.b2 = stage[7];


        fifo_push_data.e0_init = stage[ 9];
        fifo_push_data.e1_init = stage[10];
        fifo_push_data.e2_init = stage[11];


        fifo_push_data.z_at_origin = stage[12];
        fifo_push_data.z_step_x = stage[13];
        fifo_push_data.z_step_y = stage[14];


        fifo_push_data.color = stage[16][ 7:0];
        fifo_push_data.front_facing = stage[16][ 8];
    end


    always_ff @(posedge clk) begin

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


                if (!fifo_full)
                    fifo_push<=1'b1;
            end
            else if (avalon_address == ADDR_CONTROL) begin


                control_reg<=avalon_writedata;
            end
            else if (avalon_address == ADDR_PRESENT) begin


                present_req<=1'b1;
            end
        end
    end


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


    triangle_packet_t fifo_head;

    triangle_fifo #(.DEPTH(64)) u_fifo (
        .clk (clk), .rst (rst), .push (fifo_push), .push_data (fifo_push_data),
        .pop (pop_ACK), .pop_data (fifo_head), .full (fifo_full), .empty (fifo_empty),
        .level (fifo_level)
    );


    assign pop_available = !fifo_empty;
    assign pop_data = fifo_head;


    logic _pop_unused;
    assign _pop_unused = pop;

endmodule
