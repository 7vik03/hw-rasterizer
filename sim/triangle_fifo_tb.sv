// triangle_fifo_tb.sv

`timescale 1ns/1ps
`include "triangle_packet.svh"

module triangle_fifo_tb;

    localparam int DEPTH       = 64;
    localparam int NUM_PACKETS = 5;

    logic             clk;
    logic             rst;
    logic             push;
    triangle_packet_t push_data;
    logic             pop;
    triangle_packet_t pop_data;
    logic             full, empty;
    logic [5:0]       level;

    triangle_fifo #(.DEPTH(DEPTH)) dut (
        .clk(clk),
        .rst(rst),
        .push(push),
        .push_data(push_data),
        .pop(pop),
        .pop_data(pop_data),
        .full(full),
        .empty(empty),
        .level(level)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    int errors = 0;
    task automatic check(input bit cond, input string msg);
        if (!cond) begin
            $error("FAIL: %s", msg);
            errors++;
        end
    endtask


    int               seen_count;
    triangle_packet_t seen [NUM_PACKETS];

    always_ff @(posedge clk) begin
        if (rst) begin
            seen_count <= 0;
        end else if (pop && !empty) begin
            seen[seen_count] <= pop_data;
            seen_count       <= seen_count + 1;
        end
    end

    triangle_packet_t sent [NUM_PACKETS];

    initial begin
        rst       = 1;
        push      = 0;
        pop       = 0;
        push_data = '0;
        repeat (4) @(posedge clk);
        rst = 0;
        @(posedge clk);

        check(empty, "empty after reset");
        check(!full, "not full after reset");
        check(level == 6'd0, "level 0 after reset");


        for (int i = 0; i < NUM_PACKETS; i++) begin
            sent[i]        = '0;
            sent[i].color  = 8'(i + 8'hA0);
            sent[i].bbox_xmin = 10'(i);
            push      <= 1'b1;
            push_data <= sent[i];
            @(posedge clk);
        end
        push <= 1'b0;
        @(posedge clk);

        check(level == 6'(NUM_PACKETS), "level matches push count");
        check(!empty, "not empty after pushes");


        pop <= 1'b1;
        repeat (NUM_PACKETS) @(posedge clk);
        pop <= 1'b0;
        @(posedge clk);

        check(empty, "empty after draining all");
        check(seen_count == NUM_PACKETS, "captured NUM_PACKETS");

        for (int i = 0; i < NUM_PACKETS; i++) begin
            check(seen[i] === sent[i], $sformatf("order/content mismatch at %0d", i));
        end

        if (errors == 0) $display("PASS triangle_fifo_tb");
        else             $display("FAIL triangle_fifo_tb: %0d errors", errors);
        $finish;
    end

    initial begin
        #10000;
        $error("triangle_fifo_tb timeout");
        $finish;
    end

endmodule
