`timescale 1ns/1ps
`include "triangle_packet.svh"

// End-to-end: FIFO + dispatcher + N fake pixel units.
// Pushes 5 packets, fake units accept then stall ready_in for 10 cycles,
// verifies all 5 flow through both units in order with matching content.

module triangle_dispatcher_tb;

    localparam int N            = 16;   // matches the new 16-PU chain
    localparam int NUM_PACKETS  = 5;
    localparam int BUSY_CYCLES  = 10;

    logic clk;
    logic rst;

    // fifo interface
    logic             fifo_push;
    triangle_packet_t fifo_push_data;
    logic             fifo_pop;
    triangle_packet_t fifo_pop_data;
    logic             fifo_full;
    logic             fifo_empty;
    logic [5:0]       fifo_level;

    // dispatcher handshake
    logic             pop;
    logic             pop_available;
    triangle_packet_t pop_data;
    logic             pop_ACK;

    // pixel unit fan-out
    logic [N-1:0]     valid_out;
    triangle_packet_t packet_out;
    logic [N-1:0]     ready_in;

    triangle_fifo u_fifo (
        .clk(clk),
        .rst(rst),
        .push(fifo_push),
        .push_data(fifo_push_data),
        .pop(fifo_pop),
        .pop_data(fifo_pop_data),
        .full(fifo_full),
        .empty(fifo_empty),
        .level(fifo_level)
    );

    // tb-only adapter standing in for avalon_interface's pop side:
    // data is always available while not empty, fifo advances on pop_ACK
    assign pop_available = !fifo_empty;
    assign pop_data      = fifo_pop_data;
    assign fifo_pop      = pop_ACK;

    triangle_dispatcher #(.N_PU(N)) u_dispatcher (
        .clk(clk),
        .rst(rst),
        .pop(pop),
        .pop_available(pop_available),
        .pop_data(pop_data),
        .pop_ACK(pop_ACK),
        .valid_out(valid_out),
        .packet_out(packet_out),
        .ready_in(ready_in),
        .block_dispatch(1'b0)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // fake pixel units: accept one packet on valid+ready, then deassert
    // ready_in for BUSY_CYCLES cycles to mimic a real pixel_unit rasterizing
    int               busy_cnt [N];
    int               accepted [N];
    triangle_packet_t got      [N][NUM_PACKETS];

    genvar gi;
    generate
        for (gi = 0; gi < N; gi++) begin : g_fake_unit
            always_ff @(posedge clk) begin
                if (rst) begin
                    busy_cnt[gi] <= 0;
                    accepted[gi] <= 0;
                end else if (valid_out[gi] && ready_in[gi]) begin
                    got[gi][accepted[gi]] <= packet_out;
                    accepted[gi]          <= accepted[gi] + 1;
                    busy_cnt[gi]          <= BUSY_CYCLES;
                end else if (busy_cnt[gi] > 0) begin
                    busy_cnt[gi] <= busy_cnt[gi] - 1;
                end
            end
            assign ready_in[gi] = (busy_cnt[gi] == 0);
        end
    endgenerate

    // golden
    triangle_packet_t sent [NUM_PACKETS];

    function automatic triangle_packet_t make_packet(input int idx);
        triangle_packet_t p;
        p              = '0;
        p.bbox_xmin    = 10'(idx * 10);
        p.bbox_xmax    = 10'(idx * 10 + 4);
        p.bbox_ymin    = 9'(idx);
        p.bbox_ymax    = 9'(idx + 2);
        p.color        = 8'(idx + 8'h40);
        p.front_facing = 1'b1;
        return p;
    endfunction

    int errors = 0;
    task automatic check(input bit cond, input string msg);
        if (!cond) begin
            $error("FAIL: %s", msg);
            errors++;
        end
    endtask

    initial begin
        int timeout;

        rst            = 1;
        fifo_push      = 0;
        fifo_push_data = '0;
        repeat (4) @(posedge clk);
        rst = 0;
        @(posedge clk);

        check(fifo_empty, "fifo empty after reset");
        for (int u = 0; u < N; u++)
            check(accepted[u] == 0, $sformatf("unit %0d accepted nothing at reset", u));

        // push NUM_PACKETS back-to-back; dispatcher will start draining in
        // parallel once it sees pop_available
        for (int i = 0; i < NUM_PACKETS; i++) begin
            sent[i]        = make_packet(i);
            fifo_push      <= 1'b1;
            fifo_push_data <= sent[i];
            @(posedge clk);
        end
        fifo_push      <= 1'b0;
        fifo_push_data <= '0;

        // wait for every fake unit to see all packets, or give up
        timeout = 0;
        begin
            bit all_done;
            do begin
                all_done = 1'b1;
                for (int u = 0; u < N; u++)
                    if (accepted[u] < NUM_PACKETS) all_done = 1'b0;
                if (all_done) break;
                @(posedge clk);
                timeout++;
            end while (timeout < 5000);
        end

        for (int u = 0; u < N; u++)
            check(accepted[u] == NUM_PACKETS,
                  $sformatf("unit %0d accepted %0d/%0d",
                            u, accepted[u], NUM_PACKETS));
        check(fifo_empty, "fifo should drain fully");

        // content + order, plus broadcast identity across all units
        for (int i = 0; i < NUM_PACKETS; i++) begin
            for (int u = 0; u < N; u++) begin
                check(got[u][i] === sent[i],
                      $sformatf("unit %0d packet %0d mismatch", u, i));
                check(got[u][i] === got[0][i],
                      $sformatf("unit %0d disagreed with unit 0 on packet %0d",
                                u, i));
            end
        end

        if (errors == 0)
            $display("PASS triangle_dispatcher_tb: %0d packets to %0d units", NUM_PACKETS, N);
        else
            $display("FAIL triangle_dispatcher_tb: %0d errors", errors);
        $finish;
    end

    initial begin
        #50000;
        $error("triangle_dispatcher_tb timeout");
        $finish;
    end

endmodule
