`timescale 1ns/1ps
`include "triangle_packet.svh"

// avalon_interface_tb
// -------------------
// Exercises the full HPS-side path: 17 packet word writes + COMMIT push
// the packet into the FIFO, STATUS reads expose level/full/empty, and the
// downstream pop handshake matches what triangle_dispatcher uses.
//
// Test plan:
//   1. Reset + STATUS reads back zero level / empty.
//   2. Push one packet via Avalon writes; STATUS shows level=1, !empty.
//   3. Pop it via the dispatcher-style handshake; data matches what we
//      packed on the way in.
//   4. Push 5 packets back-to-back; STATUS tracks level correctly.
//   5. Drain all 5 with pop_ACK and compare against the golden array.
//   6. CONTROL register write/read roundtrip.

module avalon_interface_tb;

    localparam int NUM_PACKETS = 5;

    // Address map (must match avalon_interface.sv and avalon_kernel.h)
    localparam logic [6:0] ADDR_PACKET_LO = 7'h00;
    localparam logic [6:0] ADDR_COMMIT    = 7'h11;
    localparam logic [6:0] ADDR_STATUS    = 7'h12;
    localparam logic [6:0] ADDR_CONTROL   = 7'h13;

    // -------- DUT signals --------
    logic             clk;
    logic             rst;

    logic [6:0]       avalon_address;
    logic             avalon_write;
    logic [31:0]      avalon_writedata;
    logic [31:0]      avalon_readdata;

    logic             pop;
    logic             pop_available;
    triangle_packet_t pop_data;
    logic             pop_ACK;

    logic             fifo_full;
    logic             fifo_empty;
    logic [5:0]       fifo_level;

    avalon_interface dut (
        .clk              (clk),
        .rst              (rst),
        .avalon_address   (avalon_address),
        .avalon_write     (avalon_write),
        .avalon_writedata (avalon_writedata),
        .avalon_readdata  (avalon_readdata),
        .pop              (pop),
        .pop_available    (pop_available),
        .pop_data         (pop_data),
        .pop_ACK          (pop_ACK),
        .fifo_full        (fifo_full),
        .fifo_empty       (fifo_empty),
        .fifo_level       (fifo_level)
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

    // -----------------------------------------------------------------
    // Avalon bus drivers - one task per kind of transaction so the test
    // body reads at the level of "send packet word" rather than per-pin
    // -----------------------------------------------------------------
    task automatic avalon_write_word(input logic [6:0] addr,
                                     input logic [31:0] data);
        avalon_address  <= addr;
        avalon_writedata <= data;
        avalon_write    <= 1'b1;
        @(posedge clk);
        avalon_write    <= 1'b0;
        avalon_address  <= '0;
        avalon_writedata <= '0;
    endtask

    task automatic avalon_read_word(input logic [6:0] addr,
                                    output logic [31:0] data);
        avalon_address <= addr;
        avalon_write   <= 1'b0;
        @(posedge clk);
        // readdata is combinational off avalon_address, so the value is
        // valid in the same cycle. Sample it and then deassert the addr.
        data = avalon_readdata;
        avalon_address <= '0;
    endtask

    // Build the same packet on the software side: pack the 17 32-bit words
    // matching how rasterizer_packet_t is laid out in avalon_kernel.h.
    // This pair (make_packet_words, expected_packet) lets us send via the
    // Avalon bus and compare against the SV struct that should pop out.
    function automatic void make_packet_words(input int idx,
                                              output logic [31:0] words [17]);
        // edge coefs - small, distinguishable values
        words[ 0] = 32'(idx * 32'h1000 + 32'h0001);   // a0
        words[ 1] = 32'(idx * 32'h1000 + 32'h0002);   // b0
        words[ 2] = 32'(idx * 32'h1000 + 32'h0003);   // c0 (dropped in HW)
        words[ 3] = 32'(idx * 32'h1000 + 32'h0004);   // a1
        words[ 4] = 32'(idx * 32'h1000 + 32'h0005);   // b1
        words[ 5] = 32'(idx * 32'h1000 + 32'h0006);   // c1 (dropped)
        words[ 6] = 32'(idx * 32'h1000 + 32'h0007);   // a2
        words[ 7] = 32'(idx * 32'h1000 + 32'h0008);   // b2
        words[ 8] = 32'(idx * 32'h1000 + 32'h0009);   // c2 (dropped)
        words[ 9] = 32'(idx * 32'h1000 + 32'h000A);   // e0_init
        words[10] = 32'(idx * 32'h1000 + 32'h000B);   // e1_init
        words[11] = 32'(idx * 32'h1000 + 32'h000C);   // e2_init
        words[12] = 32'(idx * 32'h1000 + 32'h000D);   // z_origin
        words[13] = 32'(idx * 32'h1000 + 32'h000E);   // z_step_x
        words[14] = 32'(idx * 32'h1000 + 32'h000F);   // z_step_y
        // bbox_packed: { ymax, ymin, xmax, xmin }
        words[15] = {8'(idx + 8'd40),    // ymax
                     8'(idx + 8'd10),    // ymin
                     8'(idx + 8'd50),    // xmax
                     8'(idx + 8'd20)};   // xmin
        // flags_color: { ..., front_facing, color }
        words[16] = {23'd0, 1'b1, 8'(idx + 8'h80)};
    endfunction

    function automatic triangle_packet_t expected_packet(input int idx);
        triangle_packet_t p;
        p              = '0;
        p.bbox_xmin    = 10'(idx + 20);
        p.bbox_xmax    = 10'(idx + 50);
        p.bbox_ymin    = 9'(idx + 10);
        p.bbox_ymax    = 9'(idx + 40);
        p.a0           = 32'(idx * 32'h1000 + 32'h0001);
        p.b0           = 32'(idx * 32'h1000 + 32'h0002);
        p.a1           = 32'(idx * 32'h1000 + 32'h0004);
        p.b1           = 32'(idx * 32'h1000 + 32'h0005);
        p.a2           = 32'(idx * 32'h1000 + 32'h0007);
        p.b2           = 32'(idx * 32'h1000 + 32'h0008);
        p.e0_init      = 32'(idx * 32'h1000 + 32'h000A);
        p.e1_init      = 32'(idx * 32'h1000 + 32'h000B);
        p.e2_init      = 32'(idx * 32'h1000 + 32'h000C);
        p.z_at_origin  = 32'(idx * 32'h1000 + 32'h000D);
        p.z_step_x     = 32'(idx * 32'h1000 + 32'h000E);
        p.z_step_y     = 32'(idx * 32'h1000 + 32'h000F);
        p.color        = 8'(idx + 8'h80);
        p.front_facing = 1'b1;
        return p;
    endfunction

    // Send one packet over the bus: 17 word writes + COMMIT
    task automatic submit_packet(input int idx);
        logic [31:0] words [17];
        make_packet_words(idx, words);
        for (int i = 0; i < 17; i++) begin
            avalon_write_word(7'(i), words[i]);
        end
        avalon_write_word(ADDR_COMMIT, 32'h1);
    endtask

    // Pop one packet via the dispatcher-style handshake and capture it
    task automatic pop_one(output triangle_packet_t got);
        // pop is a request signal we hold high while waiting
        pop <= 1'b1;
        // wait until the interface says data is ready
        while (!pop_available) @(posedge clk);
        // sample combinationally, then ACK to advance the FIFO
        got     = pop_data;
        pop_ACK <= 1'b1;
        @(posedge clk);
        pop_ACK <= 1'b0;
        pop     <= 1'b0;
    endtask

    // -----------------------------------------------------------------
    // Test sequence
    // -----------------------------------------------------------------
    triangle_packet_t got_pkts [NUM_PACKETS];
    triangle_packet_t exp_pkts [NUM_PACKETS];
    logic [31:0]      rdata;

    initial begin
        // ---- reset ----
        rst              = 1;
        avalon_address   = '0;
        avalon_write     = 1'b0;
        avalon_writedata = '0;
        pop              = 1'b0;
        pop_ACK          = 1'b0;
        repeat (4) @(posedge clk);
        rst = 0;
        @(posedge clk);

        // ---- 1. status after reset ----
        avalon_read_word(ADDR_STATUS, rdata);
        check(rdata[5:0] == 6'd0, "level 0 after reset");
        check(rdata[6]   == 1'b1, "empty bit set after reset");
        check(rdata[7]   == 1'b0, "full bit clear after reset");
        check(fifo_empty,         "fifo_empty output high after reset");
        check(!fifo_full,         "fifo_full output low after reset");

        // ---- 2. push one packet, status reflects it ----
        submit_packet(0);
        @(posedge clk);
        avalon_read_word(ADDR_STATUS, rdata);
        check(rdata[5:0] == 6'd1, "level 1 after one commit");
        check(rdata[6]   == 1'b0, "empty clear after commit");
        check(pop_available,      "pop_available high after commit");

        // ---- 3. pop it back, content matches ----
        pop_one(got_pkts[0]);
        exp_pkts[0] = expected_packet(0);
        check(got_pkts[0] === exp_pkts[0],
              "packet 0 round-trip content matches");

        @(posedge clk);
        avalon_read_word(ADDR_STATUS, rdata);
        check(rdata[5:0] == 6'd0, "level back to 0 after pop");
        check(rdata[6]   == 1'b1, "empty bit set after pop");

        // ---- 4. push NUM_PACKETS back-to-back, level tracks ----
        for (int i = 0; i < NUM_PACKETS; i++) begin
            submit_packet(i);
        end
        @(posedge clk);
        avalon_read_word(ADDR_STATUS, rdata);
        check(rdata[5:0] == 6'(NUM_PACKETS),
              $sformatf("level %0d after %0d commits", rdata[5:0], NUM_PACKETS));

        // ---- 5. drain and verify FIFO order ----
        for (int i = 0; i < NUM_PACKETS; i++) begin
            pop_one(got_pkts[i]);
            exp_pkts[i] = expected_packet(i);
            check(got_pkts[i] === exp_pkts[i],
                  $sformatf("packet %0d order/content mismatch", i));
        end

        @(posedge clk);
        avalon_read_word(ADDR_STATUS, rdata);
        check(rdata[5:0] == 6'd0, "level 0 after full drain");
        check(rdata[6]   == 1'b1, "empty after full drain");

        // ---- 6. control register write/read roundtrip ----
        avalon_write_word(ADDR_CONTROL, 32'hDEAD_BEEF);
        @(posedge clk);
        avalon_read_word(ADDR_CONTROL, rdata);
        check(rdata == 32'hDEAD_BEEF, "control register roundtrip");

        if (errors == 0)
            $display("PASS avalon_interface_tb: %0d packets verified", NUM_PACKETS);
        else
            $display("FAIL avalon_interface_tb: %0d errors", errors);
        $finish;
    end

    initial begin
        #50000;
        $error("avalon_interface_tb timeout");
        $finish;
    end

endmodule