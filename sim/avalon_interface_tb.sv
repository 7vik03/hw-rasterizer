`timescale 1ns/1ps
`include "triangle_packet.svh"

// avalon_interface_tb (DEBUG VERSION)
// ----------------------------------
// Same test plan as before but loud:
//   - every Avalon write logs addr + data + cycle
//   - every FIFO push logs the captured packet
//   - every FIFO pop logs the data flowing out
//   - every status read logs the raw word
//   - mismatches print field-by-field diffs

module avalon_interface_tb;

    localparam int NUM_PACKETS = 5;

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

    // -------- cycle counter for debug output --------
    int cycle;
    always_ff @(posedge clk) begin
        if (rst) cycle <= 0;
        else     cycle <= cycle + 1;
    end

    int errors = 0;
    task automatic check(input bit cond, input string msg);
        if (!cond) begin
            $error("FAIL: %s", msg);
            errors++;
        end
    endtask

    // -----------------------------------------------------------------
    // SPY 1: every clock edge where we see a bus transaction, log it
    // -----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst && avalon_write) begin
            if (avalon_address >= ADDR_PACKET_LO && avalon_address <= 7'h10) begin
                $display("[cyc=%0d t=%0t] BUS WR  stage[%0d] <= %h",
                         cycle, $time, avalon_address[4:0], avalon_writedata);
            end
            else if (avalon_address == ADDR_COMMIT) begin
                $display("[cyc=%0d t=%0t] BUS WR  COMMIT  (data=%h, fifo_level=%0d, full=%b)",
                         cycle, $time, avalon_writedata, fifo_level, fifo_full);
            end
            else if (avalon_address == ADDR_CONTROL) begin
                $display("[cyc=%0d t=%0t] BUS WR  CONTROL <= %h",
                         cycle, $time, avalon_writedata);
            end
            else begin
                $display("[cyc=%0d t=%0t] BUS WR  addr=%h data=%h (UNMAPPED)",
                         cycle, $time, avalon_address, avalon_writedata);
            end
        end
    end

    // -----------------------------------------------------------------
    // SPY 2: every FIFO push, log the packet captured
    // dut.fifo_push and dut.fifo_push_data are internal hierarchical
    // references - these only work when the DUT exposes them by name.
    // -----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst && dut.fifo_push) begin
            $display("[cyc=%0d t=%0t] FIFO PUSH",
                     cycle, $time);
            $display("                a0=%h  b0=%h", dut.fifo_push_data.a0, dut.fifo_push_data.b0);
            $display("                a1=%h  b1=%h", dut.fifo_push_data.a1, dut.fifo_push_data.b1);
            $display("                a2=%h  b2=%h", dut.fifo_push_data.a2, dut.fifo_push_data.b2);
            $display("                e0_init=%h  e1_init=%h  e2_init=%h",
                     dut.fifo_push_data.e0_init, dut.fifo_push_data.e1_init, dut.fifo_push_data.e2_init);
            $display("                z_at_origin=%h  z_step_x=%h  z_step_y=%h",
                     dut.fifo_push_data.z_at_origin, dut.fifo_push_data.z_step_x, dut.fifo_push_data.z_step_y);
            $display("                bbox xmin=%0d xmax=%0d ymin=%0d ymax=%0d",
                     dut.fifo_push_data.bbox_xmin, dut.fifo_push_data.bbox_xmax,
                     dut.fifo_push_data.bbox_ymin, dut.fifo_push_data.bbox_ymax);
            $display("                color=%h front_facing=%b",
                     dut.fifo_push_data.color, dut.fifo_push_data.front_facing);
        end
    end

    // -----------------------------------------------------------------
    // SPY 3: every cycle pop_ACK is asserted, log the pop
    // -----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst && pop_ACK) begin
            $display("[cyc=%0d t=%0t] FIFO POP (level was %0d)",
                     cycle, $time, fifo_level);
            $display("                a0=%h color=%h bbox_xmin=%0d ymin=%0d",
                     pop_data.a0, pop_data.color, pop_data.bbox_xmin, pop_data.bbox_ymin);
        end
    end

    // -----------------------------------------------------------------
    // SPY 4: edge-detect on fifo_level to catch all changes
    // -----------------------------------------------------------------
    logic [5:0] prev_level;
    always_ff @(posedge clk) begin
        if (rst) begin
            prev_level <= '0;
        end else begin
            if (fifo_level != prev_level) begin
                $display("[cyc=%0d t=%0t] LEVEL %0d -> %0d   (full=%b empty=%b)",
                         cycle, $time, prev_level, fifo_level, fifo_full, fifo_empty);
            end
            prev_level <= fifo_level;
        end
    end

    // -----------------------------------------------------------------
    // Avalon bus drivers
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
        data = avalon_readdata;
        $display("[cyc=%0d t=%0t] BUS RD  addr=%h -> %h",
                 cycle, $time, addr, data);
        avalon_address <= '0;
    endtask

    // -----------------------------------------------------------------
    // Packet builder + golden expected
    // -----------------------------------------------------------------
    function automatic void make_packet_words(input int idx,
                                              output logic [31:0] words [17]);
        words[ 0] = 32'(idx * 32'h1000 + 32'h0001);   // a0
        words[ 1] = 32'(idx * 32'h1000 + 32'h0002);   // b0
        words[ 2] = 32'(idx * 32'h1000 + 32'h0003);   // c0 (dropped)
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
        words[15] = {8'(idx + 8'd40),    // ymax
                     8'(idx + 8'd10),    // ymin
                     8'(idx + 8'd50),    // xmax
                     8'(idx + 8'd20)};   // xmin
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

    // -----------------------------------------------------------------
    // submit one packet over the bus
    // -----------------------------------------------------------------
    task automatic submit_packet(input int idx);
        logic [31:0] words [17];
        $display("=========================================================");
        $display("[cyc=%0d t=%0t] >>> SUBMIT PACKET idx=%0d", cycle, $time, idx);
        $display("=========================================================");
        make_packet_words(idx, words);
        for (int i = 0; i < 17; i++) begin
            avalon_write_word(7'(i), words[i]);
        end
        avalon_write_word(ADDR_COMMIT, 32'h1);
        $display("[cyc=%0d t=%0t] <<< SUBMIT PACKET idx=%0d done\n",
                 cycle, $time, idx);
    endtask

task automatic pop_one(output triangle_packet_t got);
    $display("=========================================================");
    $display("[cyc=%0d t=%0t] >>> POP_ONE  pop_available=%b empty=%b level=%0d",
             cycle, $time, pop_available, fifo_empty, fifo_level);
    pop <= 1'b1;
    while (!pop_available) @(posedge clk);

    // Sample BEFORE the ACK so we get the current head, then ACK to advance.
    // Critical: wait for #1 after the edge so any in-flight rd_ptr update
    // from a previous ACK has fully settled before we read pop_data.
    #1;
    got = pop_data;
    $display("[cyc=%0d t=%0t] POP captured: a0=%h color=%h xmin=%0d ymin=%0d",
             cycle, $time, got.a0, got.color, got.bbox_xmin, got.bbox_ymin);
    pop_ACK <= 1'b1;
    @(posedge clk);
    pop_ACK <= 1'b0;
    pop     <= 1'b0;
    $display("[cyc=%0d t=%0t] <<< POP_ONE done (now level=%0d empty=%b)\n",
             cycle, $time, fifo_level, fifo_empty);
endtask

    // -----------------------------------------------------------------
    // packet diff helper
    // -----------------------------------------------------------------
    task automatic diff_packet(input triangle_packet_t got,
                               input triangle_packet_t exp,
                               input int idx);
        $display("---- PACKET %0d DIFF ----", idx);
        if (got.a0          !== exp.a0)          $display("  a0           got=%h exp=%h", got.a0, exp.a0);
        if (got.b0          !== exp.b0)          $display("  b0           got=%h exp=%h", got.b0, exp.b0);
        if (got.a1          !== exp.a1)          $display("  a1           got=%h exp=%h", got.a1, exp.a1);
        if (got.b1          !== exp.b1)          $display("  b1           got=%h exp=%h", got.b1, exp.b1);
        if (got.a2          !== exp.a2)          $display("  a2           got=%h exp=%h", got.a2, exp.a2);
        if (got.b2          !== exp.b2)          $display("  b2           got=%h exp=%h", got.b2, exp.b2);
        if (got.e0_init     !== exp.e0_init)     $display("  e0_init      got=%h exp=%h", got.e0_init, exp.e0_init);
        if (got.e1_init     !== exp.e1_init)     $display("  e1_init      got=%h exp=%h", got.e1_init, exp.e1_init);
        if (got.e2_init     !== exp.e2_init)     $display("  e2_init      got=%h exp=%h", got.e2_init, exp.e2_init);
        if (got.z_at_origin !== exp.z_at_origin) $display("  z_at_origin  got=%h exp=%h", got.z_at_origin, exp.z_at_origin);
        if (got.z_step_x    !== exp.z_step_x)    $display("  z_step_x     got=%h exp=%h", got.z_step_x, exp.z_step_x);
        if (got.z_step_y    !== exp.z_step_y)    $display("  z_step_y     got=%h exp=%h", got.z_step_y, exp.z_step_y);
        if (got.bbox_xmin   !== exp.bbox_xmin)   $display("  bbox_xmin    got=%0d exp=%0d", got.bbox_xmin, exp.bbox_xmin);
        if (got.bbox_xmax   !== exp.bbox_xmax)   $display("  bbox_xmax    got=%0d exp=%0d", got.bbox_xmax, exp.bbox_xmax);
        if (got.bbox_ymin   !== exp.bbox_ymin)   $display("  bbox_ymin    got=%0d exp=%0d", got.bbox_ymin, exp.bbox_ymin);
        if (got.bbox_ymax   !== exp.bbox_ymax)   $display("  bbox_ymax    got=%0d exp=%0d", got.bbox_ymax, exp.bbox_ymax);
        if (got.color       !== exp.color)       $display("  color        got=%h exp=%h", got.color, exp.color);
        if (got.front_facing !== exp.front_facing) $display("  front_facing got=%b exp=%b", got.front_facing, exp.front_facing);
        $display("------------------------");
    endtask

    // -----------------------------------------------------------------
    // Test sequence
    // -----------------------------------------------------------------
    triangle_packet_t got_pkts [NUM_PACKETS];
    triangle_packet_t exp_pkts [NUM_PACKETS];
    logic [31:0]      rdata;

    initial begin
        rst              = 1;
        avalon_address   = '0;
        avalon_write     = 1'b0;
        avalon_writedata = '0;
        pop              = 1'b0;
        pop_ACK          = 1'b0;
        repeat (4) @(posedge clk);
        rst = 0;
        @(posedge clk);

        $display("\n#### TEST 1 - status after reset ####");
        avalon_read_word(ADDR_STATUS, rdata);
        check(rdata[5:0] == 6'd0, "level 0 after reset");
        check(rdata[6]   == 1'b1, "empty bit set after reset");
        check(rdata[7]   == 1'b0, "full bit clear after reset");
        check(fifo_empty,         "fifo_empty output high after reset");
        check(!fifo_full,         "fifo_full output low after reset");

        $display("\n#### TEST 2 - push one packet ####");
        submit_packet(0);
        @(posedge clk);
        avalon_read_word(ADDR_STATUS, rdata);
        check(rdata[5:0] == 6'd1, "level 1 after one commit");
        check(rdata[6]   == 1'b0, "empty clear after commit");
        check(pop_available,      "pop_available high after commit");

        $display("\n#### TEST 3 - pop the single packet ####");
        pop_one(got_pkts[0]);
        exp_pkts[0] = expected_packet(0);
        if (got_pkts[0] !== exp_pkts[0]) diff_packet(got_pkts[0], exp_pkts[0], 0);
        check(got_pkts[0] === exp_pkts[0], "packet 0 round-trip content matches");

        @(posedge clk);
        avalon_read_word(ADDR_STATUS, rdata);
        check(rdata[5:0] == 6'd0, "level back to 0 after pop");
        check(rdata[6]   == 1'b1, "empty bit set after pop");

        $display("\n#### TEST 4 - push %0d packets back-to-back ####", NUM_PACKETS);
        for (int i = 0; i < NUM_PACKETS; i++) begin
            submit_packet(i);
        end
        @(posedge clk);
        avalon_read_word(ADDR_STATUS, rdata);
        check(rdata[5:0] == 6'(NUM_PACKETS),
              $sformatf("level %0d after %0d commits", rdata[5:0], NUM_PACKETS));

        $display("\n#### TEST 5 - drain and verify order ####");
        for (int i = 0; i < NUM_PACKETS; i++) begin
            pop_one(got_pkts[i]);
            exp_pkts[i] = expected_packet(i);
            if (got_pkts[i] !== exp_pkts[i]) diff_packet(got_pkts[i], exp_pkts[i], i);
            check(got_pkts[i] === exp_pkts[i],
                  $sformatf("packet %0d order/content mismatch", i));
        end

        @(posedge clk);
        avalon_read_word(ADDR_STATUS, rdata);
        check(rdata[5:0] == 6'd0, "level 0 after full drain");
        check(rdata[6]   == 1'b1, "empty after full drain");

        $display("\n#### TEST 6 - control register roundtrip ####");
        avalon_write_word(ADDR_CONTROL, 32'hDEAD_BEEF);
        @(posedge clk);
        avalon_read_word(ADDR_CONTROL, rdata);
        check(rdata == 32'hDEAD_BEEF, "control register roundtrip");

        $display("\n=========================================================");
        if (errors == 0)
            $display("PASS avalon_interface_tb: %0d packets verified", NUM_PACKETS);
        else
            $display("FAIL avalon_interface_tb: %0d errors", errors);
        $display("=========================================================");
        $finish;
    end

    initial begin
        #100000;
        $error("avalon_interface_tb timeout");
        $finish;
    end

endmodule
