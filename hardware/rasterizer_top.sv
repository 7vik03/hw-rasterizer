// rasterizer_top.sv

`include "triangle_packet.svh"


module rasterizer_top (
    input logic clk, rst,
    input logic [6:0] avs_address,
    input logic avs_write,
    input logic [31:0] avs_writedata,
    output logic [31:0] avs_readdata,
    output logic avs_waitrequest,


    output logic [7:0] VGA_R, VGA_G, VGA_B,
    output logic VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_n, VGA_SYNC_n
);


    assign avs_waitrequest=1'b0;

    localparam int N_PU=16;


    logic disp_pop, disp_pop_available;
    triangle_packet_t disp_pop_data;
    logic disp_pop_ACK, fifo_full, fifo_empty;
    logic [5:0] fifo_level;
    logic present_req, present_pending;


    logic swap_busy;


    avalon_interface u_avalon (
        .clk (clk), .rst (rst), .avalon_address (avs_address), .avalon_write (avs_write),
        .avalon_writedata (avs_writedata), .avalon_readdata (avs_readdata), .pop (disp_pop),
        .pop_available (disp_pop_available), .pop_data (disp_pop_data), .pop_ACK (disp_pop_ACK),
        .fifo_full (fifo_full), .fifo_empty (fifo_empty), .fifo_level (fifo_level),
        .present_req (present_req), .swap_busy (swap_busy)
    );

    logic [N_PU-1:0] pu_valid_seed;
    triangle_packet_t pu_packet;
    logic [N_PU-1:0] pu_ready;


    logic block_dispatch;

    triangle_dispatcher #(.N_PU(N_PU)) u_dispatcher (
        .clk (clk), .rst (rst), .pop (disp_pop), .pop_available (disp_pop_available),
        .pop_data (disp_pop_data), .pop_ACK (disp_pop_ACK), .valid_out (pu_valid_seed),
        .packet_out (pu_packet), .ready_in (pu_ready), .block_dispatch(block_dispatch)
    );


    logic chain_idle;
    assign chain_idle = &pu_ready;


    logic chain_seed_valid [N_PU];
    logic signed [31:0] chain_seed_e0 [N_PU], chain_seed_e1 [N_PU], chain_seed_e2 [N_PU], chain_seed_z [N_PU];


    assign chain_seed_valid[0] = pu_valid_seed[0];
    assign chain_seed_e0[0] = pu_packet.e0_init;
    assign chain_seed_e1[0] = pu_packet.e1_init;
    assign chain_seed_e2[0] = pu_packet.e2_init;
    assign chain_seed_z [0] = pu_packet.z_at_origin;


    logic [11:0] vga_r_addr;
    logic vga_r_buf_sel;
    logic fb_write_sel;
    logic [7:0] pu_vga_rdata [N_PU];


    logic z_clear_start;


    logic [N_PU-1:0] pu_p_write;
    logic [7:0] pu_p_col [N_PU],  pu_p_row [N_PU], pu_p_color [N_PU];
    logic [15:0] pu_p_depth [N_PU];

    genvar gi;
    generate
        for (gi = 0; gi < N_PU; gi++) begin : g_pu

            logic sv_out;
            logic signed [31:0] se0_out, se1_out, se2_out, sz_out;


            logic [7:0] col_base_for_pu;
            assign col_base_for_pu=pu_packet.bbox_xmin[7:0] + 8'(gi);

            pixel_unit #(.PU_ID (gi), .IS_LAST_PU ((gi == N_PU - 1) ? 1'b1 : 1'b0))
            u_pu (
                .clk (clk), .rst (rst), .ready (pu_ready[gi]), .z_clear_start (z_clear_start), .seed_valid_in (chain_seed_valid[gi]),
                .seed_e0_in (chain_seed_e0[gi]), .seed_e1_in (chain_seed_e1[gi]), .seed_e2_in (chain_seed_e2[gi]), .seed_z_in (chain_seed_z[gi]),
                .a0_in (pu_packet.a0), .a1_in (pu_packet.a1), .a2_in (pu_packet.a2), .z_step_x_in (pu_packet.z_step_x),
                .b0_in (pu_packet.b0), .b1_in (pu_packet.b1), .b2_in (pu_packet.b2), .z_step_y_in (pu_packet.z_step_y), .color_in (pu_packet.color),
                .col_base_in (col_base_for_pu), .row_base_in (pu_packet.bbox_ymin[7:0]), .last_row_in (pu_packet.bbox_ymax[7:0]), .last_col_in (pu_packet.bbox_xmax[7:0]),
                .seed_valid_out (sv_out), .seed_e0_out (se0_out), .seed_e1_out (se1_out), .seed_e2_out (se2_out), .seed_z_out (sz_out),
                .p_write (pu_p_write[gi]), .p_col (pu_p_col[gi]), .p_row (pu_p_row[gi]), .p_color (pu_p_color[gi]), .p_depth (pu_p_depth[gi]),
                .vga_r_addr (vga_r_addr), .vga_r_buf_sel (vga_r_buf_sel), .vga_r_data (pu_vga_rdata[gi]), .fb_write_sel (fb_write_sel)
            );


            if (gi == N_PU-1) begin : g_chain_tail


            end else begin : g_chain
                assign chain_seed_valid[gi + 1]=sv_out;
                assign chain_seed_e0 [gi + 1]=se0_out;
                assign chain_seed_e1 [gi + 1]=se1_out;
                assign chain_seed_e2 [gi + 1]=se2_out;
                assign chain_seed_z [gi + 1]=sz_out;
            end
        end
    endgenerate


    localparam int CLEAR_CYCLES = 4100;

    typedef enum logic [1:0] {
        SW_IDLE,
        SW_ARM,
        SW_CLEARING
    } swap_state_t;

    swap_state_t sw_state;
    logic [12:0] clear_cnt;
    logic frame_done;


    assign block_dispatch=(sw_state != SW_IDLE);


    assign swap_busy=(sw_state != SW_IDLE) || present_pending;

    always_ff @(posedge clk) begin
        z_clear_start<=1'b0;

        if (rst) begin
            sw_state <= SW_IDLE;
            fb_write_sel <= 1'b0;
            clear_cnt <= '0;
            present_pending<=1'b0;
        end

        else begin
            if (present_req)
                present_pending<=1'b1;
            unique case (sw_state)
                SW_IDLE: begin
                    if (present_pending && frame_done && fifo_empty) sw_state <= SW_ARM;
                end

                SW_ARM: begin
                    if (chain_idle) begin
                        fb_write_sel <= ~fb_write_sel;
                        z_clear_start<=1'b1;
                        clear_cnt <= 13'(CLEAR_CYCLES);
                        present_pending<=1'b0;
                        sw_state <= SW_CLEARING;
                    end
                end

                SW_CLEARING: begin
                    if (clear_cnt == 0) sw_state <= SW_IDLE;
                    else clear_cnt <= clear_cnt-13'd1;
                end

                default: sw_state<=SW_IDLE;
            endcase
        end
    end

    vga_framebuffer u_vga (
        .clk (clk), .reset (rst), .pu_vga_data (pu_vga_rdata), .vga_r_addr (vga_r_addr), .vga_r_buf_sel(vga_r_buf_sel),
        .fb_write_sel (fb_write_sel), .frame_done (frame_done), .VGA_R (VGA_R), .VGA_G (VGA_G), .VGA_B (VGA_B),
        .VGA_CLK (VGA_CLK), .VGA_HS (VGA_HS), .VGA_VS (VGA_VS), .VGA_BLANK_n (VGA_BLANK_n), .VGA_SYNC_n (VGA_SYNC_n)
    );

endmodule
