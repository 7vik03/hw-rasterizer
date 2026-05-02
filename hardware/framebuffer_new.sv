module framebuffer (
    input  logic clk,
    input  logic resetn,
    input  logic frame_done,       // from top level when all PUs done

    // Broadcast to all 16 pixel units
    output logic        fb_write_sel,   // 0=write fb_a  1=write fb_b
    output logic        vga_r_buf_sel,  // 0=VGA reads fb_a  1=reads fb_b
    output logic        clear_en,       // high each cycle of clear pass
    output logic [11:0] clear_addr,     // address to zero (0-4095)
    output logic        clearing,       // high while clear pass running
    output logic        swap_done,      // 1-cycle pulse after swap
    output logic        buf_id          // = fb_write_sel
);

    logic back_buf;

    typedef enum logic {
        CLEARING_BUF = 1'b0,
        READY        = 1'b1
    } state_t;
    state_t state;

    // rasterizer always writes to back buffer
    // VGA always reads from front buffer (opposite)
    assign fb_write_sel  = back_buf;
    assign vga_r_buf_sel = ~back_buf;
    assign buf_id        = back_buf;

    always_ff @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            back_buf   <= 0;
            swap_done  <= 0;
            clearing   <= 1;
            clear_en   <= 0;
            clear_addr <= 12'h0;
            state      <= CLEARING_BUF;
        end else begin
            swap_done <= 0;
            clear_en  <= 0;

            case (state)
                CLEARING_BUF: begin
                    // Broadcast clear address to all 16 PUs simultaneously
                    // Each PU zeros its own slice at this address
                    clear_en <= 1;
                    if (clear_addr == 12'd4095) begin
                        clear_addr <= 12'h0;
                        clearing   <= 0;
                        clear_en   <= 0;
                        state      <= READY;
                    end else begin
                        clear_addr <= clear_addr + 1;
                    end
                end

                READY: begin
                    if (frame_done) begin
                        back_buf   <= ~back_buf;
                        swap_done  <= 1;
                        clear_addr <= 12'h0;
                        clearing   <= 1;
                        state      <= CLEARING_BUF;
                    end
                end

                default: state <= CLEARING_BUF;
            endcase
        end
    end

endmodule
