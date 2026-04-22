module framebuffer (clk, resetn, wr_en, wr_x, wr_y, wr_color, rd_x, rd_y, rd_color, frame_done, swap_done, clearing, buf_id);
    input logic clk, resetn, wr_en, frame_done;
    input logic [8:0] wr_x, rd_x;
    input logic [7:0] wr_y, wr_color, rd_y;
    output logic [7:0] rd_color;
    output logic clearing, buf_id, swap_done;


//frame_done tells us when all pixel units are swap_done
//swap_done needs to be wired to the pizel units so they know its safe to start writing next frame_done
//clearing pixel units must not write during clear pass
//buf_id need to know which address space to write into
//read ports from vga controller

//back_buf is 0, fb_b is in front, when 1 fb_a is in front
    logic [7:0] fb_a [0:76799];
    logic [7:0] fb_b [0:76799];
    logic back_buf;
    logic [16:0] wr_addr, rd_addr, clear_addr;

    typedef enum logic [1:0] {
        IDLE = 2'd0,
        CLEARING_BUF = 2'd1,
        READY = 2'd2
    } state_t;

    state_t state;

    assign wr_addr = (wr_y*320)+wr_x;
    assign rd_addr = (rd_y*320)+rd_x;
    assign buf_id = back_buf;

    always_ff @(posedge clk) begin
    //pixel write - gated on READY
    //wr_en gated with ~clearing at top level

        if(wr_en && state == READY) begin
            if(back_buf == 0)
                fb_a[wr_addr]<=wr_color;
            else
                fb_b[wr_addr]<=wr_color;
        end
    
    //clear pass one address per cycle, front buffer untouched during this, VGA reads nroamlly
        if (state == CLEARING_BUF) begin
            if(back_buf == 0)
                fb_a[clear_addr]<=8'h00;
            else
                fb_b[clear_addr]<=8'h00;
        end

    //reading from buffer  
        if (back_buf==1)
            rd_color<=fb_a[rd_addr];
        else
            rd_color<=fb_b[rd_addr];
    end

    always_ff @(posedge clk or negedge resetn) begin

        if(!resetn) begin
            back_buf<=0;
            swap_done<=0;
            clearing<=1;
            clear_addr<=17'h0;
            state<=CLEARING_BUF;
        end
        else begin
            swap_done<=0;
            case (state)
                IDLE: begin
                    if (frame_done) begin
                        back_buf<=~back_buf;
                        swap_done<=1;
                        clear_addr<=17'h0;
                        clearing<=1;
                        state<=CLEARING_BUF;
                    end
                end
                //increment clear_addr till alll address are zeros
                CLEARING_BUF: begin
                    if (clear_addr == 17'd76799) begin
                        clear_addr<=17'h0;
                        clearing<=0;
                        state<=READY;
                    end
                    else 
                        clear_addr<=clear_addr+1;
                end

                READY: begin
                    if (frame_done) begin
                        back_buf<=~back_buf;
                        swap_done<=1;
                        clear_addr<=17'h0;
                        clearing<=1;
                        state<=CLEARING_BUF;
                    end
                end

                default: state<=IDLE;
            endcase
        end
    end
endmodule



