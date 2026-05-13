module vga_framebuffer (
    input logic clk, reset,
    input logic [7:0] pu_vga_data [0:15],
    output logic [11:0] vga_r_addr,
    output logic vga_r_buf_sel,
    input logic fb_write_sel,
    output logic frame_done,
    output logic [7:0] VGA_R, VGA_G, VGA_B,
    output logic VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_n, VGA_SYNC_n);

    logic [10:0] hcount;
    logic [9:0] vcount;
    
    vga_counters counters (.clk50(clk), .reset(reset), .hcount(hcount), .vcount(vcount), .frame_done(frame_done), .VGA_CLK(VGA_CLK), 
                           .VGA_HS(VGA_HS), .VGA_VS(VGA_VS), .VGA_BLANK_n(VGA_BLANK_n), .VGA_SYNC_n(VGA_SYNC_n));

    logic [10:0] fb_x_tmp;
    logic [7:0] fb_x;
    logic [7:0] fb_y;
    logic in_fb_region;

    assign in_fb_region = (hcount >= 11'd64)  &&
                           (hcount <  11'd576) &&
                          (vcount <  10'd480);

    assign fb_x_tmp = (hcount - 11'd64)>> 1;
    assign fb_x = fb_x_tmp[7:0];

    assign fb_y = vcount[8:1];
    assign vga_r_addr = {fb_x[7:4], fb_y};
    
//vga will read front buffer rast writes back
    assign vga_r_buf_sel = ~fb_write_sel;

    logic [3:0] pu_sel_q;
    logic in_fb_region_q;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            pu_sel_q <= 4'd0;
            in_fb_region_q <= 1'b0;
        end 
        else begin

            //bram latency need to delay selector
        pu_sel_q <= fb_x[3:0];
        in_fb_region_q<= in_fb_region;
        end
    end

    logic [7:0] pixelcolor;
    assign pixelcolor =pu_vga_data[pu_sel_q];

    always_comb begin
        VGA_R = 8'h00;
        VGA_G=8'h00;
        VGA_B = 8'h00;

        if (VGA_BLANK_n && in_fb_region_q) begin
            VGA_R ={pixelcolor[7:5], pixelcolor[7:5], pixelcolor[7:6]};
            VGA_G= {pixelcolor[4:2], pixelcolor[4:2], pixelcolor[4:3]};
            VGA_B = {pixelcolor[1:0], pixelcolor[1:0], pixelcolor[1:0], pixelcolor[1:0]};
      end
 end
endmodule


module vga_counters (
    input logic clk50, reset,
    output logic [10:0] hcount,
    output logic [9:0]  vcount,
    output logic frame_done, VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_n, VGA_SYNC_n
);

    logic pixel_tick;

    always_ff @(posedge clk50 or posedge reset) begin
        if (reset) begin
            pixel_tick <= 1'b0;
            hcount <= 11'd0;
            vcount <= 10'd0;
            frame_done <= 1'b0;
        end 
        else begin
            pixel_tick <= ~pixel_tick;
            frame_done <= 1'b0;

            if (pixel_tick) begin
                if (hcount == 11'd799) begin
                    hcount <= 11'd0;

                    if (vcount == 10'd524) begin
                        vcount <= 10'd0;
                        frame_done <= 1'b1;
                    end 
                    
                    else begin
                        vcount <= vcount + 10'd1;
                    end
                end 
                else begin
                    hcount <= hcount + 11'd1;
                end
         end
     end
    end

assign VGA_CLK = pixel_tick;
assign VGA_HS = ~((hcount >= 11'd656) && (hcount < 11'd752));
assign VGA_VS = ~((vcount >= 10'd490) && (vcount < 10'd492));
assign VGA_BLANK_n = (hcount < 11'd640) && (vcount < 10'd480);
assign VGA_SYNC_n = 1'b0;

endmodule
