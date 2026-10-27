`timescale 1ns/1ps
module lesson3_reset_sync(input wire clk, input wire request, output wire rst);
    reg [2:0] pipe = 3'b111;
    always @(posedge clk or posedge request)
        if (request) pipe <= 3'b111;
        else pipe <= {pipe[1:0], 1'b0};
    assign rst = pipe[2];
endmodule

// Bundled-data request/acknowledge CDC: snapshot held until destination ack.
// Request passes through two flops before the held bus is sampled.
module lesson3_media_cdc (
    input wire source_clk, source_rst,
    input wire [2:0] source_data,
    input wire pixel_clk, pixel_rst,
    output reg [2:0] pixel_data
);
    reg [2:0] held_data;
    reg request_toggle, acknowledge;
    reg [1:0] ack_sync, req_sync;
    always @(posedge source_clk or posedge source_rst) begin
        if (source_rst) begin
            held_data <= 0; request_toggle <= 0; ack_sync <= 0;
        end else begin
            ack_sync <= {ack_sync[0], acknowledge};
            if ((request_toggle == ack_sync[1]) && (held_data != source_data)) begin
                held_data <= source_data;
                request_toggle <= ~request_toggle;
            end
        end
    end
    always @(posedge pixel_clk or posedge pixel_rst) begin
        if (pixel_rst) begin
            pixel_data <= 0; acknowledge <= 0; req_sync <= 0;
        end else begin
            req_sync <= {req_sync[0], request_toggle};
            if (req_sync[1] != acknowledge) begin
                pixel_data <= held_data;
                acknowledge <= req_sync[1];
            end
        end
    end
endmodule

// Reconstruct the full horizontal coordinate from aligned negative HS.
// For 640 active + 16 front porch, the falling HS edge is coordinate 656.
// Combinational edge correction avoids a one-pixel offset at that edge.
module lesson3_video_x (
    input wire clk, rst, hs,
    output wire [9:0] video_x,
    output reg locked
);
    reg hs_q;
    reg [9:0] next_x;
    wire falling_hs = hs_q && !hs;
    assign video_x = falling_hs ? 10'd656 : next_x;
    always @(posedge clk or posedge rst) begin
        if (rst) begin hs_q <= 0; next_x <= 0; locked <= 0; end
        else begin
            hs_q <= hs;
            if (falling_hs) begin next_x <= 10'd657; locked <= 1; end
            else if (next_x == 10'd799) next_x <= 0;
            else next_x <= next_x + 1'b1;
        end
    end
endmodule
