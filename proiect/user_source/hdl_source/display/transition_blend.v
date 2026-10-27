`timescale 1ns/1ps

// The reader has already selected the old/new region for Wipe. Fade scales
// this one real pixel stream BEFORE OSD, with exact floor(channel*alpha/255).
// Combinational: no RGB/DE/HS/VS coordinate latency is introduced.
module transition_blend (
    input wire [23:0] rgb_in,
    input wire [7:0] alpha,
    output wire [23:0] rgb_out
);
    function [7:0] scale;
        input [7:0] channel;
        input [7:0] gain;
        reg [15:0] product;
        reg [16:0] quotient_numerator;
        begin
            product = channel * gain;
            quotient_numerator = {1'b0,product} + 17'd1 + {9'd0,product[15:8]};
            scale = quotient_numerator[15:8];
        end
    endfunction
    assign rgb_out = {scale(rgb_in[23:16],alpha),
                      scale(rgb_in[15:8],alpha), scale(rgb_in[7:0],alpha)};
endmodule
