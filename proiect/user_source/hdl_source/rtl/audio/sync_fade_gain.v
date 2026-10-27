`timescale 1ns/1ps
module sync_fade_gain (
    input wire clk,
    input wire rst_n,
    input wire signed [15:0] pcm_sample_in,
    input wire sample_valid_in,
    input wire [7:0] alpha_in,
    output reg signed [15:0] pcm_sample_out,
    output reg sample_valid_out
);
    (* async_reg = "true" *) reg [7:0] alpha_sync1;
    (* async_reg = "true" *) reg [7:0] alpha_sync2;

    reg signed [23:0] product_stage1;
    reg [23:0] magnitude_stage2;
    reg sign_stage2;
    reg [15:0] quotient_stage3;
    reg sign_stage3;
    reg valid_stage1;
    reg valid_stage2;
    reg valid_stage3;

    // Signed 24 bits cover every 16-bit PCM times unsigned 8-bit alpha.
    wire signed [23:0] pcm_ext = {{8{pcm_sample_in[15]}}, pcm_sample_in};
    wire signed [23:0] alpha_ext = {16'd0, alpha_sync2};
    wire signed [23:0] product_full = pcm_ext * alpha_ext;

    // Widen before shifting: m * 257 fits in unsigned 32 bits.
    wire [31:0] magnitude_ext = {8'd0, magnitude_stage2};
    wire [31:0] reciprocal_product = (magnitude_ext << 8) + magnitude_ext;
    wire [15:0] quotient_approx = reciprocal_product[31:16];
    wire [23:0] quotient_ext = {8'd0, quotient_approx};
    wire [23:0] quotient_times255 = (quotient_ext << 8) - quotient_ext;
    wire [23:0] remainder = magnitude_stage2 - quotient_times255;
    wire [15:0] quotient_corrected = quotient_approx +
        ((remainder >= 24'd255) ? 16'd1 : 16'd0);

    // Keep magnitude 32768 positive until the sign is restored.
    wire signed [16:0] quotient_signed = {1'b0, quotient_stage3};
    wire signed [16:0] scaled_signed =
        sign_stage3 ? -quotient_signed : quotient_signed;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            alpha_sync1 <= 8'd255;
            alpha_sync2 <= 8'd255;
            product_stage1 <= 24'sd0;
            magnitude_stage2 <= 24'd0;
            sign_stage2 <= 1'b0;
            quotient_stage3 <= 16'd0;
            sign_stage3 <= 1'b0;
            valid_stage1 <= 1'b0;
            valid_stage2 <= 1'b0;
            valid_stage3 <= 1'b0;
            pcm_sample_out <= 16'sd0;
            sample_valid_out <= 1'b0;
        end else begin
            alpha_sync1 <= alpha_in;
            alpha_sync2 <= alpha_sync1;
            valid_stage1 <= sample_valid_in;
            valid_stage2 <= valid_stage1;
            valid_stage3 <= valid_stage2;
            sample_valid_out <= valid_stage3;

            // Stage 1: sample the synchronized gain with its PCM sample.
            if (sample_valid_in)
                product_stage1 <= product_full;

            // Stage 2: absolute product and its sign travel together.
            if (valid_stage1) begin
                magnitude_stage2 <= product_stage1[23] ?
                    -product_stage1 : product_stage1;
                sign_stage2 <= product_stage1[23];
            end

            // Stage 3: exact unsigned divide-by-255 with one correction.
            if (valid_stage2) begin
                quotient_stage3 <= quotient_corrected;
                sign_stage3 <= sign_stage2;
            end

            // Stage 4: restore sign; retain the previous sample on bubbles.
            if (valid_stage3)
                pcm_sample_out <= scaled_signed[15:0];
        end
    end
endmodule
