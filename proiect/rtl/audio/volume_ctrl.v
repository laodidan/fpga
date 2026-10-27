`timescale 1ns/1ps
module volume_ctrl (
    input wire clk,
    input wire rst_n,
    input wire volume_key_event,
    input wire volume_init_valid,
    input wire [7:0] volume_init_value,
    output reg volume_init_applied,
    output reg [7:0] volume_level,
    input wire sample_valid_in,
    output wire sample_ready_in,
    input wire signed [15:0] pcm_left_in,
    input wire signed [15:0] pcm_right_in,
    output reg sample_valid_out,
    input wire sample_ready_out,
    output reg signed [15:0] pcm_left_out,
    output reg signed [15:0] pcm_right_out
);
    wire signed [23:0] pcm_left_ext = {{8{pcm_left_in[15]}}, pcm_left_in};
    wire signed [23:0] pcm_right_ext = {{8{pcm_right_in[15]}}, pcm_right_in};
    wire signed [23:0] volume_ext = {16'd0, volume_level};
    reg signed [23:0] left_product_stage1, right_product_stage1;
    reg [23:0] left_magnitude_stage2, right_magnitude_stage2;
    reg left_sign_stage2, right_sign_stage2;
    reg [15:0] left_quotient_stage3, right_quotient_stage3;
    reg left_sign_stage3, right_sign_stage3;
    reg valid_stage1, valid_stage2, valid_stage3;

    wire ready_stage4 = !sample_valid_out || sample_ready_out;
    wire ready_stage3 = !valid_stage3 || ready_stage4;
    wire ready_stage2 = !valid_stage2 || ready_stage3;
    wire ready_stage1 = !valid_stage1 || ready_stage2;
    wire signed [16:0] left_signed_magnitude = {1'b0, left_quotient_stage3};
    wire signed [16:0] right_signed_magnitude = {1'b0, right_quotient_stage3};
    wire signed [16:0] left_restored = left_sign_stage3 ?
        -left_signed_magnitude : left_signed_magnitude;
    wire signed [16:0] right_restored = right_sign_stage3 ?
        -right_signed_magnitude : right_signed_magnitude;
    reg init_seen;

    // Exact floor(m/255) for 0 <= m <= 32768*255. The reciprocal
    // estimate is at most one low, so one remainder correction suffices.
    function [15:0] divide_magnitude_by_255;
        input [23:0] magnitude;
        reg [31:0] magnitude_ext, reciprocal_product;
        reg [15:0] q0;
        reg [23:0] q0_ext, remainder;
        begin
            magnitude_ext = {8'd0, magnitude};
            reciprocal_product = (magnitude_ext << 8) + magnitude_ext;
            q0 = reciprocal_product[31:16];
            q0_ext = {8'd0, q0};
            remainder = magnitude - ((q0_ext << 8) - q0_ext);
            divide_magnitude_by_255 = q0 +
                ((remainder >= 24'd255) ? 16'd1 : 16'd0);
        end
    endfunction

    assign sample_ready_in = rst_n && ready_stage1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            volume_level <= 8'd255;
            volume_init_applied <= 1'b0;
            init_seen <= 1'b0;
            valid_stage1 <= 1'b0;
            valid_stage2 <= 1'b0;
            valid_stage3 <= 1'b0;
            left_product_stage1 <= 24'sd0;
            right_product_stage1 <= 24'sd0;
            left_magnitude_stage2 <= 24'd0;
            right_magnitude_stage2 <= 24'd0;
            left_sign_stage2 <= 1'b0;
            right_sign_stage2 <= 1'b0;
            left_quotient_stage3 <= 16'd0;
            right_quotient_stage3 <= 16'd0;
            left_sign_stage3 <= 1'b0;
            right_sign_stage3 <= 1'b0;
            sample_valid_out <= 1'b0;
            pcm_left_out <= 16'sd0;
            pcm_right_out <= 16'sd0;
        end else begin
            volume_init_applied <= 1'b0;
            if (!volume_init_valid) init_seen <= 1'b0;
            if (volume_init_valid) begin
                if (!init_seen) begin
                    volume_level <= volume_init_value;
                    volume_init_applied <= 1'b1;
                    init_seen <= 1'b1;
                end
            end else if (volume_key_event) begin
                case (volume_level)
                    8'd255: volume_level <= 8'd0;
                    8'd0: volume_level <= 8'd64;
                    8'd64: volume_level <= 8'd128;
                    8'd128: volume_level <= 8'd192;
                    8'd192: volume_level <= 8'd255;
                    default: volume_level <= 8'd255;
                endcase
            end

            if (ready_stage4) begin
                sample_valid_out <= valid_stage3;
                if (valid_stage3) begin
                    pcm_left_out <= left_restored[15:0];
                    pcm_right_out <= right_restored[15:0];
                end
            end
            if (ready_stage3) begin
                valid_stage3 <= valid_stage2;
                if (valid_stage2) begin
                    left_quotient_stage3 <= divide_magnitude_by_255(left_magnitude_stage2);
                    right_quotient_stage3 <= divide_magnitude_by_255(right_magnitude_stage2);
                    left_sign_stage3 <= left_sign_stage2;
                    right_sign_stage3 <= right_sign_stage2;
                end
            end
            if (ready_stage2) begin
                valid_stage2 <= valid_stage1;
                if (valid_stage1) begin
                    left_magnitude_stage2 <= left_product_stage1[23] ?
                        -left_product_stage1 : left_product_stage1;
                    right_magnitude_stage2 <= right_product_stage1[23] ?
                        -right_product_stage1 : right_product_stage1;
                    left_sign_stage2 <= left_product_stage1[23];
                    right_sign_stage2 <= right_product_stage1[23];
                end
            end
            if (ready_stage1) begin
                valid_stage1 <= sample_valid_in;
                if (sample_valid_in) begin
                    // Capture the pre-edge volume, even on a control update.
                    left_product_stage1 <= pcm_left_ext * volume_ext;
                    right_product_stage1 <= pcm_right_ext * volume_ext;
                end
            end
        end
    end
endmodule
