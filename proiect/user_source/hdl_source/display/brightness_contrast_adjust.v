// RGB888 brightness and contrast adjustment.
// An input register isolates the transition-blend output from the first
// brightness multiplier. The input bundle and parameters then pass through
// three registered processing stages, so all video sideband signals follow
// the RGB latency.
module brightness_contrast_adjust (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_de,
    input  wire        in_hs,
    input  wire        in_vs,
    input  wire [23:0] rgb_in,
    input  wire [7:0]  brightness,
    input  wire [7:0]  contrast,
    output reg         out_de,
    output reg         out_hs,
    output reg         out_vs,
    output reg  [23:0] rgb_out
);
    // Input isolation: capture the complete pixel bundle before any DSP.
    reg [23:0] rgb_in_reg;
    reg [7:0] brightness_reg;
    reg [7:0] contrast_reg;
    reg de_in_reg, hs_in_reg, vs_in_reg;

    // Stage 1: brightness multipliers. The full products are registered so
    // the divide-by-256 shift is not on the multiplier-to-output path.
    // Code 255 is used as gain 256/256, so full brightness is exact unity.
    wire [8:0] brightness_gain = (brightness_reg == 8'd255) ? 9'd256 : {1'b0, brightness_reg};
    wire [16:0] brightness_mul_r = rgb_in_reg[23:16] * brightness_gain;
    wire [16:0] brightness_mul_g = rgb_in_reg[15:8]  * brightness_gain;
    wire [16:0] brightness_mul_b = rgb_in_reg[7:0]   * brightness_gain;
    reg [16:0] brightness_mul_r_s1, brightness_mul_g_s1, brightness_mul_b_s1;
    reg [7:0] contrast_s1;
    reg bypass_s1;
    reg [23:0] rgb_bypass_s1;
    reg de_s1, hs_s1, vs_s1;

    // Stage 2: brightness shift and alignment registers.
    reg [7:0] bright_r_s2, bright_g_s2, bright_b_s2;
    reg [7:0] contrast_s2;
    reg bypass_s2;
    reg [23:0] rgb_bypass_s2;
    reg de_s2, hs_s2, vs_s2;

    // Contrast: ((bright - 128) * c >>> 7) + 128, saturated to 0..255.
    // Because 128*c is a multiple of 128, floor((bright*c - 128*c) / 128)
    // equals (bright*c >> 7) - c exactly, so the same result is computed with
    // an unsigned 8x8 product. No multiplier operand is ever negative: TD 6.2.1
    // packed the former signed green multiply into a DSP shared with an
    // unsigned one and set it unsigned, turning dark green into 255.
    function [7:0] contrast_apply;
        input [7:0] value;
        input [7:0] gain;
        reg [15:0] product;
        reg [9:0] shifted;      // (value*gain >> 7) + 128: 128..636
        begin
            product = value * gain;
            shifted = {1'b0, product[15:7]} + 10'd128;
            if (shifted < {2'b00, gain})
                contrast_apply = 8'd0;
            else if (shifted - {2'b00, gain} > 10'd255)
                contrast_apply = 8'd255;
            else
                contrast_apply = shifted[7:0] - gain;
        end
    endfunction
    wire [7:0] adjusted_r = contrast_apply(bright_r_s2, contrast_s2);
    wire [7:0] adjusted_g = contrast_apply(bright_g_s2, contrast_s2);
    wire [7:0] adjusted_b = contrast_apply(bright_b_s2, contrast_s2);

    // Final stage: contrast/saturation or exact bypass (255 and 128).
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rgb_in_reg <= 24'd0;
            brightness_reg <= 8'd0;
            contrast_reg <= 8'd0;
            de_in_reg <= 1'b0; hs_in_reg <= 1'b0; vs_in_reg <= 1'b0;
            brightness_mul_r_s1 <= 17'd0;
            brightness_mul_g_s1 <= 17'd0;
            brightness_mul_b_s1 <= 17'd0;
            contrast_s1 <= 8'd0;
            bypass_s1 <= 1'b0;
            rgb_bypass_s1 <= 24'd0;
            de_s1 <= 1'b0; hs_s1 <= 1'b0; vs_s1 <= 1'b0;
            bright_r_s2 <= 8'd0;
            bright_g_s2 <= 8'd0;
            bright_b_s2 <= 8'd0;
            contrast_s2 <= 8'd0;
            bypass_s2 <= 1'b0;
            rgb_bypass_s2 <= 24'd0;
            de_s2 <= 1'b0; hs_s2 <= 1'b0; vs_s2 <= 1'b0;
            out_de <= 1'b0;
            out_hs <= 1'b0;
            out_vs <= 1'b0;
            rgb_out <= 24'h000000;
        end else begin
            // Input isolation stage.
            rgb_in_reg <= rgb_in;
            brightness_reg <= brightness;
            contrast_reg <= contrast;
            de_in_reg <= in_de; hs_in_reg <= in_hs; vs_in_reg <= in_vs;

            // Stage 1.
            brightness_mul_r_s1 <= brightness_mul_r;
            brightness_mul_g_s1 <= brightness_mul_g;
            brightness_mul_b_s1 <= brightness_mul_b;
            contrast_s1 <= contrast_reg;
            // Exact pass-through only when both stages are identities, so
            // contrast still applies at full brightness.
            bypass_s1 <= (brightness_reg == 8'd255) && (contrast_reg == 8'd128);
            rgb_bypass_s1 <= rgb_in_reg;
            de_s1 <= de_in_reg; hs_s1 <= hs_in_reg; vs_s1 <= vs_in_reg;

            // Stage 2.
            // At most 255*256 >> 8 = 255, so bits [15:8] hold the whole result.
            bright_r_s2 <= brightness_mul_r_s1[15:8];
            bright_g_s2 <= brightness_mul_g_s1[15:8];
            bright_b_s2 <= brightness_mul_b_s1[15:8];
            contrast_s2 <= contrast_s1;
            bypass_s2 <= bypass_s1;
            rgb_bypass_s2 <= rgb_bypass_s1;
            de_s2 <= de_s1; hs_s2 <= hs_s1; vs_s2 <= vs_s1;

            // Stage 3 output register.
            out_de <= de_s2;
            out_hs <= hs_s2;
            out_vs <= vs_s2;
            rgb_out <= bypass_s2 ? rgb_bypass_s2 : {adjusted_r, adjusted_g, adjusted_b};
        end
    end
endmodule
