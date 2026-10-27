`timescale 1ns/1ps

module status_osd_overlay #(
    parameter integer OSD_X = 8,
    parameter integer OSD_Y = 8
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire [1:0]  active_index,
    input  wire        auto_enabled,
    input  wire [7:0]  error_code,
    input  wire [7:0]  brightness_active,
    input  wire [7:0]  contrast_active,
    input  wire        in_de,
    input  wire [11:0] in_rgb,
    input  wire [11:0] pixel_x,
    input  wire [11:0] pixel_y,
    output reg  [11:0] out_rgb,
    output reg         out_de,
    output wire        osd_pixel
);
    localparam integer OSD_WIDTH = 72;
    reg [1:0] index_q;
    reg auto_q;
    reg [7:0] error_q;
    reg [7:0] brightness_q;
    reg [7:0] contrast_q;
    reg osd_hit;
    reg [4:0] char_value;
    reg [4:0] subtitle_char_value;
    reg [2:0] char_row;
    reg [2:0] char_col;
    reg [4:0] glyph_row;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            index_q <= 2'd0;
            auto_q <= 1'b0;
            error_q <= 8'd0;
            brightness_q <= 8'd255;
            contrast_q <= 8'd128;
        end else if (frame_start) begin
            index_q <= active_index;
            auto_q <= auto_enabled;
            error_q <= error_code;
            brightness_q <= brightness_active;
            contrast_q <= contrast_active;
        end
    end

    function [4:0] font_row;
        input [4:0] value;
        input [2:0] row;
        begin
            case (value)
                4'h0: case(row) 0:font_row=5'b11111;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b10001;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b11111; endcase
                4'h1: case(row) 0:font_row=5'b00100;1:font_row=5'b01100;2:font_row=5'b00100;3:font_row=5'b00100;4:font_row=5'b00100;5:font_row=5'b00100;default:font_row=5'b11111; endcase
                4'h2: case(row) 0:font_row=5'b11111;1:font_row=5'b00001;2:font_row=5'b00001;3:font_row=5'b11111;4:font_row=5'b10000;5:font_row=5'b10000;default:font_row=5'b11111; endcase
                4'h3: case(row) 0:font_row=5'b11111;1:font_row=5'b00001;2:font_row=5'b00001;3:font_row=5'b11111;4:font_row=5'b00001;5:font_row=5'b00001;default:font_row=5'b11111; endcase
                4'h4: case(row) 0:font_row=5'b10001;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b11111;4:font_row=5'b00001;5:font_row=5'b00001;default:font_row=5'b00001; endcase
                4'h5: case(row) 0:font_row=5'b11111;1:font_row=5'b10000;2:font_row=5'b10000;3:font_row=5'b11111;4:font_row=5'b00001;5:font_row=5'b00001;default:font_row=5'b11111; endcase
                4'h6: case(row) 0:font_row=5'b11111;1:font_row=5'b10000;2:font_row=5'b10000;3:font_row=5'b11111;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b11111; endcase
                4'h7: case(row) 0:font_row=5'b11111;1:font_row=5'b00001;2:font_row=5'b00010;3:font_row=5'b00100;4:font_row=5'b01000;5:font_row=5'b01000;default:font_row=5'b01000; endcase
                4'h8: case(row) 0:font_row=5'b11111;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b11111;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b11111; endcase
                4'h9: case(row) 0:font_row=5'b11111;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b11111;4:font_row=5'b00001;5:font_row=5'b00001;default:font_row=5'b11111; endcase
                4'ha: case(row) 0:font_row=5'b01110;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b11111;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b10001; endcase
                // Complete the hexadecimal error-code alphabet.
                4'hb: case(row) 0:font_row=5'b11110;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b11110;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b11110; endcase
                4'hc: case(row) 0:font_row=5'b01111;1:font_row=5'b10000;2:font_row=5'b10000;3:font_row=5'b10000;4:font_row=5'b10000;5:font_row=5'b10000;default:font_row=5'b01111; endcase
                4'hd: case(row) 0:font_row=5'b11110;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b10001;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b11110; endcase
                4'he: case(row) 0:font_row=5'b11111;1:font_row=5'b10000;2:font_row=5'b10000;3:font_row=5'b11110;4:font_row=5'b10000;5:font_row=5'b10000;default:font_row=5'b11111; endcase
                4'hf: case(row) 0:font_row=5'b11111;1:font_row=5'b10000;2:font_row=5'b10000;3:font_row=5'b11110;4:font_row=5'b10000;5:font_row=5'b10000;default:font_row=5'b10000; endcase
                5'h10: case(row) 3:font_row=5'b00100;4:font_row=5'b00100;default:font_row=5'b00000; endcase
                // Museum caption alphabet; existing hexadecimal glyph codes stay unchanged.
                5'h11: case(row) 0:font_row=5'b11110;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b11110;4:font_row=5'b10100;5:font_row=5'b10010;default:font_row=5'b10001; endcase // R
                5'h12: case(row) 0:font_row=5'b01110;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b10001;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b01110; endcase // O
                5'h13: case(row) 0:font_row=5'b10001;1:font_row=5'b11001;2:font_row=5'b11001;3:font_row=5'b10101;4:font_row=5'b10011;5:font_row=5'b10011;default:font_row=5'b10001; endcase // N
                5'h14: case(row) 0:font_row=5'b11111;1:font_row=5'b00001;2:font_row=5'b00010;3:font_row=5'b00100;4:font_row=5'b01000;5:font_row=5'b10000;default:font_row=5'b11111; endcase // Z
                5'h15: case(row) 0:font_row=5'b11110;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b11110;4:font_row=5'b10000;5:font_row=5'b10000;default:font_row=5'b10000; endcase // P
                5'h16: case(row) 0:font_row=5'b10000;1:font_row=5'b10000;2:font_row=5'b10000;3:font_row=5'b10000;4:font_row=5'b10000;5:font_row=5'b10000;default:font_row=5'b11111; endcase // L
                5'h17: case(row) 0:font_row=5'b11111;1:font_row=5'b00100;2:font_row=5'b00100;3:font_row=5'b00100;4:font_row=5'b00100;5:font_row=5'b00100;default:font_row=5'b11111; endcase // I
                5'h18: case(row) 0:font_row=5'b11111;1:font_row=5'b00100;2:font_row=5'b00100;3:font_row=5'b00100;4:font_row=5'b00100;5:font_row=5'b00100;default:font_row=5'b00100; endcase // T
                5'h19: case(row) 0:font_row=5'b01111;1:font_row=5'b10000;2:font_row=5'b10000;3:font_row=5'b10111;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b01111; endcase // G
                5'h1a: case(row) 0:font_row=5'b10001;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b10001;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b01110; endcase // U
                default: font_row = 5'b00000;
            endcase
        end
    endfunction

    always @* begin
        osd_hit = 1'b0;
        char_value = 5'h00;
        subtitle_char_value = 5'h1f;
        char_row = pixel_y - OSD_Y;
        char_col = pixel_x - OSD_X;
        glyph_row = 5'b00000;
        if (pixel_x >= OSD_X && pixel_x < OSD_X + OSD_WIDTH &&
            pixel_y >= OSD_Y && pixel_y < OSD_Y + 32) begin
            case ((pixel_y - OSD_Y) / 8)
                0: case ((pixel_x - OSD_X) / 8)
                    0: char_value = {3'b000, index_q} + 1'b1;
                    1: char_value = auto_q ? 5'h0a : 5'h0e;
                    2: char_value = {1'b0, error_q[7:4]};
                    3: char_value = {1'b0, error_q[3:0]};
                    default: char_value = 5'h00;
                endcase
                2: case ((pixel_x - OSD_X) / 8)
                    0: char_value = 5'h0b;
                    1: char_value = 5'h10;
                    2: char_value = (brightness_q / 100) % 10;
                    3: char_value = (brightness_q / 10) % 10;
                    4: char_value = brightness_q % 10;
                    default: char_value = 5'h00;
                endcase
                3: case ((pixel_x - OSD_X) / 8)
                    0: char_value = 5'h0c;
                    1: char_value = 5'h10;
                    2: char_value = (contrast_q / 100) % 10;
                    3: char_value = (contrast_q / 10) % 10;
                    4: char_value = contrast_q % 10;
                    default: char_value = 5'h00;
                endcase
                1: case (index_q)
                    2'd0: case ((pixel_x - OSD_X) >> 3) // BRONZE
                        0: subtitle_char_value = 5'h0b;
                        1: subtitle_char_value = 5'h11;
                        2: subtitle_char_value = 5'h12;
                        3: subtitle_char_value = 5'h13;
                        4: subtitle_char_value = 5'h14;
                        5: subtitle_char_value = 5'h0e;
                        default: subtitle_char_value = 5'h1f;
                    endcase
                    2'd1: case ((pixel_x - OSD_X) >> 3) // PORCELAIN
                        0: subtitle_char_value = 5'h15;
                        1: subtitle_char_value = 5'h12;
                        2: subtitle_char_value = 5'h11;
                        3: subtitle_char_value = 5'h0c;
                        4: subtitle_char_value = 5'h0e;
                        5: subtitle_char_value = 5'h16;
                        6: subtitle_char_value = 5'h0a;
                        7: subtitle_char_value = 5'h17;
                        8: subtitle_char_value = 5'h13;
                        default: subtitle_char_value = 5'h1f;
                    endcase
                    2'd2: case ((pixel_x - OSD_X) >> 3) // PAINTING
                        0: subtitle_char_value = 5'h15;
                        1: subtitle_char_value = 5'h0a;
                        2: subtitle_char_value = 5'h17;
                        3: subtitle_char_value = 5'h13;
                        4: subtitle_char_value = 5'h18;
                        5: subtitle_char_value = 5'h17;
                        6: subtitle_char_value = 5'h13;
                        7: subtitle_char_value = 5'h19;
                        default: subtitle_char_value = 5'h1f;
                    endcase
                    2'd3: case ((pixel_x - OSD_X) >> 3) // DIGITAL
                        0: subtitle_char_value = 5'h0d;
                        1: subtitle_char_value = 5'h17;
                        2: subtitle_char_value = 5'h19;
                        3: subtitle_char_value = 5'h17;
                        4: subtitle_char_value = 5'h18;
                        5: subtitle_char_value = 5'h0a;
                        6: subtitle_char_value = 5'h16;
                        default: subtitle_char_value = 5'h1f;
                    endcase
                    default: subtitle_char_value = 5'h1f;
                endcase
                default: char_value = 5'h00;
            endcase
            if ((pixel_y - OSD_Y) / 8 == 1)
                glyph_row = font_row(subtitle_char_value, char_row);
            else if (pixel_x < OSD_X + 40)
                glyph_row = font_row(char_value, char_row);
            if ((pixel_x - OSD_X) % 8 < 5 && char_row < 7)
                osd_hit = glyph_row[4 - ((pixel_x - OSD_X) % 8)];
        end
        out_de = in_de;
        out_rgb = in_rgb;
        if (in_de && osd_hit)
            out_rgb = error_q == 0 ? 12'heff : 12'hf44;
    end

    assign osd_pixel = osd_hit && in_de;
endmodule
