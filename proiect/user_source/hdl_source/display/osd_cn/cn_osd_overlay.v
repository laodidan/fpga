`timescale 1ns/1ps

// Chinese status and caption OSD for the 640x480 active area.
//
// Glyphs are GNU Unifont bitmaps: 16x16 CJK and 8x16 half-width, shown with
// 2x nearest-neighbour scaling as 32x32 and 16x32 pixels. A text column is
// one 8x16 source slice = 16 screen pixels; a CJK glyph uses two columns.
//   text ROM  osd_cn_text.hex  1024 x 9: {variant, column} -> {alert, code}
//   font ROM  osd_cn_font.hex  2048 x 8: {slice, glyph row} -> row bits
// code 0x00..0x7f is a slice, 0x80|k is dynamic field k, 0xff ends a line.
//
// The lookup starts from the lead_* coordinate bundle, which is exactly
// LEAD_CLOCKS = 4 clocks ahead of rgb_in (the brightness/contrast pipeline):
//   lead -> text ROM -> field substitution -> font ROM -> bit select -> align
// so the overlay decision meets its own pixel and rgb_out is a plain mux.
// RGB, DE, HS and VS are therefore not delayed at all.
//
// Status inputs must already be in clk's domain. They are sampled together at
// the lead frame start (VS falling edge, inside vertical blanking). The two
// percentages are converted serially in the following ~25 clocks, still in
// blanking, so one frame always shows one consistent status set.
// brightness/contrast must be frame-committed values (stable for the frame).
module cn_osd_overlay #(
    // Layout in active-area pixels. X origins must be multiples of 16, and
    // BAR_PAD below 16; each text line is 32 pixels high.
    parameter integer TOP_X       = 16,   // status lines 1 and 2
    parameter integer TOP_Y_FULL  = 16,   // status line 1 y, fullscreen
    parameter integer TOP_Y_THUMB = 256,  // status line 1 y, below the 320x240 thumbnail canvas
    parameter integer LINE_PITCH  = 40,   // 32-pixel glyph + 8-pixel spacing
    parameter integer CAPTION_X   = 64,
    parameter integer CAPTION_Y   = 356,
    parameter integer BAR_PAD     = 4,    // dark bar margin around each status line
    // The caption is drawn over the picture with no bar (CAPTION_BAR = 0).
    // Its strokes get a dark shadow one source pixel (2 screen pixels) to the
    // lower right so light text stays legible on white backgrounds.
    parameter integer CAPTION_BAR    = 0,
    parameter integer CAPTION_SHADOW = 1,
    parameter [23:0] TEXT_RGB    = 24'hffffff,
    parameter [23:0] CAPTION_RGB = 24'hffe680,
    parameter [23:0] ALERT_RGB   = 24'hff5050,
    parameter [23:0] BAR_RGB     = 24'h10141c,
    parameter [23:0] SHADOW_RGB  = 24'h202020,
    parameter FONT_FILE = "osd_cn_font.hex",
    parameter TEXT_FILE = "osd_cn_text.hex"
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        lead_vsync,
    input  wire        lead_de,
    input  wire [9:0]  lead_x,
    input  wire [11:0] lead_y,        // >= 480 outside the active area
    input  wire [23:0] rgb_in,        // the pixel lead_* described 4 clocks ago
    input  wire        thumb_view,    // thumbnail page or its zoom back to fullscreen
    input  wire        auto_enabled,
    input  wire        image_valid,
    input  wire [1:0]  image_index,   // picture currently shown fullscreen
    input  wire [1:0]  select_index,  // thumbnail selection
    input  wire [7:0]  brightness,    // gain/256, 255 = exact bypass
    input  wire [7:0]  contrast,      // gain/128 around code 128
    input  wire [7:0]  error_code,
    output wire [23:0] rgb_out,
    output wire        osd_active
);
    localparam [3:0] VAR_STATUS = 4'd6;
    localparam [3:0] VAR_PICTURE = 4'd7;
    localparam [3:0] VAR_SELECT = 4'd11;
    localparam [7:0] CODE_END = 8'hff;
    localparam [6:0] SLICE_SPACE = 7'd0;   // slices 1..16 are '0'..'9', 'A'..'F'
    localparam [5:0] TOP_COL = TOP_X / 16;
    localparam [5:0] CAPTION_COL = CAPTION_X / 16;
    localparam [3:0] PAD_W = BAR_PAD;
    localparam integer Y1_FULL = TOP_Y_FULL;
    localparam integer Y2_FULL = TOP_Y_FULL + LINE_PITCH;
    localparam integer Y1_THUMB = TOP_Y_THUMB;
    localparam integer Y2_THUMB = TOP_Y_THUMB + LINE_PITCH;

    // ------------------------------------------------------------------
    // Frame snapshot
    // ------------------------------------------------------------------
    reg vs_q;
    wire frame_start = vs_q && !lead_vsync;
    reg thumb_q, auto_q, valid_q;
    reg [1:0] index_q, select_q;
    reg [7:0] error_q;
    reg [3:0] var_l1, var_l3;
    reg caption_on;
    reg [5:0] end_l1, end_l2, end_l3;    // first column after each line's text
    wire [5:0] len_l1, len_l2, len_l3;

    osd_cn_text_len u_len_l1 (.variant(var_l1), .length(len_l1));
    osd_cn_text_len u_len_l2 (.variant(VAR_STATUS), .length(len_l2));
    osd_cn_text_len u_len_l3 (.variant(var_l3), .length(len_l3));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vs_q <= 1'b1;
            thumb_q <= 1'b0;
            auto_q <= 1'b0;
            valid_q <= 1'b0;
            index_q <= 2'd0;
            select_q <= 2'd0;
            error_q <= 8'd0;
            var_l1 <= 4'd2;
            var_l3 <= VAR_PICTURE;
            caption_on <= 1'b0;
            end_l1 <= TOP_COL;
            end_l2 <= TOP_COL;
            end_l3 <= CAPTION_COL;
        end else begin
            vs_q <= lead_vsync;
            if (frame_start) begin
                thumb_q <= thumb_view;
                auto_q <= auto_enabled;
                valid_q <= image_valid;
                index_q <= image_index;
                select_q <= select_index;
                error_q <= error_code;
            end
            // Variant numbering is fixed by gen_osd_cn_rom.py:
            // 0..5 mode*2+error, 6 status, 7..10 picture, 11..14 selection.
            var_l1 <= {1'b0, thumb_q ? 2'd2 : (auto_q ? 2'd0 : 2'd1), error_q != 8'd0};
            var_l3 <= thumb_q ? VAR_SELECT + select_q : VAR_PICTURE + index_q;
            caption_on <= thumb_q || valid_q;
            end_l1 <= TOP_COL + len_l1;
            end_l2 <= TOP_COL + len_l2;
            end_l3 <= CAPTION_COL + len_l3;
        end
    end

    // ------------------------------------------------------------------
    // Percentages, converted serially after each frame start.
    // brightness_contrast_adjust scales by code/256 (code 255 is an exact
    // bypass) and contrast by code/128, so the displayed values are
    //   brightness = code==255 ? 100 : floor(code*100/256) = floor(code*25/64)
    //   contrast   = floor(code*100/128)                   = floor(code*25/32)
    // with ranges 0..100 and 0..199.
    // ------------------------------------------------------------------
    localparam [2:0] CV_IDLE = 3'd0, CV_LOAD = 3'd1, CV_HUNDREDS = 3'd2,
                     CV_TENS = 3'd3, CV_STORE = 3'd4;
    reg [2:0] cv_state;
    reg cv_contrast;
    reg [7:0] cv_rest;
    reg cv_hundreds;
    reg [3:0] cv_tens;
    reg bright_h, contrast_h;
    reg [3:0] bright_t, bright_o, contrast_t, contrast_o;
    wire [7:0] cv_code = cv_contrast ? contrast : brightness;
    wire [12:0] cv_x25 = {1'b0, cv_code, 4'd0} + {2'd0, cv_code, 3'd0} + {5'd0, cv_code};
    wire [7:0] cv_percent = cv_contrast ? cv_x25[12:5] :
                            (cv_code == 8'd255) ? 8'd100 : {1'b0, cv_x25[12:6]};
    wire [7:0] cv_step = (cv_state == CV_HUNDREDS) ? 8'd100 : 8'd10;
    wire cv_take = cv_rest >= cv_step;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cv_state <= CV_IDLE;
            cv_contrast <= 1'b0;
            cv_rest <= 8'd0;
            cv_hundreds <= 1'b0;
            cv_tens <= 4'd0;
            bright_h <= 1'b1; bright_t <= 4'd0; bright_o <= 4'd0;
            contrast_h <= 1'b1; contrast_t <= 4'd0; contrast_o <= 4'd0;
        end else begin
            case (cv_state)
            CV_IDLE: if (frame_start) begin
                cv_contrast <= 1'b0;
                cv_state <= CV_LOAD;
            end
            CV_LOAD: begin
                cv_rest <= cv_percent;
                cv_hundreds <= 1'b0;
                cv_tens <= 4'd0;
                cv_state <= CV_HUNDREDS;
            end
            CV_HUNDREDS: begin
                if (cv_take) begin
                    cv_rest <= cv_rest - cv_step;
                    cv_hundreds <= 1'b1;
                end
                cv_state <= CV_TENS;
            end
            CV_TENS: begin
                if (cv_take) begin
                    cv_rest <= cv_rest - cv_step;
                    cv_tens <= cv_tens + 4'd1;
                end else
                    cv_state <= CV_STORE;
            end
            default: begin
                if (cv_contrast) begin
                    contrast_h <= cv_hundreds; contrast_t <= cv_tens; contrast_o <= cv_rest[3:0];
                    cv_state <= CV_IDLE;
                end else begin
                    bright_h <= cv_hundreds; bright_t <= cv_tens; bright_o <= cv_rest[3:0];
                    cv_contrast <= 1'b1;
                    cv_state <= CV_LOAD;
                end
            end
            endcase
        end
    end

    // Field k replaces text code 0x80|k; see FIELD_CODES in the generator.
    // Leading zeros of a percentage are shown as spaces.
    function [6:0] field_slice;
        input [2:0] field;
        begin
            case (field)
            3'd0: field_slice = {3'd0, error_q[7:4]} + 7'd1;
            3'd1: field_slice = {3'd0, error_q[3:0]} + 7'd1;
            3'd2: field_slice = bright_h ? 7'd2 : SLICE_SPACE;
            3'd3: field_slice = (!bright_h && bright_t == 4'd0) ? SLICE_SPACE :
                                {3'd0, bright_t} + 7'd1;
            3'd4: field_slice = {3'd0, bright_o} + 7'd1;
            3'd5: field_slice = contrast_h ? 7'd2 : SLICE_SPACE;
            3'd6: field_slice = (!contrast_h && contrast_t == 4'd0) ? SLICE_SPACE :
                                {3'd0, contrast_t} + 7'd1;
            default: field_slice = {3'd0, contrast_o} + 7'd1;
            endcase
        end
    endfunction

    // ------------------------------------------------------------------
    // Lead stage: line hit test and text ROM address
    // ------------------------------------------------------------------
    // Rows of a line: bar [y0-PAD, y0+32+PAD), text [y0, y0+32).
    function bar_rows;
        input [11:0] y;
        input integer y0;
        bar_rows = (y >= y0 - BAR_PAD) && (y < y0 + 32 + BAR_PAD);
    endfunction
    function text_rows;
        input [11:0] y;
        input integer y0;
        text_rows = (y >= y0) && (y < y0 + 32);
    endfunction

    wire l1_bar = thumb_q ? bar_rows(lead_y, Y1_THUMB) : bar_rows(lead_y, Y1_FULL);
    wire l2_bar = thumb_q ? bar_rows(lead_y, Y2_THUMB) : bar_rows(lead_y, Y2_FULL);
    wire l3_bar = caption_on && bar_rows(lead_y, CAPTION_Y);
    wire l1_text = thumb_q ? text_rows(lead_y, Y1_THUMB) : text_rows(lead_y, Y1_FULL);
    wire l2_text = thumb_q ? text_rows(lead_y, Y2_THUMB) : text_rows(lead_y, Y2_FULL);
    wire l3_text = text_rows(lead_y, CAPTION_Y);
    // Glyph row = (y - y0)[4:1]; only y0 mod 32 matters.
    wire [4:0] y1_mod = thumb_q ? Y1_THUMB % 32 : Y1_FULL % 32;
    wire [4:0] y2_mod = thumb_q ? Y2_THUMB % 32 : Y2_FULL % 32;
    wire [4:0] y3_mod = CAPTION_Y % 32;

    reg line_bar, line_text, line_caption;
    reg [4:0] line_y_mod;
    reg [5:0] line_col0, line_end;
    reg [3:0] line_var;
    always @* begin
        line_bar = l1_bar;
        line_text = l1_text;
        line_caption = 1'b0;
        line_y_mod = y1_mod;
        line_col0 = TOP_COL;
        line_end = end_l1;
        line_var = var_l1;
        if (!l1_bar) begin
            line_bar = l2_bar || l3_bar;
            if (l2_bar) begin
                line_text = l2_text;
                line_y_mod = y2_mod;
                line_end = end_l2;
                line_var = VAR_STATUS;
            end else begin
                line_text = l3_text;
                line_caption = 1'b1;
                line_y_mod = y3_mod;
                line_col0 = CAPTION_COL;
                line_end = end_l3;
                line_var = var_l3;
            end
        end
    end
    wire [5:0] lead_col = lead_x[9:4];
    wire [3:0] lead_sub = lead_x[3:0];
    wire text_cols = (lead_col >= line_col0) && (lead_col < line_end);
    wire bar_cols = text_cols ||
                    (lead_col == line_col0 - 6'd1 && lead_sub >= 5'd16 - PAD_W) ||
                    (lead_col == line_end && lead_sub < PAD_W);
    wire text_hit_s0 = lead_de && line_bar && line_text && text_cols;
    wire bar_hit_s0 = lead_de && line_bar && bar_cols &&
                      (CAPTION_BAR != 0 || !line_caption);
    // Caption shadow area: text box extended 2 pixels right and down.
    wire shadow_rows = (lead_y >= CAPTION_Y) && (lead_y < CAPTION_Y + 34);
    wire shadow_bottom = lead_y >= CAPTION_Y + 32;
    wire shadow_cols = (lead_col >= line_col0) &&
                       ((lead_col < line_end) || (lead_col == line_end && lead_sub < 4'd2));
    wire shadow_hit_s0 = (CAPTION_SHADOW != 0) && lead_de && l3_bar && line_caption &&
                         shadow_rows && shadow_cols;
    wire [4:0] glyph_y = lead_y[4:0] - line_y_mod;
    wire [5:0] text_col = lead_col - line_col0;
    wire [8:0] text_q;
    osd_cn_rom #(.ADDR_W(10), .DATA_W(9), .INIT_FILE(TEXT_FILE)) u_text_rom (
        .clk(clk), .addr({line_var, text_col}), .data(text_q)
    );

    // ------------------------------------------------------------------
    // Stage 1: field substitution and font ROM address
    // Stage 2: glyph bit select
    // Stages 3, 4: align with rgb_in
    // ------------------------------------------------------------------
    reg text_s1, bar_s1, caption_s1;
    reg shadow_s1, bottom_s1;
    reg [3:0] row_s1;
    reg [2:0] bit_s1;
    reg [6:0] prev_slice;             // slice of the column left of stage 1
    reg glyph_s2, bar_s2, shadow_en_s2;
    reg [2:0] bit_s2, shadow_bit_s2;
    reg [1:0] color_s2;
    reg [4:0] flags_s3, flags_s4;     // {pixel, shadow, bar, color}
    wire [7:0] text_code = text_q[7:0];
    wire [6:0] slice_s1 = text_code[7] ? field_slice(text_code[2:0]) : text_code[6:0];
    wire slice_valid_s1 = text_code != CODE_END;
    // The shadow at source pixel (sx, sy) is glyph pixel (sx-1, sy-1): row
    // above (row 15 for the two extension rows), and for the column's first
    // source pixel the right-most pixel of the previous column's slice.
    wire [6:0] shadow_slice_s1 = (bit_s1 == 3'd0) ? prev_slice : slice_s1;
    wire [3:0] shadow_row_s1 = bottom_s1 ? 4'd15 : row_s1 - 4'd1;
    wire shadow_en_s1 = shadow_s1 && (bottom_s1 || row_s1 != 4'd0) &&
                        (bit_s1 == 3'd0 || slice_valid_s1);
    wire [7:0] font_q, shadow_q;
    osd_cn_rom2 #(.ADDR_W(11), .DATA_W(8), .INIT_FILE(FONT_FILE)) u_font_rom (
        .clk(clk), .addr_a({slice_s1, row_s1}), .data_a(font_q),
        .addr_b({shadow_slice_s1, shadow_row_s1}), .data_b(shadow_q)
    );
    // Font byte bit 7 is the leftmost source pixel.
    wire pixel_s2 = glyph_s2 && font_q[3'd7 - bit_s2];
    wire shadow_px_s2 = shadow_en_s2 && shadow_q[shadow_bit_s2];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            text_s1 <= 1'b0; bar_s1 <= 1'b0; caption_s1 <= 1'b0;
            shadow_s1 <= 1'b0; bottom_s1 <= 1'b0; prev_slice <= SLICE_SPACE;
            row_s1 <= 4'd0; bit_s1 <= 3'd0;
            glyph_s2 <= 1'b0; bar_s2 <= 1'b0; shadow_en_s2 <= 1'b0;
            bit_s2 <= 3'd0; shadow_bit_s2 <= 3'd0; color_s2 <= 2'd0;
            flags_s3 <= 5'd0; flags_s4 <= 5'd0;
        end else begin
            text_s1 <= text_hit_s0;
            bar_s1 <= bar_hit_s0;
            caption_s1 <= line_caption;
            shadow_s1 <= shadow_hit_s0;
            bottom_s1 <= shadow_bottom;
            // Remember each caption column's slice at its last source pixel.
            if (!shadow_s1)
                prev_slice <= SLICE_SPACE;
            else if (bit_s1 == 3'd7)
                prev_slice <= slice_valid_s1 ? slice_s1 : SLICE_SPACE;
            row_s1 <= glyph_y[4:1];      // 2x vertical scaling
            bit_s1 <= lead_x[3:1];       // 2x horizontal scaling
            glyph_s2 <= text_s1 && (text_code != CODE_END);
            bar_s2 <= bar_s1;
            bit_s2 <= bit_s1;
            color_s2 <= {caption_s1, text_q[8]};
            shadow_en_s2 <= shadow_en_s1;
            // bit 0 of the previous slice, else bit (bit_s1 - 1) from the left.
            shadow_bit_s2 <= (bit_s1 == 3'd0) ? 3'd0 : 3'd0 - bit_s1;
            flags_s3 <= {pixel_s2, shadow_px_s2, bar_s2, color_s2};
            flags_s4 <= flags_s3;
        end
    end
    // Only glyph strokes and their shadow cover the caption's picture pixels.
    wire [23:0] text_color = flags_s4[1] ? CAPTION_RGB :
                             flags_s4[0] ? ALERT_RGB : TEXT_RGB;
    assign rgb_out = flags_s4[4] ? text_color : flags_s4[3] ? SHADOW_RGB :
                     flags_s4[2] ? BAR_RGB : rgb_in;
    assign osd_active = flags_s4[4] || flags_s4[3] || flags_s4[2];

    // synthesis translate_off
    initial begin
        if (TOP_X % 16 != 0 || CAPTION_X % 16 != 0 || TOP_X < 16 || CAPTION_X < 16)
            $fatal(1, "cn_osd_overlay: text x origins must be multiples of 16 and >= 16");
        if (BAR_PAD < 0 || BAR_PAD > 15 || TOP_Y_FULL < BAR_PAD ||
            TOP_Y_THUMB < BAR_PAD || CAPTION_Y < BAR_PAD)
            $fatal(1, "cn_osd_overlay: BAR_PAD must be 0..15 and fit above every line");
    end
    // synthesis translate_on
endmodule
