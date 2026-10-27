`timescale 1ns/1ps

// 2x2 thumbnail and fullscreen coordinate mapper.
// active_x/active_y are zero-based coordinates in the 640x480 active area.
module thumbnail_layout (
    input  wire [9:0] active_x,
    input  wire [9:0] active_y,
    input  wire [1:0] focus_index,
    input  wire [3:0] image_complete,
    input  wire       display_mode, // 0: fullscreen, 1: thumbnail
    input  wire [1:0] active_index,
    input  wire       frame_start,
    input  wire       line_start,
    input  wire [9:0] scan_x,
    input  wire [9:0] scan_y,
    output reg        mode_frame,
    output reg        zoom_active,
    output reg  [4:0] zoom_step,
    output reg  [1:0] zoom_index,
    output reg        demand_valid,
    output reg  [1:0] demand_index,
    output reg  [9:0] demand_row,
    output reg        prefetch_valid,
    output reg  [1:0] prefetch_index,
    output reg  [9:0] prefetch_row,
    output reg  [1:0] source_index,
    output reg  [9:0] source_x,
    output reg  [9:0] source_y,
    output reg        thumb_de,
    output reg        focus_border,
    input  wire       video_clk,
    input  wire       rst_n
);
    reg [9:0] local_x;
    reg [9:0] local_y;
    reg focus_region_hit;
    reg status_marker_hit;

    reg [9:0] zoom_width;
    reg [9:0] zoom_height;
    reg [9:0] zoom_left;
    reg [9:0] zoom_top;
    reg [9:0] slot_left;
    reg [9:0] slot_top;
    wire [10:0] zoom_right = {1'b0, zoom_left} + {1'b0, zoom_width};
    wire [10:0] zoom_bottom = {1'b0, zoom_top} + {1'b0, zoom_height};
    wire zoom_pixel = zoom_active && active_x >= zoom_left &&
                      {1'b0, active_x} < zoom_right && active_y >= zoom_top &&
                      {1'b0, active_y} < zoom_bottom;
    reg [9:0] x_sample;
    reg [9:0] x_phase;
    reg [9:0] y_sample;
    reg [9:0] y_phase;
    reg [9:0] x_phase_next;
    reg [9:0] y_phase_next;
    reg [2:0] x_increment;
    reg [2:0] y_increment;
    wire [11:0] x_sum = {2'b0, x_phase} + 12'd640;
    wire [11:0] y_sum = {2'b0, y_phase} + 12'd480;
    wire [11:0] width_1 = {2'b0, zoom_width};
    wire [11:0] width_2 = {1'b0, zoom_width, 1'b0};
    wire [11:0] width_3 = width_2 + width_1;
    wire [11:0] width_4 = {zoom_width, 2'b0};
    wire [11:0] height_1 = {2'b0, zoom_height};
    wire [11:0] height_2 = {1'b0, zoom_height, 1'b0};
    wire [11:0] height_3 = height_2 + height_1;
    wire [11:0] height_4 = {zoom_height, 2'b0};
    wire [10:0] next_y_sample = {1'b0, y_sample} + {8'd0, y_increment};
    reg [9:0] thumbnail_row;
    reg [9:0] next_thumbnail_row;
    reg [1:0] thumbnail_even_bank;
    reg [1:0] next_thumbnail_even_bank;

    // Step 0 and step 16 each own one complete frame. The selected bank is
    // committed before mode release and remains fixed throughout the zoom.
    always @(posedge video_clk) begin
        if (!rst_n) begin
            mode_frame <= 1'b0;
            zoom_active <= 1'b0;
            zoom_step <= 5'd0;
            zoom_index <= 2'd0;
        end else if (frame_start) begin
            mode_frame <= display_mode;
            if (display_mode) begin
                zoom_active <= 1'b0;
                zoom_step <= 5'd0;
            end else if (mode_frame) begin
                zoom_active <= 1'b1;
                zoom_step <= 5'd0;
                zoom_index <= active_index;
            end else if (zoom_active) begin
                if (zoom_step < 5'd16)
                    zoom_step <= zoom_step + 5'd1;
                else
                    zoom_active <= 1'b0;
            end
        end
    end

    // Geometry is constant for a whole frame. All arithmetic below is
    // bounded unsigned addition, subtraction and constant shifts.
    always @* begin
        zoom_width = 10'd640;
        zoom_height = 10'd480;
        slot_left = 10'd0;
        slot_top = 10'd0;
        case (zoom_step)
            5'd0:  begin zoom_width=10'd160; zoom_height=10'd120; slot_left=10'd160; slot_top=10'd120; end
            5'd1:  begin zoom_width=10'd190; zoom_height=10'd142; slot_left=10'd150; slot_top=10'd113; end
            5'd2:  begin zoom_width=10'd220; zoom_height=10'd165; slot_left=10'd140; slot_top=10'd105; end
            5'd3:  begin zoom_width=10'd250; zoom_height=10'd187; slot_left=10'd130; slot_top=10'd98; end
            5'd4:  begin zoom_width=10'd280; zoom_height=10'd210; slot_left=10'd120; slot_top=10'd90; end
            5'd5:  begin zoom_width=10'd310; zoom_height=10'd232; slot_left=10'd110; slot_top=10'd83; end
            5'd6:  begin zoom_width=10'd340; zoom_height=10'd255; slot_left=10'd100; slot_top=10'd75; end
            5'd7:  begin zoom_width=10'd370; zoom_height=10'd277; slot_left=10'd90; slot_top=10'd68; end
            5'd8:  begin zoom_width=10'd400; zoom_height=10'd300; slot_left=10'd80; slot_top=10'd60; end
            5'd9:  begin zoom_width=10'd430; zoom_height=10'd322; slot_left=10'd70; slot_top=10'd53; end
            5'd10: begin zoom_width=10'd460; zoom_height=10'd345; slot_left=10'd60; slot_top=10'd45; end
            5'd11: begin zoom_width=10'd490; zoom_height=10'd367; slot_left=10'd50; slot_top=10'd38; end
            5'd12: begin zoom_width=10'd520; zoom_height=10'd390; slot_left=10'd40; slot_top=10'd30; end
            5'd13: begin zoom_width=10'd550; zoom_height=10'd412; slot_left=10'd30; slot_top=10'd23; end
            5'd14: begin zoom_width=10'd580; zoom_height=10'd435; slot_left=10'd20; slot_top=10'd15; end
            5'd15: begin zoom_width=10'd610; zoom_height=10'd457; slot_left=10'd10; slot_top=10'd8; end
            5'd16: begin zoom_width=10'd640; zoom_height=10'd480; slot_left=10'd0; slot_top=10'd0; end
            default: begin end
        endcase
        zoom_left = zoom_index[0] ? slot_left : 10'd0;
        zoom_top = zoom_index[1] ? slot_top : 10'd0;
    end

    always @* begin
        x_phase_next = x_sum - width_1;
        x_increment = 3'd1;
        if (x_sum >= width_4) begin
            x_phase_next = x_sum - width_4;
            x_increment = 3'd4;
        end else if (x_sum >= width_3) begin
            x_phase_next = x_sum - width_3;
            x_increment = 3'd3;
        end else if (x_sum >= width_2) begin
            x_phase_next = x_sum - width_2;
            x_increment = 3'd2;
        end
        y_phase_next = y_sum - height_1;
        y_increment = 3'd1;
        if (y_sum >= height_4) begin
            y_phase_next = y_sum - height_4;
            y_increment = 3'd4;
        end else if (y_sum >= height_3) begin
            y_phase_next = y_sum - height_3;
            y_increment = 3'd3;
        end else if (y_sum >= height_2) begin
            y_phase_next = y_sum - height_2;
            y_increment = 3'd2;
        end
    end

    // The raster advances on the same edge: these registers describe the
    // next raw pixel, with no extra video pipeline latency.
    always @(posedge video_clk) begin
        if (!rst_n || frame_start) begin
            x_sample <= 10'd0;
            x_phase <= 10'd0;
            y_sample <= 10'd0;
            y_phase <= 10'd0;
        end else begin
            if (zoom_pixel) begin
                x_sample <= x_sample + {7'd0, x_increment};
                x_phase <= x_phase_next;
            end else begin
                x_sample <= 10'd0;
                x_phase <= 10'd0;
            end
            if (line_start) begin
                if (!zoom_active || scan_y <= zoom_top || scan_y >= 10'd480) begin
                    y_sample <= 10'd0;
                    y_phase <= 10'd0;
                end else if ({1'b0, scan_y} < zoom_bottom) begin
                    y_sample <= next_y_sample[9:0];
                    y_phase <= y_phase_next;
                end
            end
        end
    end

    // Descriptor changes occur in horizontal/vertical blank before each
    // visible segment, allowing two tagged line buffers to work ahead.
    always @* begin
        demand_valid = mode_frame || zoom_active;
        prefetch_valid = mode_frame || zoom_active;
        demand_index = 2'd0;
        demand_row = 10'd0;
        prefetch_index = 2'd1;
        prefetch_row = 10'd0;
        thumbnail_row = 10'd0;
        next_thumbnail_row = 10'd0;
        thumbnail_even_bank = 2'd0;
        next_thumbnail_even_bank = 2'd0;
        if (zoom_active) begin
            demand_index = zoom_index;
            prefetch_index = zoom_index;
            if (scan_y >= zoom_top && {1'b0, scan_y} < zoom_bottom) begin
                demand_row = y_sample;
                prefetch_row = next_y_sample < 11'd480 ? next_y_sample[9:0] : 10'd0;
            end else begin
                demand_row = 10'd0;
                if (12'd480 >= height_4)
                    prefetch_row = 10'd4;
                else if (12'd480 >= height_3)
                    prefetch_row = 10'd3;
                else if (12'd480 >= height_2)
                    prefetch_row = 10'd2;
                else
                    prefetch_row = 10'd1;
            end
        end else if (mode_frame && scan_y < 10'd240) begin
            thumbnail_even_bank = scan_y < 10'd120 ? 2'd0 : 2'd2;
            thumbnail_row = scan_y < 10'd120 ? (scan_y << 2) : ((scan_y - 10'd120) << 2);
            next_thumbnail_even_bank = thumbnail_even_bank;
            next_thumbnail_row = thumbnail_row + 10'd4;
            if (thumbnail_row == 10'd476) begin
                next_thumbnail_even_bank = thumbnail_even_bank == 2'd0 ? 2'd2 : 2'd0;
                next_thumbnail_row = 10'd0;
            end
            if (scan_x < 10'd320) begin
                demand_index = thumbnail_even_bank;
                demand_row = thumbnail_row;
                prefetch_index = thumbnail_even_bank | 2'd1;
                prefetch_row = thumbnail_row;
            end else if (scan_x < 10'd480) begin
                demand_index = thumbnail_even_bank | 2'd1;
                demand_row = thumbnail_row;
                prefetch_index = next_thumbnail_even_bank;
                prefetch_row = next_thumbnail_row;
            end else begin
                demand_index = next_thumbnail_even_bank;
                demand_row = next_thumbnail_row;
                prefetch_index = next_thumbnail_even_bank | 2'd1;
                prefetch_row = next_thumbnail_row;
            end
        end
        demand_valid = demand_valid && demand_row < 10'd480;
        prefetch_valid = prefetch_valid && prefetch_row < 10'd480;
    end

    always @* begin
        source_index = 2'd0;
        source_x = 10'd0;
        source_y = 10'd0;
        thumb_de = 1'b0;
        focus_border = 1'b0;
        local_x = 10'd0;
        local_y = 10'd0;
        status_marker_hit = 1'b0;
        // Explicit absolute bounds prevent a local-coordinate edge from
        // being applied to another quadrant or outside the 320x240 canvas.
        case (focus_index)
            2'd0: focus_region_hit = active_x < 10'd160 && active_y < 10'd120;
            2'd1: focus_region_hit = active_x >= 10'd160 && active_x < 10'd320 &&
                                    active_y < 10'd120;
            2'd2: focus_region_hit = active_x < 10'd160 &&
                                    active_y >= 10'd120 && active_y < 10'd240;
            2'd3: focus_region_hit = active_x >= 10'd160 && active_x < 10'd320 &&
                                    active_y >= 10'd120 && active_y < 10'd240;
            default: focus_region_hit = 1'b0;
        endcase

        if (!mode_frame) begin
            source_index = zoom_active ? zoom_index : active_index;
            if (zoom_active) begin
                if (zoom_pixel) begin
                    source_x = x_sample;
                    source_y = y_sample;
                    thumb_de = 1'b1;
                end
            end else if ((active_x < 10'd640) && (active_y < 10'd480)) begin
                source_x = active_x;
                source_y = active_y;
                // DE describes geometry; image readiness stays with the
                // reader.
                thumb_de = (active_x < 10'd640) && (active_y < 10'd480);
            end
        end else if ((active_x < 10'd320) && (active_y < 10'd240)) begin
            // The thumbnail canvas occupies the upper-left 320x240 area.
            thumb_de = 1'b1;
            if (active_x < 10'd160) begin
                local_x = active_x;
                if (active_y < 10'd120) begin
                    source_index = 2'd0;
                    local_y = active_y;
                end else begin
                    source_index = 2'd2;
                    local_y = active_y - 10'd120;
                end
            end else begin
                local_x = active_x - 10'd160;
                if (active_y < 10'd120) begin
                    source_index = 2'd1;
                    local_y = active_y;
                end else begin
                    source_index = 2'd3;
                    local_y = active_y - 10'd120;
                end
            end

            // Each 160x120 thumbnail samples every fourth source pixel.
            source_x = local_x << 2;
            source_y = local_y << 2;

            // The existing focus_border bit is the only pixel-overlay signal
            // available at the top level.  Reuse it for a small status glyph
            // in the upper-right interior of every thumbnail.  A check marks
            // a completed image and an X marks an image that is not complete.
            // The 8x8 glyph is inset from the two-pixel focus border.
            if ((local_x >= 10'd148) && (local_x < 10'd156) &&
                (local_y >= 10'd6) && (local_y < 10'd14)) begin
                if (image_complete[source_index]) begin
                    status_marker_hit =
                        ((local_y == 10'd6)  && (local_x >= 10'd154)) ||
                        ((local_y == 10'd7)  && (local_x >= 10'd153) &&
                                                   (local_x <= 10'd154)) ||
                        ((local_y == 10'd8)  && (local_x >= 10'd152) &&
                                                   (local_x <= 10'd153)) ||
                        ((local_y == 10'd9)  &&
                            (((local_x >= 10'd148) && (local_x <= 10'd149)) ||
                             ((local_x >= 10'd151) && (local_x <= 10'd152)))) ||
                        ((local_y == 10'd10) && (local_x >= 10'd149) &&
                                                   (local_x <= 10'd151)) ||
                        ((local_y == 10'd11) && (local_x >= 10'd150) &&
                                                   (local_x <= 10'd151)) ||
                        ((local_y == 10'd12) && (local_x >= 10'd150) &&
                                                   (local_x <= 10'd151));
                end else begin
                    status_marker_hit =
                        ((local_x - 10'd148) == (local_y - 10'd6)) ||
                        ((local_x - 10'd148) + (local_y - 10'd6) == 10'd7);
                end
            end

            // An unloaded image still owns its thumbnail region, but cannot
            // display a focus border until its completion bit is asserted.
            if (focus_region_hit && image_complete[focus_index] &&
                ((local_x < 10'd2) || (local_x >= 10'd158) ||
                 (local_y < 10'd2) || (local_y >= 10'd118))) begin
                focus_border = 1'b1;
            end
            focus_border = focus_border | status_marker_hit;
        end
    end
endmodule
