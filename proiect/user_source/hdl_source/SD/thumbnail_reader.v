`timescale 1ns/1ps

// Continuous thumbnail reader.  Pixel time only selects a cached source row;
// the memory side fills one complete 640-word row at a time using five fixed
// 128-word bursts.  Two line buffers allow one row to be displayed while the
// other row is being filled.
module thumbnail_reader #(
    parameter integer ADDR_WIDTH = 21,
    parameter integer IMAGE_WORDS = 307200,
    parameter integer IMAGE_WIDTH = 640,
    parameter integer LINE_WORDS = 640,
    parameter integer BURST_SIZE = 128
) (
    input wire mem_clk,
    input wire mem_rst_n,
    input wire controller_init_done,
    input wire controller_busy,
    input wire refresh_active,
    input wire burst_grant,
    output wire burst_request,
    output wire burst_running,
    output reg app_rd_en,
    output reg [ADDR_WIDTH-1:0] app_rd_addr,
    input wire sdram_rd_valid,
    input wire [31:0] sdram_rd_data,

    input wire pix_clk,
    input wire pix_rst_n,
    input wire [1:0] source_index,
    input wire [9:0] source_x,
    input wire [9:0] source_y,
    input wire thumbnail_de,
    input wire [3:0] image_complete,
    input wire reader_enable,
    input wire demand_valid,
    input wire [1:0] demand_index,
    input wire [9:0] demand_row,
    input wire prefetch_valid,
    input wire [1:0] prefetch_index,
    input wire [9:0] prefetch_row,
    input wire [9:0] pixel_x,
    input wire [9:0] pixel_y,
    output wire [23:0] thumbnail_rgb,
    output wire thumbnail_valid,
    output reg underflow,
    output reg [31:0] underflow_count,
    output reg [31:0] burst_count,
    output reg [7:0] reader_error
);
    localparam integer BURSTS_PER_LINE = LINE_WORDS / BURST_SIZE;
    localparam integer BURST_COUNT_W = (BURSTS_PER_LINE < 2) ? 1 : $clog2(BURSTS_PER_LINE);
    localparam integer LINE_COUNT_W = (LINE_WORDS < 2) ? 1 : $clog2(LINE_WORDS + 1);

    wire [31:0] line_word0_pix, line_word1_pix;
    reg line_select0_pix_q;
    reg thumbnail_valid_pix_q;

    // Pixel to memory request handshake.  Descriptor fields remain stable
    // until the completion toggle returns to the pixel clock domain.
    reg req_toggle_pix, req_inflight_pix, replace_buf_pix;
    reg [1:0] req_bank_pix;
    reg [9:0] req_row_pix;
    reg req_buf_pix;
    reg tag_valid0_pix, tag_valid1_pix;
    reg [1:0] tag_bank0_pix, tag_bank1_pix;
    reg [9:0] tag_row0_pix, tag_row1_pix;
    (* async_reg = "true" *) reg req_sync1_mem, req_sync2_mem;
    reg req_seen_mem;
    (* async_reg = "true" *) reg done_sync1_pix, done_sync2_pix;
    reg done_seen_pix;
    reg done_toggle_mem;
    reg done_buf_mem;
    reg [1:0] done_bank_mem;
    reg [9:0] done_row_mem;

    wire tag_hit0_pix = tag_valid0_pix && tag_bank0_pix == source_index &&
                        tag_row0_pix == source_y;
    wire tag_hit1_pix = tag_valid1_pix && tag_bank1_pix == source_index &&
                        tag_row1_pix == source_y;
    wire cache_ready_pix = tag_hit0_pix || tag_hit1_pix;
    wire source_in_range_pix = source_x < 10'd640 && source_y < 10'd480;
    wire demand_ready_pix = demand_valid && demand_row < 10'd480 &&
                            image_complete[demand_index];
    wire demand_hit0_pix = tag_valid0_pix && tag_bank0_pix == demand_index &&
                           tag_row0_pix == demand_row;
    wire demand_hit1_pix = tag_valid1_pix && tag_bank1_pix == demand_index &&
                           tag_row1_pix == demand_row;
    wire demand_cached_pix = demand_hit0_pix || demand_hit1_pix;
    wire demand_request_pix = demand_ready_pix && !demand_cached_pix;
    wire prefetch_hit0_pix = tag_valid0_pix && tag_bank0_pix == prefetch_index &&
                             tag_row0_pix == prefetch_row;
    wire prefetch_hit1_pix = tag_valid1_pix && tag_bank1_pix == prefetch_index &&
                             tag_row1_pix == prefetch_row;
    wire prefetch_request_pix = (!demand_ready_pix || demand_cached_pix) &&
                                prefetch_valid && prefetch_row < 10'd480 &&
                                image_complete[prefetch_index] &&
                                !prefetch_hit0_pix && !prefetch_hit1_pix;
    // Preserve the demanded row while its successor is filled.  An invalid
    // buffer is always preferred; round-robin only applies when neither
    // valid buffer holds the current demand.
    wire request_buf_pix = !tag_valid0_pix ? 1'b0 :
                           !tag_valid1_pix ? 1'b1 :
                           demand_hit0_pix ? 1'b1 :
                           demand_hit1_pix ? 1'b0 : replace_buf_pix;
    wire [9:0] cache_x_pix = source_in_range_pix ? source_x : 10'd0;
    wire [31:0] active_word_pix = line_select0_pix_q ?
                                   line_word0_pix : line_word1_pix;

    assign thumbnail_valid = thumbnail_valid_pix_q;
    assign thumbnail_rgb = thumbnail_valid ? active_word_pix[23:0] : 24'd0;

    // Both RAMs sample cache_x on this same edge.  Capture the tag choice
    // and qualification before tag updates, so RGB and valid describe the
    // same input pixel transaction after the synchronous read completes.
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            line_select0_pix_q <= 1'b0;
            thumbnail_valid_pix_q <= 1'b0;
        end else begin
            line_select0_pix_q <= tag_hit0_pix;
            thumbnail_valid_pix_q <= thumbnail_de && source_in_range_pix &&
                                     image_complete[source_index] && cache_ready_pix;
        end
    end

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            req_toggle_pix <= 1'b0;
            req_inflight_pix <= 1'b0;
            replace_buf_pix <= 1'b0;
            req_bank_pix <= 2'd0;
            req_row_pix <= 10'd0;
            req_buf_pix <= 1'b0;
            tag_valid0_pix <= 1'b0;
            tag_valid1_pix <= 1'b0;
            tag_bank0_pix <= 2'd0;
            tag_bank1_pix <= 2'd0;
            tag_row0_pix <= 10'd0;
            tag_row1_pix <= 10'd0;
            done_sync1_pix <= 1'b0;
            done_sync2_pix <= 1'b0;
            done_seen_pix <= 1'b0;
            underflow <= 1'b0;
            underflow_count <= 32'd0;
        end else begin
            done_sync1_pix <= done_toggle_mem;
            done_sync2_pix <= done_sync1_pix;

            if (done_sync2_pix != done_seen_pix) begin
                done_seen_pix <= done_sync2_pix;
                req_inflight_pix <= 1'b0;
                if (done_buf_mem == 1'b0) begin
                    tag_valid0_pix <= 1'b1;
                    tag_bank0_pix <= done_bank_mem;
                    tag_row0_pix <= done_row_mem;
                end else begin
                    tag_valid1_pix <= 1'b1;
                    tag_bank1_pix <= done_bank_mem;
                    tag_row1_pix <= done_row_mem;
                end
            end

            // Descriptors are supplied during blanking as well as active
            // pixels, so the first visible row can be ready before DE.
            if (reader_enable && !req_inflight_pix &&
                (demand_request_pix || prefetch_request_pix)) begin
                req_bank_pix <= demand_request_pix ? demand_index : prefetch_index;
                req_row_pix <= demand_request_pix ? demand_row : prefetch_row;
                req_buf_pix <= request_buf_pix;
                req_toggle_pix <= ~req_toggle_pix;
                req_inflight_pix <= 1'b1;
                replace_buf_pix <= ~request_buf_pix;
                if (request_buf_pix == 1'b0)
                    tag_valid0_pix <= 1'b0;
                else
                    tag_valid1_pix <= 1'b0;
            end

            if (thumbnail_de && source_in_range_pix && image_complete[source_index] &&
                !cache_ready_pix) begin
                underflow <= 1'b1;
                underflow_count <= underflow_count + 1'b1;
            end
        end
    end

    // Memory clock side owns the SDRAM burst and writes the selected line
    // buffer.  A request remains pending until the complete line is returned.
    reg [1:0] burst_state;
    localparam [1:0] MEM_IDLE = 2'd0, MEM_ISSUE = 2'd1, MEM_DRAIN = 2'd2;
    reg [1:0] line_bank_mem;
    reg [9:0] line_row_mem;
    reg line_buf_mem;
    reg [BURST_COUNT_W-1:0] burst_no_mem;
    reg [LINE_COUNT_W-1:0] issued_mem;
    reg [LINE_COUNT_W-1:0] returned_mem;
    reg [ADDR_WIDTH-1:0] line_base_mem;
    reg line_busy_mem;

    // Control fills only 0..LINE_WORDS-1; the physical RAM depth is 1024.
    wire line_write_mem = mem_rst_n && (burst_state == MEM_ISSUE) &&
                          sdram_rd_valid && (returned_mem < LINE_WORDS);
    wire [9:0] line_write_addr_mem = returned_mem;

    thumbnail_line_ram u_line_buf0 (
        .wr_clk(mem_clk),
        .wr_en(line_write_mem && !line_buf_mem),
        .wr_addr(line_write_addr_mem),
        .wr_data(sdram_rd_data),
        .rd_clk(pix_clk),
        .rd_addr(cache_x_pix),
        .rd_data(line_word0_pix)
    );

    thumbnail_line_ram u_line_buf1 (
        .wr_clk(mem_clk),
        .wr_en(line_write_mem && line_buf_mem),
        .wr_addr(line_write_addr_mem),
        .wr_data(sdram_rd_data),
        .rd_clk(pix_clk),
        .rd_addr(cache_x_pix),
        .rd_data(line_word1_pix)
    );

    function [ADDR_WIDTH-1:0] bank_base;
        input [1:0] bank;
        begin
            case (bank)
                2'd0: bank_base = 0;
                2'd1: bank_base = IMAGE_WORDS;
                2'd2: bank_base = IMAGE_WORDS * 2;
                default: bank_base = IMAGE_WORDS * 3;
            endcase
        end
    endfunction

    wire pending_request_mem = (req_sync2_mem != req_seen_mem);
    assign burst_request = line_busy_mem ? 1'b0 :
                           (controller_init_done && pending_request_mem &&
                            !controller_busy && !refresh_active);
    assign burst_running = line_busy_mem || (burst_state != MEM_IDLE) || app_rd_en;

    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            req_sync1_mem <= 1'b0;
            req_sync2_mem <= 1'b0;
            req_seen_mem <= 1'b0;
            burst_state <= MEM_IDLE;
            line_bank_mem <= 2'd0;
            line_row_mem <= 10'd0;
            line_buf_mem <= 1'b0;
            burst_no_mem <= 0;
            issued_mem <= 0;
            returned_mem <= 0;
            line_base_mem <= 0;
            line_busy_mem <= 1'b0;
            app_rd_en <= 1'b0;
            app_rd_addr <= 0;
            done_toggle_mem <= 1'b0;
            done_buf_mem <= 1'b0;
            done_bank_mem <= 2'd0;
            done_row_mem <= 10'd0;
            burst_count <= 32'd0;
            reader_error <= 8'd0;
        end else begin
            req_sync1_mem <= req_toggle_pix;
            req_sync2_mem <= req_sync1_mem;
            app_rd_en <= 1'b0;

            if (burst_state == MEM_IDLE && burst_grant && pending_request_mem &&
                controller_init_done && !controller_busy && !refresh_active) begin
                req_seen_mem <= req_sync2_mem;
                line_bank_mem <= req_bank_pix;
                line_row_mem <= req_row_pix;
                line_buf_mem <= req_buf_pix;
                burst_no_mem <= 0;
                issued_mem <= 0;
                returned_mem <= 0;
                line_base_mem <= bank_base(req_bank_pix) +
                                 req_row_pix * IMAGE_WIDTH;
                line_busy_mem <= 1'b1;
                burst_state <= MEM_ISSUE;
                burst_count <= burst_count + BURSTS_PER_LINE;
            end

            if (burst_state == MEM_ISSUE) begin
                // Keep issuing contiguous reads for the complete line while
                // accepting returned words in parallel.  This preserves the
                // five 128-word burst shape but gives one word/cycle line
                // throughput instead of a separate issue/drain phase.
                if (issued_mem < LINE_WORDS) begin
                    app_rd_en <= 1'b1;
                    app_rd_addr <= line_base_mem + issued_mem;
                    issued_mem <= issued_mem + 1'b1;
                end

                if (sdram_rd_valid && returned_mem < LINE_WORDS) begin
                    returned_mem <= returned_mem + 1'b1;
                    if (returned_mem == LINE_WORDS-1) begin
                        burst_state <= MEM_IDLE;
                        line_busy_mem <= 1'b0;
                        done_buf_mem <= line_buf_mem;
                        done_bank_mem <= line_bank_mem;
                        done_row_mem <= line_row_mem;
                        done_toggle_mem <= ~done_toggle_mem;
                    end
                end
            end
        end
    end

    // Keep these ports in the contract so arbitration and diagnostics can be
    // extended without changing the reader's data path.
    wire unused_pixel_coords = &{pixel_x[0], pixel_y[0], 1'b0};
endmodule
