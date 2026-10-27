`timescale 1ns/1ps

// Raw PCM stream player for 48 kHz, signed 16-bit, mono, little-endian data.
// The memory is a dual-clock circular FIFO: SD bytes are accepted in the
// mem_clk domain and samples are consumed in the pixel_clk domain.
module wav_stream_player #(
    parameter integer ADDR_WIDTH = 12,
    parameter integer SAMPLES_PER_SECTOR = 256
) (
    // Metadata and playback control, sampled in the mem_clk domain.
    input  wire        mem_clk,
    input  wire        mem_rst_n,
    input  wire        play_enable,
    input  wire        file_valid,
    input  wire [31:0] start_lba,
    input  wire [31:0] total_sectors,

    // Audio SD client. Hold audio_req and audio_lba until audio_start grants
    // the sector; audio_done completes it. The data stream has no ready port.
    output reg         audio_req,
    output reg  [31:0] audio_lba,
    input  wire        audio_start,
    input  wire [7:0]  audio_data,
    input  wire        audio_valid,
    input  wire        audio_done,

    // Sample clock domain.
    input  wire        pixel_clk,
    input  wire        pixel_rst_n,
    input  wire        sample_tick,
    output reg signed [15:0] pcm_sample,
    output reg         sample_valid,

    // Status outputs are evaluated in the pixel_clk domain.
    output wire        stream_active,
    output wire        buffer_low,
    output wire signed [15:0] sample_out,
    output wire        stream_end
);
    localparam integer FIFO_DEPTH = (1 << ADDR_WIDTH);
    localparam integer PTR_WIDTH = ADDR_WIDTH + 1;
    localparam integer REQUEST_LIMIT =
        (FIFO_DEPTH > SAMPLES_PER_SECTOR) ?
        (FIFO_DEPTH - SAMPLES_PER_SECTOR) : 0;
    localparam integer LOW_WATERMARK = FIFO_DEPTH / 4;

    localparam [1:0] WS_IDLE = 2'd0;
    localparam [1:0] WS_REQ  = 2'd1;
    localparam [1:0] WS_FILL = 2'd2;
    localparam [1:0] WS_END  = 2'd3;

    (* ram_style = "block" *) reg signed [15:0] fifo_mem [0:FIFO_DEPTH-1];

    // Binary and Gray pointers have one extra wrap bit. The binary pointer's
    // low ADDR_WIDTH bits address the RAM; the extra bit disambiguates full
    // and empty after a complete FIFO turn.
    reg [PTR_WIDTH-1:0] wr_bin;
    reg [PTR_WIDTH-1:0] wr_gray;
    reg [PTR_WIDTH-1:0] rd_bin;
    reg [PTR_WIDTH-1:0] rd_gray;

    (* async_reg = "true" *) reg [PTR_WIDTH-1:0] rd_gray_sync1;
    (* async_reg = "true" *) reg [PTR_WIDTH-1:0] rd_gray_sync2;
    (* async_reg = "true" *) reg [PTR_WIDTH-1:0] wr_gray_sync1;
    (* async_reg = "true" *) reg [PTR_WIDTH-1:0] wr_gray_sync2;

    wire [PTR_WIDTH-1:0] rd_bin_in_mem = gray_to_bin(rd_gray_sync2);
    wire [PTR_WIDTH-1:0] wr_bin_in_pixel = gray_to_bin(wr_gray_sync2);
    wire [PTR_WIDTH-1:0] level_mem = wr_bin - rd_bin_in_mem;
    wire [PTR_WIDTH-1:0] level_pixel = wr_bin_in_pixel - rd_bin;
    wire fifo_full = (level_mem >= FIFO_DEPTH);
    wire fifo_can_request = (level_mem <= REQUEST_LIMIT);

    function [PTR_WIDTH-1:0] bin_to_gray;
        input [PTR_WIDTH-1:0] value;
        begin
            bin_to_gray = value ^ (value >> 1);
        end
    endfunction

    function [PTR_WIDTH-1:0] gray_to_bin;
        input [PTR_WIDTH-1:0] value;
        integer idx;
        begin
            gray_to_bin[PTR_WIDTH-1] = value[PTR_WIDTH-1];
            for (idx = PTR_WIDTH-2; idx >= 0; idx = idx - 1)
                gray_to_bin[idx] = gray_to_bin[idx+1] ^ value[idx];
        end
    endfunction

    reg [1:0]  write_state;
    reg [31:0] current_lba;
    reg [31:0] sectors_read;
    reg [7:0]  low_byte;
    reg        low_byte_pending;
    reg [9:0]  sector_byte_count;

    // Only one-bit status flags cross; no binary FSM encoding is sampled
    // asynchronously in the read domain.
    (* async_reg = "true" *) reg started_sync1;
    (* async_reg = "true" *) reg started_sync2;
    (* async_reg = "true" *) reg ended_sync1;
    (* async_reg = "true" *) reg ended_sync2;
    (* async_reg = "true" *) reg play_sync1;
    (* async_reg = "true" *) reg play_sync2;

    assign sample_out = pcm_sample;
    assign stream_active = play_sync2 && started_sync2 &&
                           (!ended_sync2 || level_pixel != 0);
    assign stream_end = play_sync2 && ended_sync2 && (level_pixel == 0);
    assign buffer_low = stream_active && (level_pixel < LOW_WATERMARK);

    // mem_clk domain: request one sector at a time and assemble bytes into
    // samples. A whole-sector amount of free space is reserved before a new
    // request, so normal SD traffic cannot overwrite unread samples.
    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            audio_req         <= 1'b0;
            audio_lba         <= 32'd0;
            write_state       <= WS_IDLE;
            current_lba       <= 32'd0;
            sectors_read      <= 32'd0;
            low_byte          <= 8'd0;
            low_byte_pending  <= 1'b0;
            sector_byte_count <= 10'd0;
            wr_bin            <= {PTR_WIDTH{1'b0}};
            wr_gray           <= {PTR_WIDTH{1'b0}};
            rd_gray_sync1     <= {PTR_WIDTH{1'b0}};
            rd_gray_sync2     <= {PTR_WIDTH{1'b0}};
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;

            if (!play_enable) begin
                audio_req        <= 1'b0;
                write_state      <= WS_IDLE;
                current_lba      <= 32'd0;
                sectors_read     <= 32'd0;
                low_byte_pending <= 1'b0;
                sector_byte_count <= 10'd0;
                wr_bin           <= {PTR_WIDTH{1'b0}};
                wr_gray          <= {PTR_WIDTH{1'b0}};
            end else begin
                case (write_state)
                    WS_IDLE: begin
                        if (file_valid && (total_sectors != 32'd0)) begin
                            current_lba      <= start_lba;
                            sectors_read     <= 32'd0;
                            low_byte_pending <= 1'b0;
                            sector_byte_count <= 10'd0;
                            write_state      <= WS_REQ;
                        end
                    end

                    WS_REQ: begin
                        if (!audio_req && file_valid &&
                            (total_sectors != 32'd0) && fifo_can_request) begin
                            audio_lba   <= current_lba;
                            audio_req   <= 1'b1;
                        end
                        if (audio_req && audio_start) begin
                            audio_req   <= 1'b0;
                            sector_byte_count <= 10'd0;
                            write_state <= WS_FILL;
                        end
                    end

                    WS_FILL: begin
                        if (audio_valid && sector_byte_count < 10'd512) begin
                            sector_byte_count <= sector_byte_count + 1'b1;
                            if (!low_byte_pending) begin
                                low_byte         <= audio_data;
                                low_byte_pending <= 1'b1;
                            end else begin
                                // First byte is the least significant byte.
                                if (!fifo_full) begin
                                    fifo_mem[wr_bin[ADDR_WIDTH-1:0]] <=
                                        {audio_data, low_byte};
                                    wr_bin  <= wr_bin + 1'b1;
                                    wr_gray <= bin_to_gray(wr_bin + 1'b1);
                                end
                                low_byte_pending <= 1'b0;
                            end
                        end

                        if (audio_done) begin
                            // Sectors are 512 bytes, so a well-formed source
                            // ends with no unmatched byte. Discard one if a
                            // malformed client supplies an odd byte count.
                            low_byte_pending <= 1'b0;
                            if (sectors_read + 1'b1 >= total_sectors) begin
                                write_state <= WS_END;
                            end else begin
                                sectors_read <= sectors_read + 1'b1;
                                current_lba  <= current_lba + 1'b1;
                                write_state  <= WS_REQ;
                            end
                        end
                    end

                    WS_END: begin
                        if (!play_enable)
                            write_state <= WS_IDLE;
                    end

                    default: write_state <= WS_IDLE;
                endcase
            end
        end
    end

    // pixel_clk domain: synchronize the write pointer and consume one sample
    // per sample_tick. Underflow deliberately keeps valid asserted and emits
    // zero, preserving timing for the existing HDMI audio chain.
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            rd_bin          <= {PTR_WIDTH{1'b0}};
            rd_gray         <= {PTR_WIDTH{1'b0}};
            wr_gray_sync1   <= {PTR_WIDTH{1'b0}};
            wr_gray_sync2   <= {PTR_WIDTH{1'b0}};
            started_sync1   <= 1'b0;
            started_sync2   <= 1'b0;
            ended_sync1     <= 1'b0;
            ended_sync2     <= 1'b0;
            play_sync1      <= 1'b0;
            play_sync2      <= 1'b0;
            pcm_sample      <= 16'sd0;
            sample_valid    <= 1'b0;
        end else begin
            wr_gray_sync1     <= wr_gray;
            wr_gray_sync2     <= wr_gray_sync1;
            started_sync1    <= (write_state != WS_IDLE);
            started_sync2    <= started_sync1;
            ended_sync1      <= (write_state == WS_END);
            ended_sync2      <= ended_sync1;
            play_sync1       <= play_enable;
            play_sync2       <= play_sync1;

            sample_valid <= 1'b0;
            if (!play_sync2) begin
                rd_bin       <= {PTR_WIDTH{1'b0}};
                rd_gray      <= {PTR_WIDTH{1'b0}};
                pcm_sample   <= 16'sd0;
            end else if (sample_tick) begin
                sample_valid <= 1'b1;
                if (level_pixel != 0) begin
                    pcm_sample <= fifo_mem[rd_bin[ADDR_WIDTH-1:0]];
                    rd_bin     <= rd_bin + 1'b1;
                    rd_gray    <= bin_to_gray(rd_bin + 1'b1);
                end else begin
                    pcm_sample <= 16'sd0;
                end
            end
        end
    end
endmodule
