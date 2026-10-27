`timescale 1ns/1ps

// File-offset parser, not a sector/LBA reader. All ports use clk.
// The source has no ready: every valid byte must be accepted in order.
module wav_parser (
    input  wire               clk,
    input  wire               rst_n,
    input  wire [7:0]         file_byte_data,
    input  wire               file_byte_valid,
    input  wire [31:0]        file_byte_offset,
    input  wire [31:0]        file_size,
    input  wire               file_done,
    input  wire               parse_start,
    output reg                wav_format_valid,
    output wire               format_valid,
    output reg                pcm_valid,
    output reg signed [15:0]  pcm_data,
    output reg  [31:0]        pcm_data_start,
    output reg  [31:0]        pcm_data_length,
    output reg  [7:0]         parser_error,
    output reg  [3:0]         parser_state
);
    localparam [3:0] IDLE         = 4'd0;
    localparam [3:0] RIFF_HEADER  = 4'd1;
    localparam [3:0] CHUNK_HEADER = 4'd2;
    localparam [3:0] FMT_BODY     = 4'd3;
    localparam [3:0] SKIP_BODY    = 4'd4;
    localparam [3:0] PAD          = 4'd5;
    localparam [3:0] PCM_BODY     = 4'd6;
    localparam [3:0] DATA_TAIL    = 4'd7;
    localparam [3:0] ERROR_DRAIN  = 4'd8;

    localparam [7:0] ERR_NONE       = 8'h00;
    localparam [7:0] ERR_SIGNATURE  = 8'h01;
    localparam [7:0] ERR_RIFF_BOUND = 8'h02;
    localparam [7:0] ERR_CHUNK_BOUND= 8'h03;
    localparam [7:0] ERR_FMT_SIZE   = 8'h04;
    localparam [7:0] ERR_FMT_VALUE  = 8'h05;
    localparam [7:0] ERR_FMT_ORDER  = 8'h06;
    localparam [7:0] ERR_DATA_SIZE  = 8'h07;
    localparam [7:0] ERR_NO_FMT     = 8'h08;
    localparam [7:0] ERR_NO_DATA    = 8'h09;
    localparam [7:0] ERR_OFFSET     = 8'h0a;
    localparam [7:0] ERR_TRUNCATED  = 8'h0b;
    localparam [7:0] ERR_STATE      = 8'h0c;

    // IDs are assembled little endian (fmt-space and data).
    localparam [31:0] ID_FMT  = 32'h20746d66;
    localparam [31:0] ID_DATA = 32'h61746164;

    reg [31:0] size_q, size_d;
    reg [32:0] offset_q, offset_d;
    reg [32:0] riff_end_q, riff_end_d;
    reg [31:0] riff_size_q, riff_size_d;
    reg [31:0] chunk_id_q, chunk_id_d;
    reg [31:0] chunk_size_q, chunk_size_d;
    reg [31:0] remaining_q, remaining_d;
    reg [4:0] index_q, index_d;
    reg pad_q, pad_d;
    reg fmt_seen_q, fmt_seen_d;
    reg [15:0] audio_format_q, audio_format_d;
    reg [15:0] channels_q, channels_d;
    reg [31:0] sample_rate_q, sample_rate_d;
    reg [31:0] byte_rate_q, byte_rate_d;
    reg [15:0] block_align_q, block_align_d;
    reg [15:0] bits_q, bits_d;
    reg [7:0] low_q, low_d;
    reg half_q, half_d;

    reg [3:0] state_d;
    reg valid_d, pcm_valid_d;
    reg signed [15:0] pcm_d;
    reg [31:0] data_start_d, data_length_d;
    reg [7:0] error_d;

    wire [32:0] byte_end = offset_q + 33'd1;
    wire [32:0] header_end = offset_q + 33'd8;
    wire [31:0] complete_chunk_size = {file_byte_data, chunk_size_q[23:0]};
    wire [32:0] payload_end = byte_end + {1'b0, complete_chunk_size};
    wire [32:0] padded_end = payload_end + {32'd0, complete_chunk_size[0]};

    assign format_valid = wav_format_valid;

    // Combinational helpers operate only on next-state registers.
    task fail_parse;
        input [7:0] code;
        begin
            if (error_d == ERR_NONE)
                error_d = code;
            valid_d = 1'b0;
            pcm_valid_d = 1'b0;
            half_d = 1'b0;
            state_d = ERROR_DRAIN;
        end
    endtask

    task next_chunk;
        input [32:0] next_offset;
        begin
            index_d = 5'd0;
            chunk_id_d = 32'd0;
            chunk_size_d = 32'd0;
            state_d = CHUNK_HEADER;
            if (next_offset == riff_end_d) begin
                if (!fmt_seen_d)
                    fail_parse(ERR_NO_FMT);
                else
                    fail_parse(ERR_NO_DATA);
            end else if (next_offset + 33'd8 > riff_end_d) begin
                fail_parse(ERR_CHUNK_BOUND);
            end
        end
    endtask

    always @(*) begin
        state_d = parser_state;
        valid_d = wav_format_valid;
        pcm_valid_d = 1'b0; // Write event is one clock, including across gaps.
        pcm_d = pcm_data;
        data_start_d = pcm_data_start;
        data_length_d = pcm_data_length;
        error_d = parser_error;
        size_d = size_q;
        offset_d = offset_q;
        riff_end_d = riff_end_q;
        riff_size_d = riff_size_q;
        chunk_id_d = chunk_id_q;
        chunk_size_d = chunk_size_q;
        remaining_d = remaining_q;
        index_d = index_q;
        pad_d = pad_q;
        fmt_seen_d = fmt_seen_q;
        audio_format_d = audio_format_q;
        channels_d = channels_q;
        sample_rate_d = sample_rate_q;
        byte_rate_d = byte_rate_q;
        block_align_d = block_align_q;
        bits_d = bits_q;
        low_d = low_q;
        half_d = half_q;

        if (parser_state == IDLE && parse_start) begin
            state_d = RIFF_HEADER;
            valid_d = 1'b0;
            pcm_d = 16'sd0;
            data_start_d = 32'd0;
            data_length_d = 32'd0;
            error_d = ERR_NONE;
            size_d = file_size;
            offset_d = 33'd0;
            riff_end_d = 33'd0;
            riff_size_d = 32'd0;
            chunk_id_d = 32'd0;
            chunk_size_d = 32'd0;
            remaining_d = 32'd0;
            index_d = 5'd0;
            pad_d = 1'b0;
            fmt_seen_d = 1'b0;
            audio_format_d = 16'd0;
            channels_d = 16'd0;
            sample_rate_d = 32'd0;
            byte_rate_d = 32'd0;
            block_align_d = 16'd0;
            bits_d = 16'd0;
            low_d = 8'd0;
            half_d = 1'b0;
            if (file_size < 32'd12)
                fail_parse(ERR_TRUNCATED);
        end else if (parser_state != IDLE) begin
            if (file_byte_valid && parser_state != ERROR_DRAIN) begin
                if ({1'b0, file_byte_offset} != offset_q ||
                    offset_q >= {1'b0, size_q}) begin
                    fail_parse(ERR_OFFSET);
                end else begin
                    offset_d = byte_end;
                    case (parser_state)
                        RIFF_HEADER: begin
                            index_d = index_q + 5'd1;
                            case (index_q)
                                5'd0: if (file_byte_data != 8'h52) fail_parse(ERR_SIGNATURE);
                                5'd1: if (file_byte_data != 8'h49) fail_parse(ERR_SIGNATURE);
                                5'd2, 5'd3: if (file_byte_data != 8'h46) fail_parse(ERR_SIGNATURE);
                                5'd4: riff_size_d[7:0] = file_byte_data;
                                5'd5: riff_size_d[15:8] = file_byte_data;
                                5'd6: riff_size_d[23:16] = file_byte_data;
                                5'd7: begin
                                    riff_size_d[31:24] = file_byte_data;
                                    riff_end_d = {1'b0, riff_size_d} + 33'd8;
                                    if (riff_size_d < 32'd4 || riff_end_d > {1'b0, size_q})
                                        fail_parse(ERR_RIFF_BOUND);
                                end
                                5'd8: if (file_byte_data != 8'h57) fail_parse(ERR_SIGNATURE);
                                5'd9: if (file_byte_data != 8'h41) fail_parse(ERR_SIGNATURE);
                                5'd10: if (file_byte_data != 8'h56) fail_parse(ERR_SIGNATURE);
                                5'd11: begin
                                    if (file_byte_data != 8'h45)
                                        fail_parse(ERR_SIGNATURE);
                                    else
                                        next_chunk(byte_end);
                                end
                                default: fail_parse(ERR_STATE);
                            endcase
                        end

                        CHUNK_HEADER: begin
                            index_d = index_q + 5'd1;
                            if (header_end > riff_end_q && index_q == 5'd0) begin
                                fail_parse(ERR_CHUNK_BOUND);
                            end else begin
                                case (index_q)
                                    5'd0: chunk_id_d[7:0] = file_byte_data;
                                    5'd1: chunk_id_d[15:8] = file_byte_data;
                                    5'd2: chunk_id_d[23:16] = file_byte_data;
                                    5'd3: chunk_id_d[31:24] = file_byte_data;
                                    5'd4: chunk_size_d[7:0] = file_byte_data;
                                    5'd5: chunk_size_d[15:8] = file_byte_data;
                                    5'd6: chunk_size_d[23:16] = file_byte_data;
                                    5'd7: begin
                                        chunk_size_d = complete_chunk_size;
                                        remaining_d = complete_chunk_size;
                                        pad_d = complete_chunk_size[0];
                                        index_d = 5'd0;
                                        if (payload_end > riff_end_q || padded_end > riff_end_q ||
                                            padded_end > {1'b0, size_q}) begin
                                            fail_parse(ERR_CHUNK_BOUND);
                                        end else if (chunk_id_q == ID_FMT) begin
                                            if (fmt_seen_q)
                                                fail_parse(ERR_FMT_ORDER);
                                            else if (complete_chunk_size < 32'd16)
                                                fail_parse(ERR_FMT_SIZE);
                                            else
                                                state_d = FMT_BODY;
                                        end else if (chunk_id_q == ID_DATA) begin
                                            if (!fmt_seen_q)
                                                fail_parse(ERR_FMT_ORDER);
                                            else if (complete_chunk_size == 32'd0 || complete_chunk_size[0])
                                                fail_parse(ERR_DATA_SIZE);
                                            else begin
                                                data_start_d = byte_end[31:0];
                                                data_length_d = complete_chunk_size;
                                                valid_d = 1'b1;
                                                half_d = 1'b0;
                                                state_d = PCM_BODY;
                                            end
                                        end else if (complete_chunk_size == 32'd0) begin
                                            next_chunk(byte_end);
                                        end else begin
                                            state_d = SKIP_BODY;
                                        end
                                    end
                                    default: fail_parse(ERR_STATE);
                                endcase
                            end
                        end

                        FMT_BODY: begin
                            remaining_d = remaining_q - 32'd1;
                            index_d = index_q + 5'd1;
                            case (index_q)
                                5'd0: audio_format_d[7:0] = file_byte_data;
                                5'd1: audio_format_d[15:8] = file_byte_data;
                                5'd2: channels_d[7:0] = file_byte_data;
                                5'd3: channels_d[15:8] = file_byte_data;
                                5'd4: sample_rate_d[7:0] = file_byte_data;
                                5'd5: sample_rate_d[15:8] = file_byte_data;
                                5'd6: sample_rate_d[23:16] = file_byte_data;
                                5'd7: sample_rate_d[31:24] = file_byte_data;
                                5'd8: byte_rate_d[7:0] = file_byte_data;
                                5'd9: byte_rate_d[15:8] = file_byte_data;
                                5'd10: byte_rate_d[23:16] = file_byte_data;
                                5'd11: byte_rate_d[31:24] = file_byte_data;
                                5'd12: block_align_d[7:0] = file_byte_data;
                                5'd13: block_align_d[15:8] = file_byte_data;
                                5'd14: bits_d[7:0] = file_byte_data;
                                5'd15: begin
                                    bits_d[15:8] = file_byte_data;
                                    if (audio_format_d != 16'd1 || channels_d != 16'd1 ||
                                        sample_rate_d != 32'd48000 || byte_rate_d != 32'd96000 ||
                                        block_align_d != 16'd2 || bits_d != 16'd16) begin
                                        fail_parse(ERR_FMT_VALUE);
                                    end else begin
                                        fmt_seen_d = 1'b1;
                                        if (remaining_q > 32'd1)
                                            state_d = SKIP_BODY;
                                        else if (pad_q)
                                            state_d = PAD;
                                        else
                                            next_chunk(byte_end);
                                    end
                                end
                                default: fail_parse(ERR_STATE);
                            endcase
                        end

                        SKIP_BODY: begin
                            remaining_d = remaining_q - 32'd1;
                            if (remaining_q == 32'd1) begin
                                if (pad_q)
                                    state_d = PAD;
                                else
                                    next_chunk(byte_end);
                            end
                        end

                        PAD: begin
                            pad_d = 1'b0;
                            next_chunk(byte_end);
                        end

                        PCM_BODY: begin
                            remaining_d = remaining_q - 32'd1;
                            if (!half_q) begin
                                low_d = file_byte_data;
                                half_d = 1'b1;
                            end else begin
                                pcm_d = $signed({file_byte_data, low_q});
                                pcm_valid_d = 1'b1;
                                half_d = 1'b0;
                            end
                            if (remaining_q == 32'd1)
                                state_d = DATA_TAIL;
                        end

                        DATA_TAIL: begin
                            // First data only; tail bytes are drained, not parsed.
                        end
                        default: fail_parse(ERR_STATE);
                    endcase
                end
            end

            // Evaluate AFTER this beat, preserving a final high byte/sample.
            if (file_done) begin
                if (error_d != ERR_NONE) begin
                    state_d = IDLE;
                end else if (offset_d != {1'b0, size_q} ||
                             state_d != DATA_TAIL || half_d || !valid_d) begin
                    fail_parse(ERR_TRUNCATED);
                    state_d = IDLE;
                end else begin
                    state_d = IDLE;
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            parser_state <= IDLE;
            wav_format_valid <= 1'b0;
            pcm_valid <= 1'b0;
            pcm_data <= 16'sd0;
            pcm_data_start <= 32'd0;
            pcm_data_length <= 32'd0;
            parser_error <= ERR_NONE;
            size_q <= 32'd0;
            offset_q <= 33'd0;
            riff_end_q <= 33'd0;
            riff_size_q <= 32'd0;
            chunk_id_q <= 32'd0;
            chunk_size_q <= 32'd0;
            remaining_q <= 32'd0;
            index_q <= 5'd0;
            pad_q <= 1'b0;
            fmt_seen_q <= 1'b0;
            audio_format_q <= 16'd0;
            channels_q <= 16'd0;
            sample_rate_q <= 32'd0;
            byte_rate_q <= 32'd0;
            block_align_q <= 16'd0;
            bits_q <= 16'd0;
            low_q <= 8'd0;
            half_q <= 1'b0;
        end else begin
            parser_state <= state_d;
            wav_format_valid <= valid_d;
            pcm_valid <= pcm_valid_d;
            pcm_data <= pcm_d;
            pcm_data_start <= data_start_d;
            pcm_data_length <= data_length_d;
            parser_error <= error_d;
            size_q <= size_d;
            offset_q <= offset_d;
            riff_end_q <= riff_end_d;
            riff_size_q <= riff_size_d;
            chunk_id_q <= chunk_id_d;
            chunk_size_q <= chunk_size_d;
            remaining_q <= remaining_d;
            index_q <= index_d;
            pad_q <= pad_d;
            fmt_seen_q <= fmt_seen_d;
            audio_format_q <= audio_format_d;
            channels_q <= channels_d;
            sample_rate_q <= sample_rate_d;
            byte_rate_q <= byte_rate_d;
            block_align_q <= block_align_d;
            bits_q <= bits_d;
            low_q <= low_d;
            half_q <= half_d;
        end
    end
endmodule
