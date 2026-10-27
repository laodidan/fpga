`timescale 1ns/1ps

// Single-session playback completion in the HDMI pixel clock domain.
// pcm_sample_consume qualifies the final real WAV sample's completed HDMI
// audio packet, not a FIFO read or an ordinary sample/packet handshake.
module wav_done_detector (
    input  wire clk,
    input  wire rst_n,
    input  wire parser_pcm_end,
    input  wire fifo_empty,
    input  wire pcm_sample_consume,
    output reg  wav_done
);
    localparam [2:0] PLAYING              = 3'd0;
    localparam [2:0] PARSER_END_WAIT_FIFO = 3'd1;
    localparam [2:0] WAIT_LAST_SAMPLE     = 3'd2;
    localparam [2:0] DONE_PULSE           = 3'd3;
    localparam [2:0] DONE_HOLD            = 3'd4;
    reg [2:0] state;
    reg       last_sample_seen;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= PLAYING;
            last_sample_seen <= 1'b0;
            wav_done         <= 1'b0;
        end else begin
            wav_done <= 1'b0;
            case (state)
                PLAYING: begin
                    if (pcm_sample_consume)
                        last_sample_seen <= 1'b1;
                    if (parser_pcm_end)
                        state <= PARSER_END_WAIT_FIFO;
                end
                PARSER_END_WAIT_FIFO: begin
                    if (!parser_pcm_end) begin
                        state            <= PLAYING;
                        last_sample_seen <= 1'b0;
                    end else begin
                        if (pcm_sample_consume)
                            last_sample_seen <= 1'b1;
                        if (fifo_empty)
                            state <= WAIT_LAST_SAMPLE;
                    end
                end
                WAIT_LAST_SAMPLE: begin
                    if (!parser_pcm_end) begin
                        state            <= PLAYING;
                        last_sample_seen <= 1'b0;
                    end else begin
                        if (pcm_sample_consume)
                            last_sample_seen <= 1'b1;
                        if (!fifo_empty)
                            state <= PARSER_END_WAIT_FIFO;
                        else if (last_sample_seen || pcm_sample_consume)
                            state <= DONE_PULSE;
                    end
                end
                DONE_PULSE: begin
                    if (!parser_pcm_end) begin
                        state            <= PLAYING;
                        last_sample_seen <= 1'b0;
                    end else if (!fifo_empty) begin
                        state <= PARSER_END_WAIT_FIFO;
                    end else begin
                        wav_done <= 1'b1;
                        state    <= DONE_HOLD;
                    end
                end
                DONE_HOLD: begin
                    state <= DONE_HOLD;
                end
                default: begin
                    state            <= PLAYING;
                    last_sample_seen <= 1'b0;
                end
            endcase
        end
    end
endmodule
