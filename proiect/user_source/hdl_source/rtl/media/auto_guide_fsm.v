`timescale 1ns/1ps

// Completion-driven guide control. All inputs are in the SD clock domain;
// wav_done and audio_started qualify only the current committed session.
module auto_guide_fsm (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       wav_done,
    input  wire       auto_enable,
    input  wire [1:0] active_index,
    input  wire       active_valid,
    input  wire       index_commit,
    input  wire       audio_started,
    input  wire       request_accepted,
    input  wire       switch_pending,
    output wire       auto_next_req,
    output reg  [1:0] next_index
);
    localparam [2:0] IDLE             = 3'd0;
    localparam [2:0] WAIT_WAV_DONE    = 3'd1;
    localparam [2:0] REQUEST_NEXT     = 3'd2;
    localparam [2:0] WAIT_COMMIT      = 3'd3;
    localparam [2:0] WAIT_AUDIO_START = 3'd4;

    reg [2:0] state;
    reg [1:0] origin_index;

    // The pending target is retained across backpressure. This is a normal
    // valid/ready source: ready is observed by the controller, while valid
    // remains asserted so a missing resource is reported rather than
    // silently hiding the request.
    assign auto_next_req = rst_n && auto_enable && active_valid &&
                           !index_commit && !switch_pending &&
                           (state == REQUEST_NEXT) &&
                           (active_index == origin_index);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            next_index   <= 2'd0;
            origin_index <= 2'd0;
        end else if (!auto_enable) begin
            state <= IDLE;
        end else if (index_commit) begin
            // A same-number commit still denotes a distinct audio session.
            state        <= WAIT_AUDIO_START;
            origin_index <= active_index;
        end else begin
            case (state)
                IDLE: begin
                    if (active_valid && audio_started && !switch_pending) begin
                        origin_index <= active_index;
                        if (wav_done) begin
                            next_index <= active_index + 2'd1;
                            state      <= REQUEST_NEXT;
                        end else begin
                            state <= WAIT_WAV_DONE;
                        end
                    end
                end

                WAIT_WAV_DONE: begin
                    if (switch_pending) begin
                        state <= WAIT_AUDIO_START;
                    end else if (!active_valid) begin
                        state <= IDLE;
                    end else if (active_index != origin_index) begin
                        origin_index <= active_index;
                        state        <= WAIT_AUDIO_START;
                    end else if (!audio_started) begin
                        state <= WAIT_AUDIO_START;
                    end else if (wav_done) begin
                        // Two-bit arithmetic deliberately implements 3 -> 0.
                        next_index   <= active_index + 2'd1;
                        origin_index <= active_index;
                        state        <= REQUEST_NEXT;
                    end
                end

                REQUEST_NEXT: begin
                    if (request_accepted) begin
                        state <= WAIT_COMMIT;
                    end else if (switch_pending) begin
                        state <= WAIT_AUDIO_START;
                    end else if (!active_valid) begin
                        state <= IDLE;
                    end else if (active_index != origin_index) begin
                        origin_index <= active_index;
                        state        <= WAIT_AUDIO_START;
                    end
                end

                WAIT_COMMIT: begin
                    // Neither completion nor a mere index change is an ACK.
                    state <= WAIT_COMMIT;
                end

                WAIT_AUDIO_START: begin
                    if (active_valid && audio_started && !switch_pending) begin
                        origin_index <= active_index;
                        if (wav_done) begin
                            next_index <= active_index + 2'd1;
                            state      <= REQUEST_NEXT;
                        end else begin
                            state <= WAIT_WAV_DONE;
                        end
                    end
                end

                default: begin
                    state        <= IDLE;
                    next_index   <= 2'd0;
                    origin_index <= 2'd0;
                end
            endcase
        end
    end
endmodule
