`timescale 1ns/1ps

// Pixel-domain transaction sequencer. The player holds the requested bank
// and type until request_done; ALL visible state changes happen at frame_start.
// Manual: old -> black -> commit -> new. Auto: left new / right old.
module video_fade_transition #(
    parameter integer FADE_FRAMES = 16,
    parameter integer IMAGE_WIDTH = 640
) (
    input wire pix_clk, pix_rst_n, frame_start,
    input wire request_valid,
    input wire [1:0] request_index,
    input wire request_manual,
    output reg request_done,
    output reg [1:0] active_index,
    output reg active_valid,
    output reg index_commit,
    output wire [7:0] alpha_out,
    output wire fade_active,
    output wire wipe_active,
    output reg [1:0] old_index,
    output wire [11:0] wipe_boundary,
    output wire [1:0] transition_kind
);
    localparam integer COUNT_W = (FADE_FRAMES < 2) ? 1 : $clog2(FADE_FRAMES+1);
    localparam integer SAFE_FRAMES = (FADE_FRAMES == 0) ? 1 : FADE_FRAMES;
    localparam [2:0] IDLE=0, OUT=1, HOLD=2, IN=3, WIPE=4;
    reg [2:0] state;
    reg [COUNT_W-1:0] ramp;
    reg [1:0] target_index;
    wire [31:0] alpha_calc = (ramp * 256) / SAFE_FRAMES;
    wire [31:0] boundary_calc = (ramp * IMAGE_WIDTH) / SAFE_FRAMES;
    assign alpha_out = (state==HOLD) ? 8'd0 :
                       ((state==OUT || state==IN) && ramp < FADE_FRAMES) ?
                       alpha_calc[7:0] : 8'd255;
    assign fade_active = state != IDLE;
    assign wipe_active = state == WIPE;
    assign wipe_boundary = (state==WIPE) ? boundary_calc[11:0] : IMAGE_WIDTH;
    assign transition_kind = (state==WIPE) ? 2'd1 : 2'd0;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            state<=IDLE; ramp<=0; target_index<=0;
            active_index<=0; old_index<=0; active_valid<=0;
            index_commit<=0; request_done<=0;
        end else begin
            index_commit<=0;
            request_done<=0;
            if (frame_start) begin
                case (state)
                IDLE: if (request_valid) begin
                    target_index<=request_index;
                    old_index<=active_index;
                    if (!active_valid || FADE_FRAMES==0 || request_index==active_index) begin
                        active_index<=request_index;
                        active_valid<=1;
                        index_commit<=1;
                        request_done<=1;
                    end else if (request_manual) begin
                        ramp<=FADE_FRAMES;
                        state<=OUT;
                    end else begin
                        active_index<=request_index;
                        index_commit<=1;
                        ramp<=0;
                        state<=WIPE;
                    end
                end
                OUT: if (ramp==1) begin
                    ramp<=0;
                    state<=HOLD;
                end else ramp<=ramp-1'b1;
                HOLD: begin
                    // One full old-bank black frame has elapsed. Commit in
                    // blanking; the reader reloads before the new active area.
                    active_index<=target_index;
                    index_commit<=1;
                    ramp<=0;
                    state<=IN;
                end
                IN: if (ramp==FADE_FRAMES-1) begin
                    ramp<=FADE_FRAMES;
                    state<=IDLE;
                    request_done<=1;
                end else ramp<=ramp+1'b1;
                WIPE: if (ramp==FADE_FRAMES-1) begin
                    ramp<=FADE_FRAMES;
                    state<=IDLE;
                    request_done<=1;
                end else ramp<=ramp+1'b1;
                default: state<=IDLE;
                endcase
            end
        end
    end

    // synthesis translate_off
    initial begin
        if (FADE_FRAMES<0 || FADE_FRAMES>256 ||
            (FADE_FRAMES!=0 && (256 % SAFE_FRAMES)!=0))
            $fatal(1,"FADE_FRAMES must be 0 or a divisor of 256");
        if (IMAGE_WIDTH<1 || IMAGE_WIDTH>4095)
            $fatal(1,"IMAGE_WIDTH out of range");
    end
    // synthesis translate_on
endmodule
