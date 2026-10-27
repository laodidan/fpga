`timescale 1ns/1ps

// Video-to-audio alpha bridge for the PCM stream. Alpha is captured only at a
// video frame boundary, then synchronized into the audio clock domain. The
// small output FIFO reserves space for every in-flight sample in the four-stage
// sync_fade_gain pipeline and absorbs HDMI packetizer back-pressure.
module sync_fade_gain_video_audio #(
    parameter integer FIFO_DEPTH = 4
) (
    input  wire               pix_clk,
    input  wire               pix_rst_n,
    input  wire               frame_start,
    input  wire [7:0]         alpha_pix,

    input  wire               aud_clk,
    input  wire               aud_rst_n,

    input  wire               pcm_valid_in,
    output wire               pcm_ready_in,
    input  wire signed [15:0] pcm_left_in,
    input  wire signed [15:0] pcm_right_in,

    output wire               pcm_valid_out,
    input  wire               pcm_ready_out,
    output wire signed [15:0] pcm_left_out,
    output wire signed [15:0] pcm_right_out,
    output wire [7:0]         alpha_aud
);
    localparam integer PTR_WIDTH = (FIFO_DEPTH <= 2) ? 1 : $clog2(FIFO_DEPTH);
    localparam integer COUNT_WIDTH = $clog2(FIFO_DEPTH + 1);

    reg [7:0] alpha_pix_latched;
    (* async_reg = "true" *) reg [7:0] alpha_sync1;
    (* async_reg = "true" *) reg [7:0] alpha_sync2;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n)
            alpha_pix_latched <= 8'd255;
        else if (frame_start)
            alpha_pix_latched <= alpha_pix;
    end

    always @(posedge aud_clk or negedge aud_rst_n) begin
        if (!aud_rst_n) begin
            alpha_sync1 <= 8'd255;
            alpha_sync2 <= 8'd255;
        end else begin
            alpha_sync1 <= alpha_pix_latched;
            alpha_sync2 <= alpha_sync1;
        end
    end

    assign alpha_aud = alpha_sync2;

    // Each accepted stereo sample reserves one FIFO entry until it is popped.
    // Track all four gain stages, including the result awaiting its FIFO write.
    wire gain_launch_valid;
    wire gain_left_valid;
    wire gain_right_valid;
    wire signed [15:0] gain_left;
    wire signed [15:0] gain_right;
    reg [3:0] pipeline_valid;

    always @(posedge aud_clk or negedge aud_rst_n) begin
        if (!aud_rst_n)
            pipeline_valid <= 4'b0000;
        else
            pipeline_valid <= {pipeline_valid[2:0], gain_launch_valid};
    end

    wire [2:0] pipeline_valid_count =
        {2'b00, pipeline_valid[0]} + {2'b00, pipeline_valid[1]} +
        {2'b00, pipeline_valid[2]} + {2'b00, pipeline_valid[3]};

    reg [COUNT_WIDTH-1:0] fifo_count;
    reg [PTR_WIDTH-1:0] fifo_wr_ptr;
    reg [PTR_WIDTH-1:0] fifo_rd_ptr;
    reg signed [15:0] fifo_left [0:FIFO_DEPTH-1];
    reg signed [15:0] fifo_right [0:FIFO_DEPTH-1];

    wire fifo_valid = (fifo_count != 0);
    wire fifo_pop = fifo_valid && pcm_ready_out;
    wire gain_valid = gain_left_valid && gain_right_valid;
    // Moving Stage 4 into the FIFO transfers a reservation; it adds no new
    // occupancy. A real output handshake frees one entry for a same-edge launch.
    wire [COUNT_WIDTH+1:0] occupancy_after_current =
        {2'b00, fifo_count} +
        {{(COUNT_WIDTH-1){1'b0}}, pipeline_valid_count} -
        {{(COUNT_WIDTH+1){1'b0}}, fifo_pop};

    assign pcm_ready_in = occupancy_after_current < FIFO_DEPTH;
    assign gain_launch_valid = pcm_valid_in && pcm_ready_in;

    sync_fade_gain u_gain_left (
        .clk(aud_clk), .rst_n(aud_rst_n),
        .pcm_sample_in(pcm_left_in), .sample_valid_in(gain_launch_valid),
        .alpha_in(alpha_sync2), .pcm_sample_out(gain_left),
        .sample_valid_out(gain_left_valid)
    );

    sync_fade_gain u_gain_right (
        .clk(aud_clk), .rst_n(aud_rst_n),
        .pcm_sample_in(pcm_right_in), .sample_valid_in(gain_launch_valid),
        .alpha_in(alpha_sync2), .pcm_sample_out(gain_right),
        .sample_valid_out(gain_right_valid)
    );

    assign pcm_valid_out = fifo_valid;
    assign pcm_left_out = fifo_valid ? fifo_left[fifo_rd_ptr] : 16'sd0;
    assign pcm_right_out = fifo_valid ? fifo_right[fifo_rd_ptr] : 16'sd0;

    function [PTR_WIDTH-1:0] ptr_next;
        input [PTR_WIDTH-1:0] ptr;
        begin
            if (ptr == FIFO_DEPTH-1)
                ptr_next = {PTR_WIDTH{1'b0}};
            else
                ptr_next = ptr + 1'b1;
        end
    endfunction

    always @(posedge aud_clk or negedge aud_rst_n) begin
        if (!aud_rst_n) begin
            fifo_count <= {COUNT_WIDTH{1'b0}};
            fifo_wr_ptr <= {PTR_WIDTH{1'b0}};
            fifo_rd_ptr <= {PTR_WIDTH{1'b0}};
        end else begin
            if (gain_valid) begin
                fifo_left[fifo_wr_ptr] <= gain_left;
                fifo_right[fifo_wr_ptr] <= gain_right;
                fifo_wr_ptr <= ptr_next(fifo_wr_ptr);
            end
            if (fifo_pop)
                fifo_rd_ptr <= ptr_next(fifo_rd_ptr);

            case ({gain_valid, fifo_pop})
                2'b10: fifo_count <= fifo_count + 1'b1;
                2'b01: fifo_count <= fifo_count - 1'b1;
                default: fifo_count <= fifo_count;
            endcase
        end
    end

    // synthesis translate_off
    initial begin
        if (FIFO_DEPTH < 2 || (FIFO_DEPTH & (FIFO_DEPTH - 1)) != 0)
            $fatal(1, "FIFO_DEPTH must be a power of two >= 2");
    end
    // synthesis translate_on
endmodule
