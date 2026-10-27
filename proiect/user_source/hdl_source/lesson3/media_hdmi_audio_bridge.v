`timescale 1ns/1ps

// Player-owned boundary between the registered video bundle and the verified
// Route B HDMI packet/symbol chain. The EG4 DDR PHY remains in the board Top so
// this bridge can be exercised in ordinary RTL simulation without vendor pads.
// WAV PCM has priority during a valid session; the media tone remains available
// for fallback/debug. Video registration and the HDMI symbol chain are preserved.
module media_hdmi_audio_bridge (
    input  wire        pixel_clk,
    input  wire        pixel_rst_n,
    input  wire        pcm_rst_n,
    input  wire        audio_session_rst_n,
    input  wire        wav_active,
    input  wire        fifo_empty,
    input  wire        pcm_out_valid,
    input  wire signed [15:0] pcm_out_data,
    output wire        pcm_rd_en,
    output wire        wav_underflow,
    output wire        wav_output_stall,
    input  wire        parser_pcm_end,
    output wire        wav_done,
    output wire        audio_session_tail_drained,
    input  wire        video_hsync,
    input  wire        video_vsync,
    input  wire        video_de,
    input  wire [9:0]  video_x,
    input  wire [23:0] video_rgb,
    input  wire [1:0]  media_id,
    input  wire [7:0]  volume,
    input  wire        volume_key_event,
    input  wire        volume_init_valid,
    input  wire [7:0]  volume_init_value,
    output wire        volume_init_applied,
    output wire [7:0]  volume_level,
    input  wire [7:0]  fade_alpha,
    input  wire        fade_frame_start,
    output wire [9:0]  channel0_symbol,
    output wire [9:0]  channel1_symbol,
    output wire [9:0]  channel2_symbol,
    output wire        sequence_error,
    output wire        tone_overflow,
    output wire        pcm_contract_error,
    output wire        blip_active,
    output wire        pcm_tap_valid,
    output wire signed [15:0] pcm_tap_left
);
    wire tone_valid;
    wire tone_ready;
    wire signed [15:0] tone_left;
    wire signed [15:0] tone_right;
    wire wav_valid, wav_ready;
    wire signed [15:0] wav_left, wav_right;
    wire wav_content_consume, wav_pcm_drained;
    wire audio_packet_done;
    wire tone_mux_ready;
    reg wav_selected;
    wire fade_tone_valid;
    wire fade_tone_ready;
    wire signed [15:0] fade_tone_left;
    wire signed [15:0] fade_tone_right;
    wire mux_valid;
    wire mux_ready;
    wire signed [15:0] mux_left;
    wire signed [15:0] mux_right;
    wire volume_valid;
    wire volume_ready;
    wire signed [15:0] volume_left;
    wire signed [15:0] volume_right;
    wire sample_valid;
    wire sample_ready;
    wire signed [15:0] sample_left;
    wire signed [15:0] sample_right;

    reg video_hsync_q;
    reg video_vsync_q;
    reg video_de_q;
    reg [9:0] video_x_q;
    reg [23:0] video_rgb_q;
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            video_hsync_q <= 1'b1;
            video_vsync_q <= 1'b1;
            video_de_q <= 1'b0;
            video_x_q <= 10'd0;
            video_rgb_q <= 24'h000000;
        end else begin
            video_hsync_q <= video_hsync;
            video_vsync_q <= video_vsync;
            video_de_q <= video_de;
            video_x_q <= video_x;
            video_rgb_q <= video_de ? video_rgb : 24'h000000;
        end
    end

    pcm_media_tone u_tone (
        .clk(pixel_clk), .rst_n(pixel_rst_n),
        .media_id(media_id), .volume(volume),
        .sample_valid(tone_valid), .sample_ready(tone_ready),
        .sample_left(tone_left), .sample_right(tone_right),
        .overflow(tone_overflow), .blip_active(blip_active)
    );

    // FIFO responses survive a temporary HDMI lock reset. Only the PCM global
    // reset clears this queue; pixel_rst_n stops new reads during lock loss.
    pcm_output_ctrl u_pcm_output_ctrl (
        .clk(pixel_clk), .rst_n(audio_session_rst_n),
        .enable(wav_active && pixel_rst_n),
        .fifo_empty(fifo_empty), .pcm_out_valid(pcm_out_valid),
        .pcm_out_data(pcm_out_data), .pcm_rd_en(pcm_rd_en),
        .sample_valid(wav_valid), .sample_ready(wav_ready && pixel_rst_n),
        .sample_left(wav_left), .sample_right(wav_right),
        .underflow(wav_underflow), .output_stall(wav_output_stall),
        .sample_consume_done(wav_content_consume), .pcm_drained(wav_pcm_drained)
    );

    // Never replace an unaccepted mux sample when session ownership changes.
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n)
            wav_selected <= 1'b0;
        else if (!mux_valid || mux_ready)
            wav_selected <= wav_active;
    end

    // Keep the debug generator running while WAV owns the output. Its samples
    // are discarded in the background, so idle tone does not fill its queue.
    assign tone_ready = tone_mux_ready || wav_selected;
    audio_source_mux u_audio_source_mux (
        .clk(pixel_clk), .rst_n(pixel_rst_n),
        .wav_valid(wav_valid), .wav_ready(wav_ready),
        .wav_left(wav_left), .wav_right(wav_right), .wav_active(wav_selected),
        .arp_valid(tone_valid), .arp_ready(tone_mux_ready),
        .arp_left(tone_left), .arp_right(tone_right),
        .force_arpeggio(1'b0),
        .out_valid(mux_valid), .out_ready(mux_ready),
        .out_left(mux_left), .out_right(mux_right),
        .sample_valid(), .pcm_sample()
    );

    // Both sources pass through the same existing gain stage exactly once.
    sync_fade_gain_video_audio u_sync_fade_gain (
        .pix_clk(pixel_clk), .pix_rst_n(pixel_rst_n),
        .frame_start(fade_frame_start), .alpha_pix(fade_alpha),
        .aud_clk(pixel_clk), .aud_rst_n(pixel_rst_n),
        .pcm_valid_in(mux_valid), .pcm_ready_in(mux_ready),
        .pcm_left_in(mux_left), .pcm_right_in(mux_right),
        .pcm_valid_out(fade_tone_valid), .pcm_ready_out(fade_tone_ready),
        .pcm_left_out(fade_tone_left), .pcm_right_out(fade_tone_right),
        .alpha_aud()
    );

    // Apply user volume once to the selected final stereo PCM stream.
    volume_ctrl u_volume_ctrl (
        .clk(pixel_clk), .rst_n(pixel_rst_n),
        .volume_key_event(volume_key_event), .volume_level(volume_level),
        .volume_init_valid(volume_init_valid), .volume_init_value(volume_init_value),
        .volume_init_applied(volume_init_applied),
        .sample_valid_in(fade_tone_valid), .sample_ready_in(fade_tone_ready),
        .pcm_left_in(fade_tone_left), .pcm_right_in(fade_tone_right),
        .sample_valid_out(volume_valid), .sample_ready_out(volume_ready),
        .pcm_left_out(volume_left), .pcm_right_out(volume_right)
    );

    m06_pcm_contract_adapter u_m06_pcm_contract (
        .clk(pixel_clk), .rst_n(pixel_rst_n),
        .source_valid(volume_valid), .source_ready(volume_ready),
        .source_left(volume_left), .source_right(volume_right),
        .pcm_valid(sample_valid), .pcm_ready(sample_ready),
        .pcm_left(sample_left), .pcm_right(sample_right),
        .contract_error(pcm_contract_error)
    );

    hdmi_audio_symbol_core u_symbol_core (
        .pixel_clk(pixel_clk), .rst_n(pixel_rst_n),
        .video_hsync(video_hsync_q), .video_vsync(video_vsync_q),
        .video_de(video_de_q), .video_x(video_x_q), .video_rgb(video_rgb_q),
        .sample_valid(sample_valid), .sample_ready(sample_ready),
        .sample_left(sample_left), .sample_right(sample_right),
        .channel0_symbol(channel0_symbol),
        .channel1_symbol(channel1_symbol),
        .channel2_symbol(channel2_symbol),
        .sequence_error(sequence_error), .audio_packet_done(audio_packet_done)
    );

    // Count the complete ordered stream, including tone and generated silence.
    // Fade, volume and contract stages preserve sample order/count. An audio
    // payload completion accounts for four samples accepted by the packetizer.
    // Recording every content ordinal also covers a tail sent before parser
    // end crosses domains (or while trailing file bytes are still being read).
    reg [63:0] samples_launched;
    reg [63:0] samples_sent;
    reg [63:0] last_content_ordinal;
    reg content_seen;
    reg completion_invalid;
    wire mux_accept = pixel_rst_n && mux_valid && mux_ready;
    wire content_accept = wav_content_consume && wav_selected && pixel_rst_n;
    always @(posedge pixel_clk or negedge pcm_rst_n) begin
        if (!pcm_rst_n) begin
            samples_launched <= 64'd0;
            samples_sent <= 64'd0;
            last_content_ordinal <= 64'd0;
            content_seen <= 1'b0;
            completion_invalid <= 1'b0;
        end else if (!pixel_rst_n) begin
            // HDMI reset discards downstream samples. Never report that
            // discarded WAV content was successfully transmitted.
            samples_launched <= 64'd0;
            samples_sent <= 64'd0;
            if (wav_active || content_seen)
                completion_invalid <= 1'b1;
        end else begin
            if (mux_accept)
                samples_launched <= samples_launched + 64'd1;
            if (audio_packet_done)
                samples_sent <= samples_sent + 64'd4;
            // The HDMI stream keeps running across sessions. Retain its
            // absolute ordinals; reset only the WAV content attribution.
            if (!audio_session_rst_n) begin
                last_content_ordinal <= 64'd0;
                content_seen <= 1'b0;
                completion_invalid <= 1'b0;
            end else begin
                if (content_accept) begin
                    last_content_ordinal <= samples_launched + 64'd1;
                    content_seen <= 1'b1;
                end
                if (sequence_error || pcm_contract_error)
                    completion_invalid <= 1'b1;
            end
        end
    end
    wire completion_qualified = parser_pcm_end && wav_active && pixel_rst_n &&
                                !completion_invalid && !sequence_error &&
                                !pcm_contract_error;
    wire pcm_sample_consume = completion_qualified && wav_pcm_drained &&
                              content_seen &&
                              (samples_sent >= last_content_ordinal);
    // Stop acknowledgement covers content already accepted downstream. The
    // remaining source FIFO/queue may be discarded at the session boundary.
    assign audio_session_tail_drained = pixel_rst_n && !wav_selected &&
        !content_accept && (!content_seen || samples_sent >= last_content_ordinal);
    wav_done_detector u_wav_done_detector (
        .clk(pixel_clk), .rst_n(audio_session_rst_n),
        .parser_pcm_end(completion_qualified), .fifo_empty(fifo_empty),
        .pcm_sample_consume(pcm_sample_consume), .wav_done(wav_done)
    );

    assign pcm_tap_valid = sample_valid && sample_ready;
    assign pcm_tap_left = sample_left;
endmodule
