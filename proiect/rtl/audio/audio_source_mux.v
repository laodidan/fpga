`timescale 1ns/1ps

// Selects the PCM source presented to the HDMI audio contract.  A WAV stream
// owns the output while it is active; the media arpeggio is the idle source.
// Ready is routed only to the selected producer so an inactive source cannot
// consume or advance a sample while it is waiting in the background.
module audio_source_mux (
    input  wire               clk,
    input  wire               rst_n,

    // WAV stream source.
    input  wire               wav_valid,
    output wire               wav_ready,
    input  wire signed [15:0] wav_left,
    input  wire signed [15:0] wav_right,
    input  wire               wav_active,

    // Existing pcm_media_tone source.
    input  wire               arp_valid,
    output wire               arp_ready,
    input  wire signed [15:0] arp_left,
    input  wire signed [15:0] arp_right,

    // Optional override used by a debug/menu control.
    input  wire               force_arpeggio,

    // Downstream PCM consumer.
    output wire               out_valid,
    input  wire               out_ready,
    output wire signed [15:0] out_left,
    output wire signed [15:0] out_right,

    // Stage-four observation names.  These are aliases of the left channel
    // and valid output, preserving the existing stereo contract above.
    output wire               sample_valid,
    output wire signed [15:0] pcm_sample
);
    // clk and rst_n are part of the source-domain contract.  Selection is
    // intentionally combinational, so switching has no added cycle latency.
    wire use_wav = wav_active && !force_arpeggio;
    wire selected_valid = use_wav ? wav_valid : arp_valid;
    wire signed [15:0] selected_left = use_wav ? wav_left : arp_left;
    wire signed [15:0] selected_right = use_wav ? wav_right : arp_right;

    assign out_valid = selected_valid;
    assign out_left  = selected_valid ? selected_left : 16'sd0;
    assign out_right = selected_valid ? selected_right : 16'sd0;

    assign wav_ready = use_wav ? out_ready : 1'b0;
    assign arp_ready = use_wav ? 1'b0 : out_ready;

    assign sample_valid = out_valid;
    assign pcm_sample   = out_left;

endmodule
