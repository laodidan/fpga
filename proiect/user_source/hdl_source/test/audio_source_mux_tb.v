`timescale 1ns/1ps

module audio_source_mux_tb;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    reg wav_valid;
    wire wav_ready;
    reg signed [15:0] wav_left;
    reg signed [15:0] wav_right;
    reg wav_active;

    reg arp_valid;
    wire arp_ready;
    reg signed [15:0] arp_left;
    reg signed [15:0] arp_right;

    reg force_arpeggio;
    reg out_ready;
    wire out_valid;
    wire signed [15:0] out_left;
    wire signed [15:0] out_right;
    wire sample_valid;
    wire signed [15:0] pcm_sample;

    audio_source_mux dut (
        .clk(clk), .rst_n(rst_n),
        .wav_valid(wav_valid), .wav_ready(wav_ready),
        .wav_left(wav_left), .wav_right(wav_right),
        .wav_active(wav_active),
        .arp_valid(arp_valid), .arp_ready(arp_ready),
        .arp_left(arp_left), .arp_right(arp_right),
        .force_arpeggio(force_arpeggio),
        .out_valid(out_valid), .out_ready(out_ready),
        .out_left(out_left), .out_right(out_right),
        .sample_valid(sample_valid), .pcm_sample(pcm_sample)
    );

    task check_outputs;
        input expected_valid;
        input signed [15:0] expected_left;
        input signed [15:0] expected_right;
        input expected_wav_ready;
        input expected_arp_ready;
        begin
            #1;
            if (out_valid !== expected_valid || sample_valid !== expected_valid ||
                out_left !== expected_left || pcm_sample !== expected_left ||
                out_right !== expected_right ||
                wav_ready !== expected_wav_ready ||
                arp_ready !== expected_arp_ready) begin
                $display("FAIL: valid=%b left=%h right=%h wav_ready=%b arp_ready=%b",
                         out_valid, out_left, out_right, wav_ready, arp_ready);
                $fatal(1);
            end
        end
    endtask

    initial begin
        wav_valid = 1'b0;
        wav_left = 16'sd0;
        wav_right = 16'sd0;
        wav_active = 1'b0;
        arp_valid = 1'b0;
        arp_left = 16'sd0;
        arp_right = 16'sd0;
        force_arpeggio = 1'b0;
        out_ready = 1'b1;

        repeat (2) @(negedge clk);
        rst_n = 1'b1;

        // WAV has priority whenever its stream is active.
        wav_active = 1'b1;
        wav_valid = 1'b1;
        wav_left = 16'sh1234;
        wav_right = -16'sh2345;
        arp_valid = 1'b1;
        arp_left = 16'sh4567;
        arp_right = 16'sh5678;
        check_outputs(1'b1, 16'sh1234, -16'sh2345, 1'b1, 1'b0);

        // The inactive arpeggio source is not consumed while WAV is selected.
        out_ready = 1'b0;
        check_outputs(1'b1, 16'sh1234, -16'sh2345, 1'b0, 1'b0);

        // End of WAV playback immediately returns ownership to the arpeggio.
        out_ready = 1'b1;
        wav_active = 1'b0;
        check_outputs(1'b1, 16'sh4567, 16'sh5678, 1'b0, 1'b1);

        // With no valid source the mux emits silence and no valid transfer.
        arp_valid = 1'b0;
        check_outputs(1'b0, 16'sd0, 16'sd0, 1'b0, 1'b1);

        // The explicit override can force the fallback source while WAV is
        // still active.
        wav_active = 1'b1;
        wav_left = 16'sh1111;
        wav_right = 16'sh2222;
        arp_valid = 1'b1;
        arp_left = 16'sh3333;
        arp_right = 16'sh4444;
        force_arpeggio = 1'b1;
        check_outputs(1'b1, 16'sh3333, 16'sh4444, 1'b0, 1'b1);

        $display("audio_source_mux_tb PASS");
        $finish;
    end
endmodule
