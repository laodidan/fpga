`timescale 1ns/1ps

module wav_stream_player_tb;
    reg mem_clk = 1'b0;
    reg pixel_clk = 1'b0;
    always #5 mem_clk = ~mem_clk;
    always #7 pixel_clk = ~pixel_clk;

    reg mem_rst_n = 1'b0;
    reg pixel_rst_n = 1'b0;
    reg play_enable = 1'b0;
    reg file_valid = 1'b0;
    reg [31:0] start_lba = 32'd100;
    reg [31:0] total_sectors = 32'd3;
    wire audio_req;
    wire [31:0] audio_lba;
    reg audio_start = 1'b0;
    reg [7:0] audio_data = 8'd0;
    reg audio_valid = 1'b0;
    reg audio_done = 1'b0;
    reg sample_tick = 1'b0;
    wire signed [15:0] pcm_sample;
    wire signed [15:0] sample_out;
    wire sample_valid;
    wire stream_active;
    wire buffer_low;
    wire stream_end;
    integer i;
    integer watchdog = 0;

    wav_stream_player #(.ADDR_WIDTH(9)) dut (
        .mem_clk(mem_clk), .mem_rst_n(mem_rst_n),
        .play_enable(play_enable), .file_valid(file_valid),
        .start_lba(start_lba), .total_sectors(total_sectors),
        .audio_req(audio_req), .audio_lba(audio_lba),
        .audio_start(audio_start), .audio_data(audio_data),
        .audio_valid(audio_valid), .audio_done(audio_done),
        .pixel_clk(pixel_clk), .pixel_rst_n(pixel_rst_n),
        .sample_tick(sample_tick), .pcm_sample(pcm_sample),
        .sample_valid(sample_valid), .stream_active(stream_active),
        .buffer_low(buffer_low), .sample_out(sample_out),
        .stream_end(stream_end)
    );

    always @(posedge mem_clk) begin
        watchdog = watchdog + 1;
        if (watchdog > 20000) begin
            $display("FAIL: timeout");
            $finish;
        end
    end

    task check_sample;
        input [15:0] expected;
        begin
            @(negedge pixel_clk);
            sample_tick = 1'b1;
            @(negedge pixel_clk);
            if (sample_valid !== 1'b1 || pcm_sample !== expected ||
                sample_out !== expected) begin
                $display("FAIL: sample expected=%h actual=%h valid=%b at %0t",
                         expected, pcm_sample, sample_valid, $time);
                $finish;
            end
            sample_tick = 1'b0;
            repeat (3) @(negedge pixel_clk);
        end
    endtask

    task send_sector;
        input integer sector_index;
        integer byte_index;
        reg [15:0] value;
        begin
            wait (audio_req === 1'b1);
            if (audio_lba !== start_lba + sector_index) begin
                $display("FAIL: LBA expected=%0d actual=%0d",
                         start_lba + sector_index, audio_lba);
                $finish;
            end
            repeat (5) begin
                @(negedge mem_clk);
                if (audio_req !== 1'b1 ||
                    audio_lba !== start_lba + sector_index) begin
                    $display("FAIL: SD request did not hold until grant");
                    $finish;
                end
            end
            audio_start = 1'b1;
            @(negedge mem_clk);
            audio_start = 1'b0;
            if (audio_req !== 1'b0) begin
                $display("FAIL: SD request did not clear after grant");
                $finish;
            end
            for (byte_index = 0; byte_index < 512;
                 byte_index = byte_index + 1) begin
                value = 16'h8001 + sector_index * 256 + (byte_index / 2);
                audio_data = (byte_index % 2) ? value[15:8] : value[7:0];
                audio_valid = 1'b1;
                @(negedge mem_clk);
            end
            audio_valid = 1'b0;
            audio_done = 1'b1;
            @(negedge mem_clk);
            audio_done = 1'b0;
        end
    endtask

    initial begin
        repeat (5) @(negedge mem_clk);
        mem_rst_n = 1'b1;
        pixel_rst_n = 1'b1;
        repeat (8) @(negedge pixel_clk);
        if (audio_req !== 1'b0 || stream_active !== 1'b0) begin
            $display("FAIL: reset/idle state");
            $finish;
        end

        play_enable = 1'b1;
        repeat (8) @(negedge pixel_clk);
        check_sample(16'h0000);
        if (audio_req !== 1'b0) begin
            $display("FAIL: requested without file_valid");
            $finish;
        end
        file_valid = 1'b1;

        send_sector(0);
        send_sector(1);
        repeat (10) @(negedge mem_clk);
        if (audio_req !== 1'b0) begin
            $display("FAIL: requested third sector with full FIFO");
            $finish;
        end
        repeat (8) @(negedge pixel_clk);
        if (stream_active !== 1'b1 || buffer_low !== 1'b0) begin
            $display("FAIL: active/full status");
            $finish;
        end

        for (i = 0; i < 256; i = i + 1)
            check_sample(16'h8001 + i);
        // The synchronized read pointer needs two mem_clk edges before the
        // next whole sector can be reserved.
        send_sector(2);
        repeat (8) @(negedge pixel_clk);
        for (i = 256; i < 768; i = i + 1)
            check_sample(16'h8001 + i);
        repeat (8) @(negedge pixel_clk);
        if (stream_active !== 1'b0 || stream_end !== 1'b1) begin
            $display("FAIL: tail did not drain before stream_end");
            $finish;
        end
        check_sample(16'h0000);
        if (sample_valid !== 1'b0) begin
            $display("FAIL: sample_valid did not pulse");
            $finish;
        end

        // A new play interval must clear both pointers and start from the
        // first sample instead of exposing data left from the prior stream.
        play_enable = 1'b0;
        repeat (8) @(negedge pixel_clk);
        repeat (8) @(negedge mem_clk);
        if (stream_active !== 1'b0 || stream_end !== 1'b0) begin
            $display("FAIL: stop did not clear stream status");
            $finish;
        end
        start_lba = 32'd200;
        total_sectors = 32'd1;
        play_enable = 1'b1;
        send_sector(0);
        repeat (8) @(negedge pixel_clk);
        check_sample(16'h8001);
        play_enable = 1'b0;
        $display("wav_stream_player_tb PASS");
        $finish;
    end
endmodule
