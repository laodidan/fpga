`timescale 1ns/1ps

module audio_file_manager_tb;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    reg scan_done;
    reg [7:0] scan_error;
    reg [31:0] data_start_lba;
    reg [7:0] sectors_per_cluster;
    reg [3:0] resource_valid;
    reg [1:0] audio_resource_index;
    reg [31:0] resource0_cluster, resource1_cluster;
    reg [31:0] resource2_cluster, resource3_cluster;
    reg [31:0] resource0_size, resource1_size;
    reg [31:0] resource2_size, resource3_size;
    wire audio_valid;
    wire [31:0] start_lba, total_sectors, file_size, first_cluster;
    wire requires_fat_chain;
    wire [7:0] manager_error;
    wire result_pulse;

    audio_file_manager dut (
        .clk(clk), .rst_n(rst_n), .scan_done(scan_done),
        .scan_error(scan_error), .data_start_lba(data_start_lba),
        .sectors_per_cluster(sectors_per_cluster),
        .resource_valid(resource_valid),
        .audio_resource_index(audio_resource_index),
        .resource0_cluster(resource0_cluster),
        .resource1_cluster(resource1_cluster),
        .resource2_cluster(resource2_cluster),
        .resource3_cluster(resource3_cluster),
        .resource0_size(resource0_size), .resource1_size(resource1_size),
        .resource2_size(resource2_size), .resource3_size(resource3_size),
        .audio_valid(audio_valid), .start_lba(start_lba),
        .total_sectors(total_sectors), .file_size(file_size),
        .first_cluster(first_cluster),
        .requires_fat_chain(requires_fat_chain),
        .manager_error(manager_error), .result_pulse(result_pulse)
    );

    task scan_once;
        begin
            @(negedge clk);
            scan_done = 1'b1;
            @(negedge clk);
            scan_done = 1'b0;
        end
    endtask

    initial begin
        scan_done = 1'b0;
        scan_error = 8'h00;
        data_start_lba = 32'd100;
        sectors_per_cluster = 8'd8;
        resource_valid = 4'b0100;
        audio_resource_index = 2'd2;
        resource0_cluster = 32'd0;
        resource1_cluster = 32'd0;
        resource2_cluster = 32'd5;
        resource3_cluster = 32'd0;
        resource0_size = 32'd0;
        resource1_size = 32'd0;
        resource2_size = 32'd1025;
        resource3_size = 32'd0;

        repeat (2) @(negedge clk);
        rst_n = 1'b1;
        scan_once();
        #1;
        if (!audio_valid || !result_pulse || start_lba != 32'd124 ||
            total_sectors != 32'd3 || file_size != 32'd1025 ||
            first_cluster != 32'd5 || !requires_fat_chain ||
            manager_error != 8'h00)
            $fatal(1, "valid metadata conversion failed");

        resource_valid = 4'b0000;
        scan_once();
        #1;
        if (audio_valid || manager_error != 8'h40)
            $fatal(1, "missing resource was not rejected");

        resource_valid = 4'b0100;
        scan_error = 8'h2a;
        scan_once();
        #1;
        if (audio_valid || manager_error != 8'h2a)
            $fatal(1, "scanner error was not propagated");

        scan_error = 8'h00;
        resource2_cluster = 32'd1;
        scan_once();
        #1;
        if (audio_valid || manager_error != 8'h41)
            $fatal(1, "invalid cluster was not rejected");

        resource2_cluster = 32'd5;
        sectors_per_cluster = 8'd0;
        scan_once();
        #1;
        if (audio_valid || manager_error != 8'h42)
            $fatal(1, "invalid geometry was not rejected");

        sectors_per_cluster = 8'd8;
        data_start_lba = 32'hfffffff0;
        scan_once();
        #1;
        if (audio_valid || manager_error != 8'h43)
            $fatal(1, "LBA overflow was not rejected");

        $display("audio_file_manager_tb PASS");
        $finish;
    end
endmodule
