`timescale 1ns/1ps

// Converts one FAT32 scanner resource entry into the metadata consumed by a
// future audio stream reader. This module does not issue SD transactions and
// does not parse WAV/PCM payload bytes.
module audio_file_manager #(
    parameter integer SECTOR_BYTES = 512
) (
    input  wire        clk,
    input  wire        rst_n,

    // FAT32 scan result. scan_done is a one-cycle pulse from the scanner.
    input  wire        scan_done,
    input  wire [7:0]  scan_error,
    input  wire [31:0] data_start_lba,
    input  wire [7:0]  sectors_per_cluster,

    // Resource table published by fat32_volume_scanner. The selected slot is
    // identified by audio_resource_index at scan_done.
    input  wire [3:0]  resource_valid,
    input  wire [1:0]  audio_resource_index,
    input  wire [31:0] resource0_cluster,
    input  wire [31:0] resource1_cluster,
    input  wire [31:0] resource2_cluster,
    input  wire [31:0] resource3_cluster,
    input  wire [31:0] resource0_size,
    input  wire [31:0] resource1_size,
    input  wire [31:0] resource2_size,
    input  wire [31:0] resource3_size,

    // Latched metadata for the future wav_stream_player.
    output reg         audio_valid,
    output reg [31:0]  start_lba,
    output reg [31:0]  total_sectors,
    output reg [31:0]  file_size,
    output reg [31:0]  first_cluster,
    output reg         requires_fat_chain,
    output reg [7:0]   manager_error,
    output reg         result_pulse
);
    localparam [7:0] ERR_NONE             = 8'h00;
    localparam [7:0] ERR_RESOURCE_MISSING = 8'h40;
    localparam [7:0] ERR_CLUSTER_RANGE    = 8'h41;
    localparam [7:0] ERR_GEOMETRY         = 8'h42;
    localparam [7:0] ERR_LBA_OVERFLOW      = 8'h43;
    localparam [7:0] ERR_SIZE_ZERO         = 8'h44;
    localparam [7:0] ERR_SECTOR_BYTES     = 8'h45;

    reg [31:0] selected_cluster;
    reg [31:0] selected_size;
    reg        selected_valid;
    reg [63:0] cluster_delta_calc;
    reg [63:0] start_lba_calc;
    reg [63:0] total_sectors_calc;

    // The scanner currently exposes four resource slots. Keep the selection
    // as a port so an audio-aware scanner can map its file into any slot.
    always @* begin
        selected_cluster = 32'd0;
        selected_size = 32'd0;
        selected_valid = 1'b0;
        case (audio_resource_index)
            2'd0: begin
                selected_cluster = resource0_cluster;
                selected_size = resource0_size;
                selected_valid = resource_valid[0];
            end
            2'd1: begin
                selected_cluster = resource1_cluster;
                selected_size = resource1_size;
                selected_valid = resource_valid[1];
            end
            2'd2: begin
                selected_cluster = resource2_cluster;
                selected_size = resource2_size;
                selected_valid = resource_valid[2];
            end
            default: begin
                selected_cluster = resource3_cluster;
                selected_size = resource3_size;
                selected_valid = resource_valid[3];
            end
        endcase

        cluster_delta_calc = 64'd0;
        if (selected_cluster >= 32'd2 && sectors_per_cluster != 8'd0)
            cluster_delta_calc = (selected_cluster - 32'd2) * sectors_per_cluster;

        start_lba_calc = {32'd0, data_start_lba} + cluster_delta_calc;
        // SECTOR_BYTES is required to be 512 by the capture-time contract.
        // Keep this as an explicit shift so the common path stays cheap in
        // synthesis and does not infer a general divider.
        total_sectors_calc = ({32'd0, selected_size} + 64'd511) >> 9;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            audio_valid       <= 1'b0;
            start_lba         <= 32'd0;
            total_sectors     <= 32'd0;
            file_size         <= 32'd0;
            first_cluster     <= 32'd0;
            requires_fat_chain<= 1'b0;
            manager_error     <= ERR_NONE;
            result_pulse      <= 1'b0;
        end else begin
            result_pulse <= 1'b0;

            if (scan_done) begin
                result_pulse       <= 1'b1;
                audio_valid        <= 1'b0;
                start_lba          <= 32'd0;
                total_sectors      <= 32'd0;
                file_size          <= 32'd0;
                first_cluster      <= 32'd0;
                requires_fat_chain <= 1'b0;

                if (scan_error != ERR_NONE) begin
                    manager_error <= scan_error;
                end else if (SECTOR_BYTES != 512) begin
                    manager_error <= ERR_SECTOR_BYTES;
                end else if (sectors_per_cluster == 8'd0) begin
                    manager_error <= ERR_GEOMETRY;
                end else if (!selected_valid) begin
                    manager_error <= ERR_RESOURCE_MISSING;
                end else if (selected_cluster < 32'd2) begin
                    manager_error <= ERR_CLUSTER_RANGE;
                end else if (selected_size == 32'd0) begin
                    manager_error <= ERR_SIZE_ZERO;
                end else if (start_lba_calc[63:32] != 32'd0 ||
                             total_sectors_calc[63:32] != 32'd0 ||
                             total_sectors_calc == 64'd0) begin
                    manager_error <= ERR_LBA_OVERFLOW;
                end else begin
                    manager_error      <= ERR_NONE;
                    audio_valid       <= 1'b1;
                    start_lba         <= start_lba_calc[31:0];
                    total_sectors     <= total_sectors_calc[31:0];
                    file_size         <= selected_size;
                    first_cluster     <= selected_cluster;
                    // FAT32 files may be fragmented. The metadata gives the
                    // first data sector and logical byte extent; a future
                    // player must follow the FAT chain when this is asserted.
                    requires_fat_chain <= 1'b1;
                end
            end
        end
    end
endmodule
