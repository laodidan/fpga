`timescale 1ns/1ps

// Four committed WAV resources. The caller owns wav_index; this module
// never advances the selection or reacts to playback completion.
module audio_file_manager (
    input wire clk, rst_n,
    input wire scan_done,
    input wire [7:0] scan_error,
    input wire [3:0] scanned_resource_valid,
    input wire [31:0] resource0_cluster, resource1_cluster,
    input wire [31:0] resource2_cluster, resource3_cluster,
    input wire [31:0] resource0_size, resource1_size,
    input wire [31:0] resource2_size, resource3_size,
    input wire [1:0] wav_index,
    output reg [31:0] start_cluster,
    output reg [31:0] file_size,
    output reg resource_valid
);
    reg [3:0] valid_table;
    reg [31:0] cluster_table [0:3];
    reg [31:0] size_table [0:3];
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_table <= 4'd0;
            for (i = 0; i < 4; i = i + 1) begin
                cluster_table[i] <= 32'd0;
                size_table[i] <= 32'd0;
            end
        end else if (scan_done) begin
            // Missing files do not invalidate other usable entries.
            valid_table <= (scan_error == 8'h00 || scan_error == 8'h22) ?
                           scanned_resource_valid : 4'd0;
            cluster_table[0] <= resource0_cluster;
            cluster_table[1] <= resource1_cluster;
            cluster_table[2] <= resource2_cluster;
            cluster_table[3] <= resource3_cluster;
            size_table[0] <= resource0_size;
            size_table[1] <= resource1_size;
            size_table[2] <= resource2_size;
            size_table[3] <= resource3_size;
        end
    end
    always @* begin
        start_cluster = 32'd0;
        file_size = 32'd0;
        resource_valid = 1'b0;
        case (wav_index)
            2'd0, 2'd1, 2'd2, 2'd3: begin
                resource_valid = valid_table[wav_index];
                if (valid_table[wav_index]) begin
                    start_cluster = cluster_table[wav_index];
                    file_size = size_table[wav_index];
                end
            end
            default: begin end
        endcase
    end
endmodule
