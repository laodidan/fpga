`timescale 1ns/1ps

// Serializes the independent WAV scanner and file streamer onto the
// sd_arbiter audio sector port.  A request is held until audio_start grants it.
module audio_sector_client (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        scan_sector_req,
    input  wire [31:0] scan_sector_lba,
    output wire        scan_sector_busy,
    output wire        scan_sector_valid,
    output wire [7:0]  scan_sector_byte,
    output wire [8:0]  scan_sector_byte_index,
    output wire        scan_sector_done,
    output wire [7:0]  scan_sector_error,
    input  wire        file_sector_req,
    input  wire [31:0] file_sector_lba,
    output wire        file_sector_busy,
    output wire        file_sector_valid,
    output wire [7:0]  file_sector_byte,
    output wire [8:0]  file_sector_byte_index,
    output wire        file_sector_done,
    output wire [7:0]  file_sector_error,
    output wire        audio_req,
    output wire [31:0] audio_lba,
    input  wire        audio_start,
    input  wire [7:0]  audio_data,
    input  wire        audio_valid,
    input  wire        audio_done
);
    reg active;
    reg owner_file;
    reg request_pending;
    reg [31:0] request_lba;
    reg [8:0] byte_index;
    reg scan_pending, file_pending;
    reg scan_armed, file_armed;
    reg [31:0] scan_lba_q, file_lba_q;

    wire choose_file = !request_pending && !active && !scan_pending && file_pending;
    wire choose_scan = !request_pending && !active && scan_pending;
    assign audio_req = request_pending;
    assign audio_lba = request_lba;
    assign scan_sector_busy = scan_pending || ((request_pending || active) && !owner_file);
    assign file_sector_busy = file_pending || ((request_pending || active) && owner_file);
    assign scan_sector_valid = active && !owner_file && audio_valid;
    assign file_sector_valid = active && owner_file && audio_valid;
    assign scan_sector_byte = audio_data;
    assign file_sector_byte = audio_data;
    assign scan_sector_byte_index = byte_index;
    assign file_sector_byte_index = byte_index;
    assign scan_sector_done = active && !owner_file && audio_done;
    assign file_sector_done = active && owner_file && audio_done;
    assign scan_sector_error = 8'h00;
    assign file_sector_error = 8'h00;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active <= 1'b0;
            owner_file <= 1'b0;
            request_pending <= 1'b0;
            request_lba <= 32'd0;
            byte_index <= 9'd0;
            scan_pending <= 1'b0;
            file_pending <= 1'b0;
            scan_armed <= 1'b1;
            file_armed <= 1'b1;
            scan_lba_q <= 32'd0;
            file_lba_q <= 32'd0;
        end else begin
            if (!scan_sector_req) scan_armed <= 1'b1;
            if (!file_sector_req) file_armed <= 1'b1;
            if (scan_sector_req && scan_armed && !scan_sector_busy) begin
                scan_pending <= 1'b1;
                scan_lba_q <= scan_sector_lba;
                scan_armed <= 1'b0;
            end
            if (file_sector_req && file_armed && !file_sector_busy) begin
                file_pending <= 1'b1;
                file_lba_q <= file_sector_lba;
                file_armed <= 1'b0;
            end
            if (choose_scan) begin
                scan_pending <= 1'b0;
                request_pending <= 1'b1;
                owner_file <= 1'b0;
                request_lba <= scan_lba_q;
            end else if (choose_file) begin
                file_pending <= 1'b0;
                request_pending <= 1'b1;
                owner_file <= 1'b1;
                request_lba <= file_lba_q;
            end
            if (request_pending && audio_start) begin
                request_pending <= 1'b0;
                active <= 1'b1;
                byte_index <= 9'd0;
            end
            if (active && audio_valid && byte_index != 9'd511)
                byte_index <= byte_index + 9'd1;
            if (active && audio_done) begin
                active <= 1'b0;
                byte_index <= 9'd0;
            end
        end
    end
endmodule
