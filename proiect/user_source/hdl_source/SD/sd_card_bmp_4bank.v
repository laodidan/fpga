`timescale 1ns/1ps

// Four fixed image slots. Scan FAT32 once, preload each available BMP into
// its corresponding SDRAM bank, then leave the card idle during playback.
// This module is an SD client; the system-level sd_arbiter owns the card.
module sd_card_bmp_4bank #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter [31:0] SCAN_START_SECTOR = 32'd0,
    parameter [31:0] SCAN_MAX_SECTOR = 32'd131071,
    parameter [2:0] SCAN_TARGET_COUNT = 3'd4,
    parameter integer LONG_PRESS_MS = 1000,
    parameter integer SD_READ_TIMEOUT_MS = 500
)(
    input wire clk,
    input wire rst,
    input wire sdram_init_done,
    input wire key_next,
    input wire key_auto,
    output reg key_next_event,
    output reg key_prev_event,
    output reg key_auto_event,
    output reg [3:0] state_code,
    input wire [15:0] bmp_width,
    input wire [15:0] bmp_height,
    output reg event_valid,
    output reg [1:0] event_type,
    output reg [31:0] event_data,
    input wire event_full,
    input wire loader_result_toggle,
    input wire [7:0] loader_result_error,
    input wire sd_init_done,
    output wire bmp_req,
    output wire [31:0] bmp_lba,
    input wire bmp_busy,
    input wire [7:0] bmp_data,
    input wire bmp_valid,
    input wire bmp_done,
    input wire [7:0] bmp_error
);
wire key_next_press;
wire key_auto_stable;

wire             fat_scan_start;
wire             fat_scan_busy;
wire             fat_scan_done;
wire [7:0]       fat_scan_error;
wire [31:0]      fat_partition_lba;
wire [7:0]       fat_sectors_per_cluster;
wire [31:0]      fat_start_lba;
wire [31:0]      fat_data_start_lba;
wire [31:0]      fat_max_cluster;
wire [3:0]       fat_resource_valid;
wire [31:0]      fat_resource0_cluster, fat_resource1_cluster;
wire [31:0]      fat_resource2_cluster, fat_resource3_cluster;
wire [31:0]      fat_resource0_size, fat_resource1_size;
wire [31:0]      fat_resource2_size, fat_resource3_size;
wire             fat_scan_sector_req;
wire [31:0]      fat_scan_sector_lba;
wire             fat_scan_sector_busy;
wire             fat_scan_sector_valid;
wire [7:0]       fat_scan_sector_byte;
wire [8:0]       fat_scan_sector_byte_index;
wire             fat_scan_sector_done;
wire [7:0]       fat_scan_sector_error;

wire             fat_file_start;
wire             fat_file_busy;
wire             fat_file_done;
wire             fat_file_valid;
wire [7:0]       fat_file_byte;
wire [31:0]      fat_file_byte_offset;
wire [7:0]       fat_file_error;
wire             fat_file_sector_req;
wire [31:0]      fat_file_sector_lba;
wire             fat_file_sector_busy;
wire             fat_file_sector_valid;
wire [7:0]       fat_file_sector_byte;
wire [8:0]       fat_file_sector_byte_index;
wire             fat_file_sector_done;
wire [7:0]       fat_file_sector_error;

wire             fat_sector_req;
wire [31:0]      fat_sector_lba;
wire             fat_sector_busy;
wire             fat_adapter_busy;
wire             fat_sector_valid;
wire [7:0]       fat_sector_byte;
wire [8:0]       fat_sector_byte_index;
wire             fat_sector_done;
wire [7:0]       fat_sector_error;
wire             fat_sd_sec_read;
wire [31:0]      fat_sd_sec_read_addr;
wire             scan_sector_grant;
wire             file_sector_grant;
reg              sector_owner_file;
reg              sector_owned;


reg scan_start_pulse, file_start_pulse;
reg [31:0] load_cluster, load_size;
reg scan_kicked, scan_complete, scan_abort, load_abort;
reg [31:0] scan_timeout_cnt, load_timeout_cnt, recovery_wait;
reg [4:0] sd_reset_count;
reg [3:0] attempted_mask, failed_mask;
reg [1:0] load_idx;
reg retry_count, aborted;
reg [2:0] result_sync;
reg [7:0] result_error_sync1, result_error_sync2;
reg result_pending;
reg [7:0] result_error_pending;
wire result_pulse = result_sync[2] ^ result_sync[1];
wire driver_reset = rst || sd_reset_count != 0;
wire file_reset = rst || load_abort;
localparam [63:0] RECOVERY_CYCLES_RAW =
    (64'd1*CLK_FREQ_HZ*SD_READ_TIMEOUT_MS)/1000;
localparam [63:0] RECOVERY_CYCLES = RECOVERY_CYCLES_RAW < 1 ? 1 : RECOVERY_CYCLES_RAW;
localparam [63:0] FRAME_LIMIT = 64'd5*CLK_FREQ_HZ;
localparam [3:0] S_SCAN=0, S_PICK=1, S_START=2, S_STREAM=3,
                 S_DRAIN=4, S_ABORT=5, S_RESULT=6, S_RECOVER=7, S_READY=8;
reg [3:0] phase;
reg [7:0] header_0, header_1;
reg [31:0] file_len, pixel_offset, dib_size, width, height, compression;
reg [15:0] planes, bit_count;
reg header_checked, parser_error;
reg [31:0] pixel_count;
reg [1:0] bmp_byte_idx;
reg [7:0] blue_byte, green_byte;
reg [23:0] bmp_pixel_data;
reg bmp_data_wr_en;
reg [31:0] long_press_cnt;
reg long_press_marked;
wire [63:0] expected_pixels = 64'd1 * bmp_width * bmp_height;
wire [63:0] pixel_end = {32'd0, pixel_offset} + expected_pixels * 3;
wire header_match = header_0 == 8'h42 && header_1 == 8'h4d &&
    dib_size >= 40 && planes == 1 && width == {16'd0, bmp_width} &&
    height == {16'd0, bmp_height} && bmp_width != 0 && bmp_height != 0 &&
    bmp_width[1:0] == 0 && bit_count == 24 && compression == 0 &&
    {32'd0,pixel_offset} >= 64'd14 + {32'd0,dib_size} &&
    pixel_offset >= 54 && pixel_offset < file_len &&
    file_len <= load_size && {32'd0,file_len} >= pixel_end &&
    pixel_end <= {32'd0,load_size};

key_press_debounce #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .DEBOUNCE_MS(50)) u_next (
    .clk(clk), .rst(rst), .button_in(key_next),
    .press_pulse(key_next_press), .button_stable_out()
);
key_press_debounce #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .DEBOUNCE_MS(50)) u_auto (
    .clk(clk), .rst(rst), .button_in(key_auto),
    .press_pulse(), .button_stable_out(key_auto_stable)
);

function [31:0] cluster_lut;
    input [1:0] index;
    begin
        case(index)
            0: cluster_lut=fat_resource0_cluster;
            1: cluster_lut=fat_resource1_cluster;
            2: cluster_lut=fat_resource2_cluster;
            default: cluster_lut=fat_resource3_cluster;
        endcase
    end
endfunction
function [31:0] size_lut;
    input [1:0] index;
    begin
        case(index)
            0: size_lut=fat_resource0_size;
            1: size_lut=fat_resource1_size;
            2: size_lut=fat_resource2_size;
            default: size_lut=fat_resource3_size;
        endcase
    end
endfunction
function [1:0] first_pending;
    input [3:0] available;
    begin
        if (available[0]) first_pending=0;
        else if (available[1]) first_pending=1;
        else if (available[2]) first_pending=2;
        else first_pending=3;
    end
endfunction

task reset_parser;
    begin
        header_0<=0; header_1<=0; file_len<=0; pixel_offset<=0;
        dib_size<=0; width<=0; height<=0; compression<=0;
        planes<=0; bit_count<=0; header_checked<=0; parser_error<=0;
        pixel_count<=0; bmp_byte_idx<=0; bmp_data_wr_en<=0;
        aborted<=0; load_timeout_cnt<=0;
    end
endtask

always @(posedge clk or posedge rst) begin
    if (rst) begin
        scan_start_pulse<=0; file_start_pulse<=0;
        scan_kicked<=0; scan_complete<=0; scan_abort<=0;
        scan_timeout_cnt<=0; load_timeout_cnt<=0; recovery_wait<=0;
        sd_reset_count<=0; load_abort<=0; load_cluster<=0; load_size<=0;
        attempted_mask<=0; failed_mask<=0; load_idx<=0; retry_count<=0;
        result_sync<=0; result_error_sync1<=0; result_error_sync2<=0;
        result_pending<=0; result_error_pending<=0;
        phase<=S_SCAN; state_code<=4'd1; event_valid<=0;
        event_type<=0; event_data<=0;
        key_next_event<=0; key_prev_event<=0; key_auto_event<=0;
        long_press_cnt<=0; long_press_marked<=0;
        header_0<=0; header_1<=0; file_len<=0; pixel_offset<=0;
        dib_size<=0; width<=0; height<=0; compression<=0;
        planes<=0; bit_count<=0; header_checked<=0; parser_error<=0;
        pixel_count<=0; bmp_byte_idx<=0; blue_byte<=0; green_byte<=0;
        bmp_pixel_data<=0; bmp_data_wr_en<=0; aborted<=0;
    end else begin
        result_sync <= {result_sync[1:0],loader_result_toggle};
        result_error_sync1<=loader_result_error;
        result_error_sync2<=result_error_sync1;
        // A loader failure can finish before the file stream or ABORT event.
        // Keep its first completion until this transaction reaches S_RESULT.
        if((phase==S_STREAM || phase==S_DRAIN || phase==S_ABORT ||
            phase==S_RESULT) && result_pulse && !result_pending) begin
            result_pending<=1;
            result_error_pending<=result_error_sync2;
        end
        scan_start_pulse<=0; file_start_pulse<=0; load_abort<=0;
        event_valid<=0; bmp_data_wr_en<=0;
        key_next_event<=key_next_press;
        key_prev_event<=0; key_auto_event<=0;
        if(sd_reset_count!=0) sd_reset_count<=sd_reset_count-1'b1;
        if(key_auto_stable==0) begin
            if(long_press_cnt<((CLK_FREQ_HZ/1000)*LONG_PRESS_MS-1))
                long_press_cnt<=long_press_cnt+1'b1;
            else if(!long_press_marked) begin
                long_press_marked<=1; key_prev_event<=1;
            end
        end else if(long_press_cnt!=0) begin
            if(!long_press_marked) key_auto_event<=1;
            long_press_cnt<=0; long_press_marked<=0;
        end
        if(scan_kicked && !scan_complete && !scan_abort && fat_scan_busy) begin
            if(fat_scan_sector_done) scan_timeout_cnt<=0;
            else scan_timeout_cnt<=scan_timeout_cnt+1'b1;
            if(scan_timeout_cnt>=RECOVERY_CYCLES-1) begin
                scan_abort<=1; sd_reset_count<=16; state_code<=4'ha;
            end
        end
        if(phase==S_STREAM || phase==S_DRAIN || phase==S_RESULT)
            load_timeout_cnt<=load_timeout_cnt+1'b1;
        // Queue the RGB pixel registered by the byte parser on the prior edge.
        if(bmp_data_wr_en && (phase==S_STREAM || phase==S_DRAIN)) begin
            if(!event_full) begin
                event_valid<=1; event_type<=2'b01;
                event_data<={8'd0,bmp_pixel_data};
            end else parser_error<=1;
        end
        case (phase)
            S_SCAN: begin
                if(!scan_kicked && sd_init_done && sdram_init_done &&
                   !fat_sector_busy && sd_reset_count==0) begin
                    scan_start_pulse<=1; scan_kicked<=1;
                end
                if(fat_scan_done && !scan_abort) begin
                    if(fat_scan_error==0 && fat_resource_valid!=0) begin
                        scan_complete<=1; phase<=S_PICK;
                    end else begin
                        scan_abort<=1; state_code<=4'h8;
                    end
                end
            end
            S_PICK: begin
                if(|(fat_resource_valid & ~attempted_mask)) begin
                    load_idx<=first_pending(fat_resource_valid & ~attempted_mask);
                    load_cluster<=cluster_lut(first_pending(fat_resource_valid & ~attempted_mask));
                    load_size<=size_lut(first_pending(fat_resource_valid & ~attempted_mask));
                    retry_count<=0;
                    reset_parser();
                    phase<=S_START;
                end else phase<=S_READY;
            end
            S_START: if(!event_full && sd_init_done && !fat_file_busy &&
                        !fat_sector_busy && sd_reset_count==0) begin
                result_pending<=0; result_error_pending<=0;
                event_valid<=1; event_type<=2'b00;
                event_data<={30'd0,load_idx};
                file_start_pulse<=1;
                phase<=S_STREAM;
            end
            S_STREAM: begin
                if(parser_error || (fat_file_done && fat_file_error!=0) ||
                   load_timeout_cnt>=FRAME_LIMIT-1) begin
                    phase<=S_ABORT; aborted<=1; load_abort<=1;
                    sd_reset_count<=16; state_code<=4'h9;
                    bmp_data_wr_en<=0;
                end else if(fat_file_done) phase<=S_DRAIN;
            end
            S_DRAIN: begin
                if(parser_error || load_timeout_cnt>=FRAME_LIMIT-1 ||
                   (!bmp_data_wr_en && {32'd0,pixel_count}!=expected_pixels)) begin
                    phase<=S_ABORT; aborted<=1; load_abort<=1;
                    sd_reset_count<=16; state_code<=4'h9;
                end else if(!bmp_data_wr_en && !event_full) begin
                    event_valid<=1; event_type<=2'b10; event_data<=0;
                    phase<=S_RESULT;
                end
            end
            S_ABORT: if(!event_full) begin
                event_valid<=1; event_type<=2'b11;
                event_data<=32'h00000001;
                phase<=S_RESULT;
            end
            S_RESULT: if(result_pending || result_pulse) begin
                result_pending<=0; result_error_pending<=0;
                if(aborted ||
                   (result_pending ? result_error_pending : result_error_sync2)!=0) begin
                    state_code<=4'h9;
                    if(!retry_count) begin
                        retry_count<=1;
                        recovery_wait<=0;
                        phase<=S_RECOVER;
                    end else begin
                        failed_mask[load_idx]<=1;
                        attempted_mask[load_idx]<=1;
                        phase<=S_PICK;
                    end
                end else begin
                    attempted_mask[load_idx]<=1;
                    state_code<={2'b00,load_idx}+4'd1;
                    phase<=S_PICK;
                end
            end
            S_RECOVER: begin
                if(recovery_wait>=RECOVERY_CYCLES-1) begin
                    failed_mask[load_idx]<=1;
                    attempted_mask[load_idx]<=1;
                    state_code<=4'ha; phase<=S_PICK;
                end else if(recovery_wait>=32 && !fat_file_busy &&
                            !fat_sector_busy && sd_init_done &&
                            sd_reset_count==0) begin
                    reset_parser();
                    phase<=S_START;
                end else recovery_wait<=recovery_wait+1'b1;
            end
            default: ; // S_READY: all available resources were attempted.
        endcase

        if(phase==S_STREAM && fat_file_valid && !parser_error) begin
            case(fat_file_byte_offset)
                0: header_0<=fat_file_byte;
                1: header_1<=fat_file_byte;
                2: file_len[7:0]<=fat_file_byte;
                3: file_len[15:8]<=fat_file_byte;
                4: file_len[23:16]<=fat_file_byte;
                5: file_len[31:24]<=fat_file_byte;
                10: pixel_offset[7:0]<=fat_file_byte;
                11: pixel_offset[15:8]<=fat_file_byte;
                12: pixel_offset[23:16]<=fat_file_byte;
                13: pixel_offset[31:24]<=fat_file_byte;
                14: dib_size[7:0]<=fat_file_byte;
                15: dib_size[15:8]<=fat_file_byte;
                16: dib_size[23:16]<=fat_file_byte;
                17: dib_size[31:24]<=fat_file_byte;
                18: width[7:0]<=fat_file_byte;
                19: width[15:8]<=fat_file_byte;
                20: width[23:16]<=fat_file_byte;
                21: width[31:24]<=fat_file_byte;
                22: height[7:0]<=fat_file_byte;
                23: height[15:8]<=fat_file_byte;
                24: height[23:16]<=fat_file_byte;
                25: height[31:24]<=fat_file_byte;
                26: planes[7:0]<=fat_file_byte;
                27: planes[15:8]<=fat_file_byte;
                28: bit_count[7:0]<=fat_file_byte;
                29: bit_count[15:8]<=fat_file_byte;
                30: compression[7:0]<=fat_file_byte;
                31: compression[15:8]<=fat_file_byte;
                32: compression[23:16]<=fat_file_byte;
                33: compression[31:24]<=fat_file_byte;
                default: ;
            endcase
            if(fat_file_byte_offset==53) begin
                header_checked<=1;
                if(!header_match) parser_error<=1;
            end
            if(header_checked && fat_file_byte_offset>=pixel_offset &&
               {32'd0,fat_file_byte_offset}<pixel_end) begin
                case(bmp_byte_idx)
                    0: blue_byte<=fat_file_byte;
                    1: green_byte<=fat_file_byte;
                    2: begin
                        bmp_pixel_data<={fat_file_byte,green_byte,blue_byte};
                        bmp_data_wr_en<=1;
                        pixel_count<=pixel_count+1'b1;
                    end
                    default: ;
                endcase
                bmp_byte_idx<=bmp_byte_idx==2 ? 0 : bmp_byte_idx+1'b1;
            end
        end
    end
end

assign fat_scan_start = scan_start_pulse;
assign fat_file_start = file_start_pulse;
assign scan_sector_grant = !scan_complete && fat_scan_sector_req;
assign file_sector_grant = scan_complete && fat_file_sector_req;
assign fat_sector_req = !sector_owned && !fat_sector_busy &&
                        (scan_sector_grant || file_sector_grant);
assign fat_sector_lba = scan_complete ? fat_file_sector_lba : fat_scan_sector_lba;
assign bmp_req = fat_sd_sec_read;
assign bmp_lba = fat_sd_sec_read_addr;

always @(posedge clk or posedge driver_reset) begin
    if (driver_reset) begin
        sector_owned <= 1'b0;
        sector_owner_file <= 1'b0;
    end else if (!sector_owned && fat_sector_req) begin
        sector_owned <= 1'b1;
        sector_owner_file <= file_sector_grant;
    end else if (sector_owned && fat_sector_done) begin
        sector_owned <= 1'b0;
    end
end

fat32_volume_scanner fat32_volume_scanner_m0(
    .clk                    (clk),
    .rst_n                  (~(rst | scan_abort)),
    .scan_start             (fat_scan_start),
    .scan_busy              (fat_scan_busy),
    .scan_done              (fat_scan_done),
    .fs_error               (fat_scan_error),
    .partition_lba          (fat_partition_lba),
    .sectors_per_cluster    (fat_sectors_per_cluster),
    .fat_start_lba          (fat_start_lba),
    .data_start_lba         (fat_data_start_lba),
    .max_cluster            (fat_max_cluster),
    .resource_valid         (fat_resource_valid),
    .resource0_cluster      (fat_resource0_cluster),
    .resource1_cluster      (fat_resource1_cluster),
    .resource2_cluster      (fat_resource2_cluster),
    .resource3_cluster      (fat_resource3_cluster),
    .resource0_size         (fat_resource0_size),
    .resource1_size         (fat_resource1_size),
    .resource2_size         (fat_resource2_size),
    .resource3_size         (fat_resource3_size),
    .sector_req             (fat_scan_sector_req),
    .sector_lba             (fat_scan_sector_lba),
    .sector_busy            (fat_scan_sector_busy),
    .sector_valid           (fat_scan_sector_valid),
    .sector_byte            (fat_scan_sector_byte),
    .sector_byte_index      (fat_scan_sector_byte_index),
    .sector_done            (fat_scan_sector_done),
    .sector_error           (fat_scan_sector_error)
);

fat32_file_streamer fat32_file_streamer_m0(
    .clk                    (clk),
    .rst_n                  (~file_reset),
    .file_start             (fat_file_start),
    .first_cluster          (load_cluster),
    .file_size             (load_size),
    .sectors_per_cluster    (fat_sectors_per_cluster),
    .fat_start_lba          (fat_start_lba),
    .data_start_lba         (fat_data_start_lba),
    .max_cluster            (fat_max_cluster),
    .file_busy              (fat_file_busy),
    .file_done              (fat_file_done),
    .file_valid             (fat_file_valid),
    .file_byte             (fat_file_byte),
    .file_byte_offset      (fat_file_byte_offset),
    .fs_error               (fat_file_error),
    .sector_req             (fat_file_sector_req),
    .sector_lba             (fat_file_sector_lba),
    .sector_busy            (fat_file_sector_busy),
    .sector_valid           (fat_file_sector_valid),
    .sector_byte            (fat_file_sector_byte),
    .sector_byte_index      (fat_file_sector_byte_index),
    .sector_done            (fat_file_sector_done),
    .sector_error           (fat_file_sector_error)
);

sd_sector_adapter sd_sector_adapter_m0(
    .clk                    (clk),
    .rst_n                  (~driver_reset),
    .sector_req             (fat_sector_req),
    .sector_lba             (fat_sector_lba),
    .sector_busy            (fat_adapter_busy),
    .sector_valid           (fat_sector_valid),
    .sector_byte            (fat_sector_byte),
    .sector_byte_index      (fat_sector_byte_index),
    .sector_done            (fat_sector_done),
    .sector_error           (fat_sector_error),
    .sd_sec_read            (fat_sd_sec_read),
    .sd_sec_read_addr       (fat_sd_sec_read_addr),
    .sd_sec_read_data_valid(bmp_valid),
    .sd_sec_read_data       (bmp_data),
    .sd_sec_read_end        (bmp_done)
);

// The external arbiter may be serving the reserved audio client.  Hold both
// FAT clients busy until the BMP transaction is granted and completed.
assign fat_sector_busy = fat_adapter_busy || bmp_busy;

assign fat_scan_sector_busy       = scan_complete || sector_owned || fat_sector_busy;
assign fat_scan_sector_valid      = fat_sector_valid && sector_owned && !sector_owner_file;
assign fat_scan_sector_byte       = sector_owned && !sector_owner_file ? fat_sector_byte : 8'd0;
assign fat_scan_sector_byte_index = sector_owned && !sector_owner_file ? fat_sector_byte_index : 9'd0;
assign fat_scan_sector_done       = fat_sector_done && sector_owned && !sector_owner_file;
assign fat_scan_sector_error      = sector_owned && !sector_owner_file ? fat_sector_error : 8'd0;
assign fat_file_sector_busy       = !scan_complete || sector_owned || fat_sector_busy;
assign fat_file_sector_valid      = fat_sector_valid && sector_owned && sector_owner_file;
assign fat_file_sector_byte       = sector_owned && sector_owner_file ? fat_sector_byte : 8'd0;
assign fat_file_sector_byte_index = sector_owned && sector_owner_file ? fat_sector_byte_index : 9'd0;
assign fat_file_sector_done       = fat_sector_done && sector_owned && sector_owner_file;
assign fat_file_sector_error      = sector_owned && sector_owner_file ? fat_sector_error : 8'd0;

endmodule
