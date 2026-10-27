`timescale 1ns/1ps

// Fixed WAV metadata + streaming FAT32 volume initialization.
// FIRST_CLUSTER is a FAT32 cluster number; FILE_SIZE includes the entire WAV.
// Bound to the deployment FAT32 SD card captured in sim/stage8_wav_resources.json.
// Re-extract these constants after replacing files or changing the card layout.
module wav_resource_scanner #(
    parameter [31:0] PIC1_FIRST_CLUSTER = 32'd210,
    parameter [31:0] PIC1_FILE_SIZE     = 32'd1092558,
    parameter [31:0] PIC2_FIRST_CLUSTER = 32'd156,
    parameter [31:0] PIC2_FILE_SIZE     = 32'd929358,
    parameter [31:0] PIC3_FIRST_CLUSTER = 32'd185,
    parameter [31:0] PIC3_FILE_SIZE     = 32'd804558,
    parameter [31:0] PIC4_FIRST_CLUSTER = 32'd122,
    parameter [31:0] PIC4_FILE_SIZE     = 32'd1104078
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        scan_start,
    output wire        scan_busy,
    output reg         scan_done,
    output reg  [7:0]  fs_error,
    output reg  [31:0] partition_lba,
    output reg  [7:0]  sectors_per_cluster,
    output reg  [31:0] fat_start_lba,
    output reg  [31:0] data_start_lba,
    output reg  [31:0] max_cluster,
    output reg  [3:0]  resource_present,
    output reg  [3:0]  resource_valid,
    output reg  [31:0] resource0_cluster,
    output reg  [31:0] resource1_cluster,
    output reg  [31:0] resource2_cluster,
    output reg  [31:0] resource3_cluster,
    output reg  [31:0] resource0_size,
    output reg  [31:0] resource1_size,
    output reg  [31:0] resource2_size,
    output reg  [31:0] resource3_size,
    output reg         sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);
    localparam [7:0] ERR_NONE = 8'h00, ERR_SECTOR = 8'h10,
                     ERR_SIGNATURE = 8'h20, ERR_BPB = 8'h21,
                     ERR_RESOURCE = 8'h22;
    localparam [4:0] ST_IDLE = 5'd0, ST_REQ = 5'd1, ST_WAIT = 5'd2,
                     ST_CHECK = 5'd3, ST_MBR_END = 5'd4, ST_BPB = 5'd5,
                     ST_VOLUME_END = 5'd6, ST_FAT_SPAN = 5'd7,
                     ST_OVERHEAD = 5'd8, ST_DATA_BASE = 5'd9,
                     ST_DATA_SIZE = 5'd10, ST_CLUSTERS = 5'd11,
                     ST_MAX_CLUSTER = 5'd12, ST_FAT_CAPACITY = 5'd13,
                     ST_FAT_BASE = 5'd14, ST_ACTIVE_FAT = 5'd15,
                     ST_TABLE = 5'd16, ST_PUBLISH = 5'd17,
                     ST_FINISH = 5'd18;

    reg [4:0] state;
    reg read_boot;
    reg start_armed;
    reg [9:0] received_bytes;
    reg sector_sequence_error;
    reg [7:0] completed_sector_error;
    reg boot_jump_valid;
    reg [15:0] bytes_per_sector;
    reg [7:0] bpb_spc;
    reg [15:0] reserved_sectors;
    reg [7:0] fat_count;
    reg legacy_fields_nonzero;
    reg version_nonzero;
    reg [31:0] total_sectors;
    reg [31:0] fat_sectors;
    reg fat_mirroring_disabled;
    reg [3:0] active_fat;
    reg signature_low_valid;
    reg signature_high_valid;
    reg mbr_found;
    reg mbr_type_valid;
    reg [31:0] mbr_candidate_lba;
    reg [31:0] mbr_candidate_sectors;
    reg [31:0] partition_sectors;
    reg has_partition;
    reg [2:0] cluster_shift;
    // Reused for FAT span, volume overhead and cluster count in separate phases.
    reg [31:0] layout_value;
    reg [31:0] data_sector_count;
    reg [1:0] table_slot;

    assign scan_busy = (state != ST_IDLE);

    function valid_spc;
        input [7:0] value;
        begin
            valid_spc = (value != 0) && ((value & (value - 8'd1)) == 0);
        end
    endfunction

    function [2:0] spc_shift;
        input [7:0] value;
        begin
            case (value)
                8'd2: spc_shift = 3'd1;
                8'd4: spc_shift = 3'd2;
                8'd8: spc_shift = 3'd3;
                8'd16: spc_shift = 3'd4;
                8'd32: spc_shift = 3'd5;
                8'd64: spc_shift = 3'd6;
                8'd128: spc_shift = 3'd7;
                default: spc_shift = 3'd0;
            endcase
        end
    endfunction

    // One shared 33-bit add/subtract path for all volume layout operations.
    reg [31:0] math_a, math_b;
    reg math_subtract;
    always @* begin
        math_a = 32'd0;
        math_b = 32'd0;
        math_subtract = 1'b0;
        case (state)
            ST_MBR_END: begin math_a = partition_lba; math_b = partition_sectors; end
            ST_VOLUME_END: begin math_a = partition_lba; math_b = total_sectors; end
            ST_OVERHEAD: begin math_a = layout_value; math_b = {16'd0, reserved_sectors}; end
            ST_DATA_BASE: begin math_a = partition_lba; math_b = layout_value; end
            ST_DATA_SIZE: begin
                math_a = total_sectors;
                math_b = layout_value;
                math_subtract = 1'b1;
            end
            ST_MAX_CLUSTER: begin math_a = layout_value; math_b = 32'd1; end
            ST_FAT_CAPACITY: begin math_a = max_cluster; math_b = 32'd1; end
            ST_FAT_BASE: begin math_a = partition_lba; math_b = {16'd0, reserved_sectors}; end
            ST_ACTIVE_FAT: begin
                math_a = fat_start_lba;
                math_b = (fat_mirroring_disabled && active_fat != 0) ? fat_sectors : 32'd0;
            end
            default: begin end
        endcase
    end
    wire [32:0] math_result = {1'b0, math_a} +
                             ({1'b0, math_b} ^ {33{math_subtract}}) +
                             {32'd0, math_subtract};
    wire [31:0] shifted_cluster_count = data_sector_count >> cluster_shift;
    // Wiring-only shifts, with all overflow bits retained for comparison.
    wire [38:0] fat_entry_capacity = {fat_sectors, 7'd0};
    wire [40:0] volume_byte_capacity = {data_sector_count, 9'd0};

    // Four constants selected sequentially; no writable resource array.
    reg [31:0] selected_cluster, selected_size;
    always @* begin
        selected_cluster = 32'd0;
        selected_size = 32'd0;
        case (table_slot)
            2'd0: begin selected_cluster = PIC1_FIRST_CLUSTER; selected_size = PIC1_FILE_SIZE; end
            2'd1: begin selected_cluster = PIC2_FIRST_CLUSTER; selected_size = PIC2_FILE_SIZE; end
            2'd2: begin selected_cluster = PIC3_FIRST_CLUSTER; selected_size = PIC3_FILE_SIZE; end
            2'd3: begin selected_cluster = PIC4_FIRST_CLUSTER; selected_size = PIC4_FILE_SIZE; end
            default: begin end
        endcase
    end
    wire selected_valid = (selected_cluster >= 32'd2) &&
                          (selected_cluster <= max_cluster) &&
                          (selected_size != 0) &&
                          ({9'd0, selected_size} <= volume_byte_capacity);
    // An entry is configured if either field is supplied; valid still requires both.
    wire selected_present = (selected_cluster != 0) || (selected_size != 0);

    // MBR bytes 446..509 contain four consecutive 16-byte entries.
    // The low nibble selects fields; only one candidate and the first selected
    // FAT32 partition are kept. No reread, array or four-wide metadata bank.
    wire [8:0] mbr_offset = sector_byte_index - 9'd446;

    task finish_error;
        input [7:0] code;
        begin
            fs_error <= code;
            resource_present <= 4'd0;
            resource_valid <= 4'd0;
            state <= ST_FINISH;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            read_boot <= 1'b0;
            start_armed <= 1'b1;
            scan_done <= 1'b0;
            fs_error <= ERR_NONE;
            partition_lba <= 32'd0;
            sectors_per_cluster <= 8'd0;
            fat_start_lba <= 32'd0;
            data_start_lba <= 32'd0;
            max_cluster <= 32'd0;
            resource_present <= 4'd0;
            resource_valid <= 4'd0;
            resource0_cluster <= 32'd0;
            resource1_cluster <= 32'd0;
            resource2_cluster <= 32'd0;
            resource3_cluster <= 32'd0;
            resource0_size <= 32'd0;
            resource1_size <= 32'd0;
            resource2_size <= 32'd0;
            resource3_size <= 32'd0;
            sector_req <= 1'b0;
            sector_lba <= 32'd0;
            received_bytes <= 10'd0;
            sector_sequence_error <= 1'b0;
            completed_sector_error <= 8'd0;
            boot_jump_valid <= 1'b0;
            bytes_per_sector <= 16'd0;
            bpb_spc <= 8'd0;
            reserved_sectors <= 16'd0;
            fat_count <= 8'd0;
            legacy_fields_nonzero <= 1'b0;
            version_nonzero <= 1'b0;
            total_sectors <= 32'd0;
            fat_sectors <= 32'd0;
            fat_mirroring_disabled <= 1'b0;
            active_fat <= 4'd0;
            signature_low_valid <= 1'b0;
            signature_high_valid <= 1'b0;
            mbr_found <= 1'b0;
            mbr_type_valid <= 1'b0;
            mbr_candidate_lba <= 32'd0;
            mbr_candidate_sectors <= 32'd0;
            partition_sectors <= 32'd0;
            has_partition <= 1'b0;
            cluster_shift <= 3'd0;
            layout_value <= 32'd0;
            data_sector_count <= 32'd0;
            table_slot <= 2'd0;
        end else begin
            scan_done <= 1'b0;
            sector_req <= 1'b0;
            // Starts while busy are ignored; re-arm only after idle observes low.
            if (state == ST_IDLE && !scan_start)
                start_armed <= 1'b1;

            if (state == ST_WAIT && sector_valid) begin
                if (received_bytes >= 10'd512 ||
                    sector_byte_index != received_bytes[8:0])
                    sector_sequence_error <= 1'b1;
                if (received_bytes < 10'd513)
                    received_bytes <= received_bytes + 10'd1;

                case (sector_byte_index)
                    9'd0: boot_jump_valid <= (sector_byte == 8'heb || sector_byte == 8'he9);
                    9'd11: bytes_per_sector[7:0] <= sector_byte;
                    9'd12: bytes_per_sector[15:8] <= sector_byte;
                    9'd13: bpb_spc <= sector_byte;
                    9'd14: reserved_sectors[7:0] <= sector_byte;
                    9'd15: reserved_sectors[15:8] <= sector_byte;
                    9'd16: fat_count <= sector_byte;
                    9'd17, 9'd18, 9'd19, 9'd20, 9'd22, 9'd23:
                        legacy_fields_nonzero <= legacy_fields_nonzero || (sector_byte != 0);
                    9'd32: total_sectors[7:0] <= sector_byte;
                    9'd33: total_sectors[15:8] <= sector_byte;
                    9'd34: total_sectors[23:16] <= sector_byte;
                    9'd35: total_sectors[31:24] <= sector_byte;
                    9'd36: fat_sectors[7:0] <= sector_byte;
                    9'd37: fat_sectors[15:8] <= sector_byte;
                    9'd38: fat_sectors[23:16] <= sector_byte;
                    9'd39: fat_sectors[31:24] <= sector_byte;
                    9'd40: begin
                        fat_mirroring_disabled <= sector_byte[7];
                        active_fat <= sector_byte[3:0];
                    end
                    9'd42, 9'd43: version_nonzero <= version_nonzero || (sector_byte != 0);
                    9'd510: signature_low_valid <= (sector_byte == 8'h55);
                    9'd511: signature_high_valid <= (sector_byte == 8'haa);
                    default: begin end
                endcase

                if (!read_boot && !mbr_found &&
                    sector_byte_index >= 9'd446 && sector_byte_index <= 9'd509) begin
                    case (mbr_offset[3:0])
                        4'd4: mbr_type_valid <= (sector_byte == 8'h0b || sector_byte == 8'h0c ||
                                                sector_byte == 8'h1b || sector_byte == 8'h1c);
                        4'd8: mbr_candidate_lba[7:0] <= sector_byte;
                        4'd9: mbr_candidate_lba[15:8] <= sector_byte;
                        4'd10: mbr_candidate_lba[23:16] <= sector_byte;
                        4'd11: mbr_candidate_lba[31:24] <= sector_byte;
                        4'd12: mbr_candidate_sectors[7:0] <= sector_byte;
                        4'd13: mbr_candidate_sectors[15:8] <= sector_byte;
                        4'd14: mbr_candidate_sectors[23:16] <= sector_byte;
                        4'd15: if (mbr_type_valid) begin
                            mbr_found <= 1'b1;
                            partition_lba <= mbr_candidate_lba;
                            // Include the current final byte without an NBA race.
                            partition_sectors <= {sector_byte, mbr_candidate_sectors[23:0]};
                        end
                        default: begin end
                    endcase
                end
            end

            case (state)
                ST_IDLE: if (scan_start && start_armed) begin
                    start_armed <= 1'b0;
                    fs_error <= ERR_NONE;
                    partition_lba <= 32'd0;
                    sectors_per_cluster <= 8'd0;
                    fat_start_lba <= 32'd0;
                    data_start_lba <= 32'd0;
                    max_cluster <= 32'd0;
                    resource_present <= 4'd0;
                    resource_valid <= 4'd0;
                    partition_sectors <= 32'd0;
                    has_partition <= 1'b0;
                    mbr_found <= 1'b0;
                    read_boot <= 1'b0;
                    sector_lba <= 32'd0;
                    state <= ST_REQ;
                end
                ST_REQ: if (!sector_busy) begin
                    received_bytes <= 10'd0;
                    sector_sequence_error <= 1'b0;
                    completed_sector_error <= 8'd0;
                    legacy_fields_nonzero <= 1'b0;
                    version_nonzero <= 1'b0;
                    signature_low_valid <= 1'b0;
                    signature_high_valid <= 1'b0;
                    sector_req <= 1'b1;
                    state <= ST_WAIT;
                end
                ST_WAIT: if (sector_done) begin
                    completed_sector_error <= sector_error;
                    // CHECK sees the coincident last byte and sequence/count updates.
                    state <= ST_CHECK;
                end
                ST_CHECK: begin
                    if (completed_sector_error != 0 || sector_sequence_error ||
                        received_bytes != 10'd512)
                        finish_error(ERR_SECTOR);
                    else if (!signature_low_valid || !signature_high_valid)
                        finish_error(ERR_SIGNATURE);
                    else if (read_boot)
                        state <= ST_BPB;
                    else if (boot_jump_valid && bytes_per_sector == 16'd512) begin
                        // BPB bytes can overlap an apparent MBR; superfloppy wins.
                        partition_lba <= 32'd0;
                        partition_sectors <= 32'd0;
                        state <= ST_BPB;
                    end else if (!mbr_found || partition_lba == 0 || partition_sectors == 0)
                        finish_error(ERR_BPB);
                    else
                        state <= ST_MBR_END;
                end
                ST_MBR_END: begin
                    if (math_result > 33'h100000000)
                        finish_error(ERR_BPB);
                    else begin
                        has_partition <= 1'b1;
                        read_boot <= 1'b1;
                        sector_lba <= partition_lba;
                        state <= ST_REQ;
                    end
                end
                ST_BPB: begin
                    if (bytes_per_sector != 16'd512 || !valid_spc(bpb_spc) ||
                        reserved_sectors == 0 || (fat_count != 1 && fat_count != 2) ||
                        legacy_fields_nonzero || version_nonzero ||
                        total_sectors == 0 || fat_sectors == 0 ||
                        (has_partition && total_sectors > partition_sectors) ||
                        (fat_mirroring_disabled && {4'd0, active_fat} >= fat_count))
                        finish_error(ERR_BPB);
                    else begin
                        sectors_per_cluster <= bpb_spc;
                        cluster_shift <= spc_shift(bpb_spc);
                        state <= ST_VOLUME_END;
                    end
                end
                ST_VOLUME_END: begin
                    // Exclusive end may be exactly 2^32; sector LBAs may not.
                    if (math_result > 33'h100000000)
                        finish_error(ERR_BPB);
                    else state <= ST_FAT_SPAN;
                end
                ST_FAT_SPAN: begin
                    if (fat_count == 8'd2 && fat_sectors[31])
                        finish_error(ERR_BPB);
                    else begin
                        layout_value <= (fat_count == 8'd2) ? (fat_sectors << 1) : fat_sectors;
                        state <= ST_OVERHEAD;
                    end
                end
                ST_OVERHEAD: begin
                    if (math_result >= {1'b0, total_sectors})
                        finish_error(ERR_BPB);
                    else begin layout_value <= math_result[31:0]; state <= ST_DATA_BASE; end
                end
                ST_DATA_BASE: begin
                    if (math_result[32])
                        finish_error(ERR_BPB);
                    else begin data_start_lba <= math_result[31:0]; state <= ST_DATA_SIZE; end
                end
                ST_DATA_SIZE: begin
                    // ST_OVERHEAD has proved total_sectors > layout_value.
                    data_sector_count <= math_result[31:0];
                    state <= ST_CLUSTERS;
                end
                ST_CLUSTERS: begin
                    if (shifted_cluster_count == 0 || shifted_cluster_count > 32'h0fffffee)
                        finish_error(ERR_BPB);
                    else begin layout_value <= shifted_cluster_count; state <= ST_MAX_CLUSTER; end
                end
                ST_MAX_CLUSTER: begin
                    max_cluster <= math_result[31:0];
                    state <= ST_FAT_CAPACITY;
                end
                ST_FAT_CAPACITY: begin
                    // Entries 0 and 1 are reserved: require cluster_count + 2.
                    if (fat_entry_capacity < {6'd0, math_result})
                        finish_error(ERR_BPB);
                    else state <= ST_FAT_BASE;
                end
                ST_FAT_BASE: begin
                    if (math_result[32])
                        finish_error(ERR_BPB);
                    else begin fat_start_lba <= math_result[31:0]; state <= ST_ACTIVE_FAT; end
                end
                ST_ACTIVE_FAT: begin
                    if (math_result[32])
                        finish_error(ERR_BPB);
                    else begin
                        fat_start_lba <= math_result[31:0];
                        table_slot <= 2'd0;
                        state <= ST_TABLE;
                    end
                end
                ST_TABLE: begin
                    resource_present[table_slot] <= selected_present;
                    resource_valid[table_slot] <= selected_valid;
                    if (table_slot == 2'd3) state <= ST_PUBLISH;
                    else table_slot <= table_slot + 2'd1;
                end
                ST_PUBLISH: begin
                    resource0_cluster <= PIC1_FIRST_CLUSTER;
                    resource1_cluster <= PIC2_FIRST_CLUSTER;
                    resource2_cluster <= PIC3_FIRST_CLUSTER;
                    resource3_cluster <= PIC4_FIRST_CLUSTER;
                    resource0_size <= PIC1_FILE_SIZE;
                    resource1_size <= PIC2_FILE_SIZE;
                    resource2_size <= PIC3_FILE_SIZE;
                    resource3_size <= PIC4_FILE_SIZE;
                    fs_error <= (resource_valid == 4'hf) ? ERR_NONE : ERR_RESOURCE;
                    state <= ST_FINISH;
                end
                ST_FINISH: begin
                    scan_done <= 1'b1;
                    state <= ST_IDLE;
                end
                default: finish_error(ERR_BPB);
            endcase
        end
    end
endmodule
