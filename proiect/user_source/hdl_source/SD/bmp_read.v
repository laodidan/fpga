// Lesson 4: strict BMP validation, bounded reads and explicit completion.
module bmp_read #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer SD_READ_TIMEOUT_MS = 500
)(
    input                       clk,
    input                       rst,
    output                      ready,
    input                       load_abort,
    output reg [2:0]            bmp_error,
    output reg                  load_error,
    output reg                  load_done,
    output reg                  sd_recover,

    // 上电扫描：从 scan_start_sector 开始，顺序寻找前 scan_target_count 张 BMP
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,

    // 按指定扇区加载一张图到 SDRAM
    input                       load_start,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data
);

localparam [2:0] ST_IDLE=0, ST_SCAN=1, ST_LOAD_HDR=2,
                 ST_LOAD_WAIT=3, ST_LOAD_DATA=4, ST_WRITE_RELEASE=5;
localparam [2:0] ERR_NONE=0, ERR_HEADER=1, ERR_TIMEOUT=2, ERR_TRUNCATED=3;
// Divide first and use 64-bit arithmetic: 500*100000000 overflows 32 bits.
localparam [63:0] TIMEOUT_CALC = (64'd1 * CLK_FREQ_HZ * SD_READ_TIMEOUT_MS)/1000;
localparam [63:0] TIMEOUT_CYCLES = (TIMEOUT_CALC < 1) ? 1 : TIMEOUT_CALC;
reg [2:0] state;
reg [63:0] timeout_count;
reg [9:0] rd_cnt;
reg [7:0] header_0, header_1;
reg [31:0] file_len, pixel_offset, dib_size, width, height, compression;
reg [15:0] planes, bit_count;
reg [31:0] scan_sector, load_sector_latched, bmp_len_cnt, pixel_count;
reg [1:0] bmp_byte_idx;
wire [63:0] expected_pixels = 64'd1 * bmp_width * bmp_height;
wire [63:0] pixel_end = {32'd0,pixel_offset} + expected_pixels * 3;
wire signature_ok = header_0==8'h42 && header_1==8'h4d;
// This reader retains the existing fixed RGB24, bottom-up, row-aligned format.
wire header_match = signature_ok && dib_size>=40 && planes==1 &&
    width=={16'd0,bmp_width} && height=={16'd0,bmp_height} &&
    bmp_width!=0 && bmp_height!=0 && bmp_width[1:0]==0 &&
    bit_count==24 && compression==0 &&
    {32'd0,pixel_offset} >= 64'd14+{32'd0,dib_size} &&
    pixel_offset>=54 && pixel_offset<file_len &&
    {32'd0,file_len}>=pixel_end;
wire [32:0] file_sectors = ({1'b0,file_len}+33'd511)>>9;
wire [32:0] fast_next = {1'b0,scan_sector}+file_sectors;
// A damaged length must not wrap the sector address or skip the entire scan.
wire fast_skip_safe = signature_ok && dib_size>=40 &&
    pixel_offset>=54 && pixel_offset<file_len && file_sectors!=0 &&
    fast_next>{1'b0,scan_sector} &&
    fast_next<=({1'b0,scan_max_sector}+33'd1);
wire [32:0] scan_next = fast_skip_safe ? fast_next : {1'b0,scan_sector}+33'd1;
wire [10:0] sector_bytes = {1'b0,rd_cnt} + (sd_sec_read_data_valid ? 11'd1:11'd0);
wire pixel_byte_valid = state==ST_LOAD_DATA && sd_sec_read_data_valid &&
    bmp_len_cnt>=pixel_offset && {32'd0,bmp_len_cnt}<pixel_end;
wire pixel_emit = pixel_byte_valid && bmp_byte_idx==2;
wire [32:0] pixels_after = {1'b0,pixel_count} + (pixel_emit ? 33'd1:33'd0);
wire [32:0] bytes_after = {1'b0,bmp_len_cnt} + (sd_sec_read_data_valid ? 33'd1:33'd0);
assign ready = state==ST_IDLE;

// Failure wins over normal completion. Reset only the SD transaction layer
// on a timeout; never reset SDRAM or the displayed frame.
task fail;
    input [2:0] code;
    begin
        bmp_error <= code; load_error <= 1; load_done <= 0;
        if (state==ST_SCAN) scan_done <= 1;
        sd_recover <= (code==ERR_TIMEOUT);
        sd_sec_read <= 0; write_req <= 0; bmp_data_wr_en <= 0;
        state <= ST_IDLE; timeout_count <= 0;
    end
endtask

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state<=ST_IDLE; timeout_count<=0; rd_cnt<=0;
        header_0<=0; header_1<=0; file_len<=0; pixel_offset<=0;
        dib_size<=0; width<=0; height<=0; planes<=0; bit_count<=0; compression<=0;
        scan_sector<=0; load_sector_latched<=0; bmp_len_cnt<=0;
        pixel_count<=0; bmp_byte_idx<=0; bmp_data<=0; bmp_data_wr_en<=0;
        state_code<=0; sd_sec_read<=0; sd_sec_read_addr<=0; write_req<=0;
        scan_done<=0; scan_found_valid<=0; scan_found_sector<=0; scan_found_total<=0;
        bmp_error<=ERR_NONE; load_error<=0; load_done<=0; sd_recover<=0;
    end else begin
        load_error<=0; load_done<=0; sd_recover<=0;
        scan_found_valid<=0; bmp_data_wr_en<=0;
        if (state==ST_IDLE) timeout_count<=0;
        else timeout_count<=timeout_count+1'b1;
        if (sd_sec_read_end) begin rd_cnt<=0; timeout_count<=0; end
        else if (sd_sec_read_data_valid && (state==ST_SCAN || state==ST_LOAD_HDR || state==ST_LOAD_DATA))
            rd_cnt<=rd_cnt+1'b1;

        if ((state==ST_SCAN || state==ST_LOAD_HDR) && sd_sec_read_data_valid) begin
            case(rd_cnt)
                10'd0: header_0 <= sd_sec_read_data;
                10'd1: header_1 <= sd_sec_read_data;
                10'd2: file_len[7:0] <= sd_sec_read_data;
                10'd3: file_len[15:8] <= sd_sec_read_data;
                10'd4: file_len[23:16] <= sd_sec_read_data;
                10'd5: file_len[31:24] <= sd_sec_read_data;
                10'd10: pixel_offset[7:0] <= sd_sec_read_data;
                10'd11: pixel_offset[15:8] <= sd_sec_read_data;
                10'd12: pixel_offset[23:16] <= sd_sec_read_data;
                10'd13: pixel_offset[31:24] <= sd_sec_read_data;
                10'd14: dib_size[7:0] <= sd_sec_read_data;
                10'd15: dib_size[15:8] <= sd_sec_read_data;
                10'd16: dib_size[23:16] <= sd_sec_read_data;
                10'd17: dib_size[31:24] <= sd_sec_read_data;
                10'd18: width[7:0] <= sd_sec_read_data;
                10'd19: width[15:8] <= sd_sec_read_data;
                10'd20: width[23:16] <= sd_sec_read_data;
                10'd21: width[31:24] <= sd_sec_read_data;
                10'd22: height[7:0] <= sd_sec_read_data;
                10'd23: height[15:8] <= sd_sec_read_data;
                10'd24: height[23:16] <= sd_sec_read_data;
                10'd25: height[31:24] <= sd_sec_read_data;
                10'd26: planes[7:0] <= sd_sec_read_data;
                10'd27: planes[15:8] <= sd_sec_read_data;
                10'd28: bit_count[7:0] <= sd_sec_read_data;
                10'd29: bit_count[15:8] <= sd_sec_read_data;
                10'd30: compression[7:0] <= sd_sec_read_data;
                10'd31: compression[15:8] <= sd_sec_read_data;
                10'd32: compression[23:16] <= sd_sec_read_data;
                10'd33: compression[31:24] <= sd_sec_read_data;
                default: ;
            endcase
        end
        if (state==ST_LOAD_DATA && sd_sec_read_data_valid) begin
            bmp_len_cnt<=bmp_len_cnt+1'b1;
            if (pixel_byte_valid) begin
                case(bmp_byte_idx)
                    0: bmp_data[7:0]<=sd_sec_read_data;
                    1: bmp_data[15:8]<=sd_sec_read_data;
                    2: begin
                        bmp_data[23:16]<=sd_sec_read_data;
                        bmp_data_wr_en<=1; pixel_count<=pixel_count+1'b1;
                    end
                    default: ;
                endcase
                bmp_byte_idx <= (bmp_byte_idx==2) ? 0 : bmp_byte_idx+1'b1;
            end
        end
        if (load_abort && state!=ST_IDLE) fail(ERR_TIMEOUT);
        else if (state!=ST_IDLE && timeout_count>=TIMEOUT_CYCLES-1 &&
                 !sd_sec_read_end) fail(ERR_TIMEOUT);
        else case(state)
            ST_IDLE: begin
                state_code<=1; sd_sec_read<=0; write_req<=0; rd_cnt<=0;
                if (sd_init_done && (scan_start || load_start)) begin
                    header_0<=0; header_1<=0; file_len<=0; dib_size<=0;
                    planes<=0; pixel_count<=0; bmp_len_cnt<=0; bmp_byte_idx<=0;
                    bmp_error<=ERR_NONE;
                    if (scan_start) begin
                        scan_done<=0; scan_found_total<=0; scan_sector<=scan_start_sector;
                        sd_sec_read_addr<=scan_start_sector;
                        if (scan_start_sector>scan_max_sector || scan_target_count==0)
                            scan_done<=1;
                        else state<=ST_SCAN;
                    end else begin
                        load_sector_latched<=load_sector;
                        sd_sec_read_addr<=load_sector; state<=ST_LOAD_HDR;
                    end
                end
            end
            ST_SCAN: begin
                state_code<=2; sd_sec_read<=1;
                if (sd_sec_read_end) begin
                    sd_sec_read<=0;
                    if (sector_bytes!=512) fail(ERR_TRUNCATED);
                    else begin
                        if (header_match) begin
                            scan_found_valid<=1; scan_found_sector<=scan_sector;
                            scan_found_total<=scan_found_total+1'b1;
                        end
                        if ((header_match && scan_found_total+1>=scan_target_count) ||
                            scan_next>{1'b0,scan_max_sector}) begin
                            scan_done<=1; state<=ST_IDLE;
                        end else begin
                            scan_sector<=scan_next[31:0]; sd_sec_read_addr<=scan_next[31:0];
                        end
                    end
                end
            end
            ST_LOAD_HDR: begin
                state_code<=2; sd_sec_read<=1;
                if (sd_sec_read_end) begin
                    sd_sec_read<=0;
                    if (sector_bytes!=512) fail(ERR_TRUNCATED);
                    else if (!header_match) fail(ERR_HEADER);
                    else begin
                        write_req<=1; state<=ST_LOAD_WAIT;
                        sd_sec_read_addr<=load_sector_latched;
                    end
                end
            end
            ST_LOAD_WAIT: begin
                state_code<=3;
                if (write_req_ack) begin
                    write_req<=0; state<=ST_WRITE_RELEASE; timeout_count<=0;
                end
            end
            ST_WRITE_RELEASE: begin
                if (!write_req_ack) begin
                    state<=ST_LOAD_DATA; timeout_count<=0;
                    bmp_len_cnt<=0; pixel_count<=0; bmp_byte_idx<=0;
                end
            end
            ST_LOAD_DATA: begin
                state_code<=4; sd_sec_read<=1;
                if (sd_sec_read_end) begin
                    sd_sec_read<=0;
                    if (sector_bytes!=512) fail(ERR_TRUNCATED);
                    else if (bytes_after>=pixel_end) begin
                        if (pixels_after!=expected_pixels) fail(ERR_TRUNCATED);
                        else begin load_done<=1; bmp_error<=ERR_NONE; state<=ST_IDLE; end
                    end else if (sd_sec_read_addr==32'hffffffff) fail(ERR_HEADER);
                    else sd_sec_read_addr<=sd_sec_read_addr+1'b1;
                end
            end
            default: fail(ERR_TIMEOUT);
        endcase
    end
end
endmodule
