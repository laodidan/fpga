module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,
    parameter integer LONG_PRESS_MS     = 1000,   // KEY2长按触发时间，默认1秒
    parameter integer SD_READ_TIMEOUT_MS = 500,
    parameter integer AUTO_PLAY_MS      = 3000    // 自动轮播间隔，默认3秒
)(
    input                       clk,
    input                       rst,
    input                       sdram_init_done,
    input                       key_next,
    input                       key_auto,
    output reg [3:0]            state_code,       // 适配例程5的顺序，并改为reg型
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,
    output [1:0]                current_media_id, // committed image, not SDRAM bank

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO
);

wire key_next_press;
wire key_auto_press;
wire key_auto_stable;

wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire             sd_init_done;

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
reg              scan_complete;
reg              scan_abort;
reg [31:0]       scan_timeout_cnt;

reg              scan_start_pulse;
reg              file_start_pulse;
reg              write_req_r;
reg [1:0]        load_phase;
reg [31:0]       load_cluster;
reg [31:0]       load_size;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [31:0]       auto_cnt;
reg [2:0]        img_found_count;
reg [1:0]        img_idx;
assign current_media_id = img_idx;              
reg [1:0]        load_idx;             
reg [1:0]        pending_buf_idx;      
reg              next_req_pending;
reg              load_busy;
reg [7:0]        header_0, header_1;
reg [31:0]       file_len, pixel_offset, dib_size, width, height, compression;
reg [15:0]       planes, bit_count;
reg              header_checked;
reg              parser_error;
reg [31:0]       pixel_count;
reg [1:0]        bmp_byte_idx;
reg [7:0]        blue_byte, green_byte;
reg [23:0]       bmp_data;
reg              bmp_data_wr_en;
wire [63:0]      expected_pixels = 64'd1 * bmp_width * bmp_height;
wire [63:0]      pixel_end = {32'd0, pixel_offset} + expected_pixels * 3;
wire             header_match = header_0 == 8'h42 && header_1 == 8'h4d &&
    dib_size >= 40 && planes == 1 && width == {16'd0, bmp_width} &&
    height == {16'd0, bmp_height} && bmp_width != 0 && bmp_height != 0 &&
    bmp_width[1:0] == 0 && bit_count == 24 && compression == 0 &&
    {32'd0, pixel_offset} >= 64'd14 + {32'd0, dib_size} &&
    pixel_offset >= 54 && pixel_offset < file_len &&
    file_len <= load_size && {32'd0, file_len} >= pixel_end &&
    pixel_end <= {32'd0, load_size};

// 保留例程4优秀的防死机和超时机制
reg [31:0]       load_timeout_cnt;
reg              source_done_seen;
reg              write_done_seen;

// 长短按复用新增寄存器
reg [31:0]       long_press_cnt;       
reg              long_press_marked;     
reg              prev_req_pending;      

reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;
wire auto_tick;
localparam [1:0] LOAD_IDLE=0, LOAD_WAIT_ACK=1, LOAD_RELEASE=2, LOAD_STREAM=3;

assign write_req  = write_req_r;
assign write_en   = bmp_data_wr_en;
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];

wire [31:0] auto_play_cycles = (CLK_FREQ_HZ / 1000) * AUTO_PLAY_MS;
assign auto_tick = (auto_cnt == (auto_play_cycles - 1));

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (50)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(key_next_press)
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (50)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_auto),
    .press_pulse(key_auto_press),
    .button_stable_out(key_auto_stable)
);

function [1:0] next_index_limited;
    input [1:0] cur;
    input [2:0] count;
    begin
        case (count)
            3'd0: next_index_limited = 2'd0;
            3'd1: next_index_limited = 2'd0;
            3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : (cur + 2'd1);
            3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
            default: next_index_limited = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);
        endcase
    end
endfunction

function [1:0] prev_index_limited;
    input [1:0] cur;
    input [2:0] count;
    begin
        case (count)
            3'd0: prev_index_limited = 2'd0;
            3'd1: prev_index_limited = 2'd0;
            3'd2: prev_index_limited = (cur == 2'd0) ? 2'd1 : (cur - 2'd1);
            3'd3: prev_index_limited = (cur == 2'd0) ? 2'd2 : (cur - 2'd1);
            default: prev_index_limited = (cur == 2'd0) ? 2'd3 : (cur - 2'd1);
        endcase
    end
endfunction

function [31:0] cluster_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: cluster_lut = fat_resource0_cluster;
            2'd1: cluster_lut = fat_resource1_cluster;
            2'd2: cluster_lut = fat_resource2_cluster;
            2'd3: cluster_lut = fat_resource3_cluster;
            default: cluster_lut = fat_resource0_cluster;
        endcase
    end
endfunction

function [31:0] size_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: size_lut = fat_resource0_size;
            2'd1: size_lut = fat_resource1_size;
            2'd2: size_lut = fat_resource2_size;
            2'd3: size_lut = fat_resource3_size;
            default: size_lut = fat_resource0_size;
        endcase
    end
endfunction

function [1:0] next_buf_lut;
    input [1:0] cur_disp_buf;
    input       valid_now;
    begin
        if (!valid_now)
            next_buf_lut = 2'd0;
        else if (cur_disp_buf == 2'd0)
            next_buf_lut = 2'd1;
        else
            next_buf_lut = 2'd0;
    end
endfunction

// Lesson 4 recovery state. Course scope: one retry, then skip/hold.
reg [2:0] last_bmp_error;
reg retry_count, retry_pending;
reg [3:0] failed_mask;
reg [31:0] recovery_wait;
reg [4:0] sd_reset_count;
reg write_armed;
reg [3:0] arm_age;
reg [1:0] ack_sync;
reg load_abort;
wire driver_reset = rst || sd_reset_count!=0;
wire file_reset = rst || load_abort;
localparam [63:0] RECOVERY_CYCLES_RAW = (64'd1*CLK_FREQ_HZ*SD_READ_TIMEOUT_MS)/1000;
localparam [63:0] RECOVERY_CYCLES = RECOVERY_CYCLES_RAW<1 ? 1 : RECOVERY_CYCLES_RAW;
localparam [63:0] FRAME_LIMIT = 64'd5*CLK_FREQ_HZ;
wire have_candidate = |(fat_resource_valid & ~failed_mask);
wire load_error = parser_error || (fat_file_done && fat_file_error != 0);
wire failure_now = load_busy && (load_error || load_timeout_cnt>=FRAME_LIMIT-1);

function [1:0] select_image;
    input [1:0] current;
    input backwards;
    input [3:0] bad;
    reg [1:0] candidate;
    reg found;
    integer k;
    begin
        candidate=current; found=0; select_image=current;
        for(k=0;k<4;k=k+1) begin
            // Walk the physical four-entry resource map and skip holes.
            candidate=backwards ?
                      ((candidate == 2'd0) ? 2'd3 : candidate - 1'b1) :
                      ((candidate == 2'd3) ? 2'd0 : candidate + 1'b1);
            if(!found && !bad[candidate] && fat_resource_valid[candidate]) begin
                select_image=candidate; found=1;
            end
        end
    end
endfunction

function [2:0] resource_count;
    input [3:0] valid;
    integer n;
    begin
        resource_count = 3'd0;
        for(n=0;n<4;n=n+1)
            resource_count = resource_count + valid[n];
    end
endfunction

task start_image;
    input [1:0] index;
    input is_retry;
    begin
        if (!fat_resource_valid[index] || failed_mask[index]) begin
            // Missing resources cancel the request and preserve the last frame.
            write_req_r<=0; load_phase<=LOAD_IDLE; load_busy<=0;
            retry_pending<=0; recovery_wait<=0; load_abort<=0;
            next_req_pending<=0; prev_req_pending<=0;
        end else begin
            load_idx<=index; load_cluster<=cluster_lut(index); load_size<=size_lut(index);
            pending_buf_idx<=next_buf_lut(disp_buf_idx,display_valid);
            write_buf_idx<=next_buf_lut(disp_buf_idx,display_valid);
            write_req_r<=1; load_phase<=LOAD_WAIT_ACK;
            load_busy<=1; load_timeout_cnt<=0;
            source_done_seen<=0; write_done_seen<=0; write_armed<=0; arm_age<=0;
            header_0<=0; header_1<=0; file_len<=0; pixel_offset<=0; dib_size<=0;
            width<=0; height<=0; compression<=0; planes<=0; bit_count<=0;
            header_checked<=0; parser_error<=0; pixel_count<=0; bmp_byte_idx<=0;
            retry_pending<=0; recovery_wait<=0; auto_cnt<=0;
            if (!is_retry) retry_count<=0;
        end
    end
endtask

task abandon_image;
    begin
        failed_mask[load_idx]<=1;
        load_busy<=0; retry_pending<=0; source_done_seen<=0; write_done_seen<=0;
        write_armed<=0; recovery_wait<=0; write_req_r<=0; load_phase<=LOAD_IDLE;
        parser_error<=0;
        // No assignment to disp_buf_idx/img_idx/display_valid: keep last good frame.
        if (auto_play_en || !display_valid) next_req_pending<=1;
    end
endtask

always @(posedge clk or posedge rst) begin
    if(rst) begin
        wrfin_tgl_sync<=0; ack_sync<=0; scan_start_pulse<=0; file_start_pulse<=0;
        scan_kicked<=0; scan_complete<=0; scan_abort<=0; scan_timeout_cnt<=0;
        first_image_committed<=0; load_cluster<=0; load_size<=0;
        write_req_r<=0; load_phase<=LOAD_IDLE;
        auto_play_en<=0; auto_cnt<=0; img_found_count<=0; img_idx<=0; load_idx<=0;
        pending_buf_idx<=0; write_buf_idx<=0; disp_buf_idx<=0;
        next_req_pending<=0; prev_req_pending<=0; load_busy<=0;
        load_timeout_cnt<=0; load_abort<=0; source_done_seen<=0; write_done_seen<=0;
        display_valid<=0; state_code<=1; long_press_cnt<=0; long_press_marked<=0;
        last_bmp_error<=0; retry_count<=0; retry_pending<=0; failed_mask<=0;
        recovery_wait<=0; sd_reset_count<=0; write_armed<=0; arm_age<=0;
        header_0<=0; header_1<=0; file_len<=0; pixel_offset<=0; dib_size<=0;
        width<=0; height<=0; compression<=0; planes<=0; bit_count<=0;
        header_checked<=0; parser_error<=0; pixel_count<=0; bmp_byte_idx<=0;
        blue_byte<=0; green_byte<=0; bmp_data<=0; bmp_data_wr_en<=0;
    end else begin
        wrfin_tgl_sync<={wrfin_tgl_sync[1:0],write_finish_toggle};
        ack_sync<={ack_sync[0],write_req_ack};
        scan_start_pulse<=0; file_start_pulse<=0; load_abort<=0;
        bmp_data_wr_en<=0;
        if(sd_reset_count!=0) sd_reset_count<=sd_reset_count-1'b1;
        if(scan_kicked && !scan_complete && !scan_abort && fat_scan_busy) begin
            if(fat_scan_sector_done) scan_timeout_cnt<=0;
            else scan_timeout_cnt<=scan_timeout_cnt+1'b1;
            if(scan_timeout_cnt>=RECOVERY_CYCLES-1) begin
                scan_abort<=1; sd_reset_count<=16; state_code<=4'ha;
            end
        end
        if(fat_scan_done && !scan_abort) begin
            if(fat_scan_error==0 && fat_resource_valid!=4'h0) begin
                scan_complete<=1;
                img_found_count<=resource_count(fat_resource_valid);
                state_code<=1;
            end else begin
                scan_abort<=1; state_code<=4'h8;
            end
        end
        if(load_error) begin
            last_bmp_error<=fat_file_error!=0 ? 3'd3 : 3'd1;
            state_code<=fat_file_error!=0 ? 4'hb : 4'h9;
        end
        // Keep the existing key semantics and debounce module.
        if(key_auto_stable==0) begin
            if(long_press_cnt<((CLK_FREQ_HZ/1000)*LONG_PRESS_MS-1))
                long_press_cnt<=long_press_cnt+1'b1;
            else if(!long_press_marked) begin
                long_press_marked<=1; prev_req_pending<=1;
            end
        end else if(long_press_cnt!=0) begin
            if(!long_press_marked && scan_complete && img_found_count>1) begin
                auto_play_en<=~auto_play_en; auto_cnt<=0;
            end
            long_press_cnt<=0; long_press_marked<=0;
        end
        if(key_next_press && scan_complete && img_found_count>0) next_req_pending<=1;
        if(scan_complete && auto_play_en && display_valid && !load_busy &&
           !retry_pending && img_found_count>1) begin
            if(auto_tick) begin auto_cnt<=0; next_req_pending<=1; end
            else auto_cnt<=auto_cnt+1'b1;
        end else auto_cnt<=0;

        if(load_busy) begin
            load_timeout_cnt<=load_timeout_cnt+1'b1;
            if(write_req && ack_sync[1]) begin write_armed<=1; arm_age<=0; end
            else if(write_armed && arm_age<8) arm_age<=arm_age+1'b1;
            case(load_phase)
                LOAD_WAIT_ACK: if(ack_sync[1]) begin
                    write_req_r<=0; load_phase<=LOAD_RELEASE;
                end
                LOAD_RELEASE: if(!ack_sync[1]) begin
                    file_start_pulse<=1; load_phase<=LOAD_STREAM;
                end
                default: ;
            endcase
            if(fat_file_done) begin
                if(fat_file_error==0 && header_checked && !parser_error &&
                   {32'd0,pixel_count}==expected_pixels)
                    source_done_seen<=1;
                else parser_error<=1;
            end
            if(write_armed && arm_age==8 && write_finish_pulse) write_done_seen<=1;
        end

        if(load_busy && load_phase==LOAD_STREAM && fat_file_valid && !parser_error) begin
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
                        bmp_data<={fat_file_byte,green_byte,blue_byte};
                        bmp_data_wr_en<=1;
                        pixel_count<=pixel_count+1'b1;
                    end
                    default: ;
                endcase
                bmp_byte_idx<=bmp_byte_idx==2 ? 0 : bmp_byte_idx+1'b1;
            end
        end

        // Mutually exclusive priority chain: a failure cannot also commit a frame.
        if(failure_now) begin
            load_busy<=0; source_done_seen<=0; write_done_seen<=0; write_armed<=0;
            load_timeout_cnt<=0; recovery_wait<=0; write_req_r<=0;
            load_phase<=LOAD_IDLE; load_abort<=1; sd_reset_count<=16;
            bmp_data_wr_en<=0;
            if(!load_error) begin
                last_bmp_error<=2; state_code<=4'ha;
            end
            if(!retry_count) begin retry_count<=1; retry_pending<=1; end
            else abandon_image;
        end else if(load_busy && source_done_seen && write_done_seen) begin
            load_busy<=0; source_done_seen<=0; write_done_seen<=0; write_armed<=0;
            load_phase<=LOAD_IDLE;
            disp_buf_idx<=pending_buf_idx; img_idx<=load_idx; display_valid<=1;
            first_image_committed<=1; retry_count<=0; load_timeout_cnt<=0;
            last_bmp_error<=0; state_code<={2'b0,load_idx}+4'd1;
        end else if(retry_pending) begin
            if(recovery_wait>=32 && !fat_file_busy && !fat_sector_busy &&
               sd_init_done && sd_reset_count==0 && !load_abort)
                start_image(load_idx,1'b1);
            else if(recovery_wait>=RECOVERY_CYCLES-1) begin
                last_bmp_error<=2; state_code<=4'ha; abandon_image;
            end else recovery_wait<=recovery_wait+1'b1;
        end else if(!scan_kicked && sd_init_done && sdram_init_done &&
                    !fat_sector_busy && sd_reset_count==0) begin
            scan_start_pulse<=1; scan_kicked<=1; img_found_count<=0; failed_mask<=0;
        end else if(scan_complete && !fat_file_busy && !fat_sector_busy &&
                    sd_init_done && sd_reset_count==0 && !load_busy &&
                    !load_error && !load_abort && have_candidate) begin
            if(!first_image_committed) begin
                start_image(select_image(2'd3,1'b0,failed_mask),1'b0);
                next_req_pending<=0;
            end else if(next_req_pending) begin
                start_image(select_image(img_idx,1'b0,failed_mask),1'b0);
                next_req_pending<=0;
            end else if(prev_req_pending) begin
                start_image(select_image(img_idx,1'b1,failed_mask),1'b0);
                prev_req_pending<=0;
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
assign sd_sec_read = fat_sd_sec_read;
assign sd_sec_read_addr = fat_sd_sec_read_addr;

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
    .sector_busy            (fat_sector_busy),
    .sector_valid           (fat_sector_valid),
    .sector_byte            (fat_sector_byte),
    .sector_byte_index      (fat_sector_byte_index),
    .sector_done            (fat_sector_done),
    .sector_error           (fat_sector_error),
    .sd_sec_read            (fat_sd_sec_read),
    .sd_sec_read_addr       (fat_sd_sec_read_addr),
    .sd_sec_read_data_valid(sd_sec_read_data_valid),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_end        (sd_sec_read_end)
);

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

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (driver_reset),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (8'd0),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,
    output reg  press_pulse,
    output reg  button_stable_out
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
        button_stable_out <= 1'b1;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                button_stable_out <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule
