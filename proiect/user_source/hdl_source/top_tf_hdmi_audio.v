`timescale 1ns/1ps

module top(
    input clk,
    input rst_n,
    input key1,
    input key2,
    input key_volume,
    input sw1,
    input sw2,
    input sw3,
    input sw4,
    output [5:0] seg_sel,
    output [7:0] seg_data,
    output HDMI_CLK_P,
    output HDMI_D2_P,
    output HDMI_D1_P,
    output HDMI_D0_P,
    output HDMI_DDC_SCL,
    inout HDMI_DDC_SDA,
    output sd_ncs,
    output sd_dclk,
    output sd_mosi,
    input sd_miso,
    output FLASH_CS,
    output FLASH_SCLK,
    output FLASH_SDI,
    input FLASH_SDO,
    output FLASH_WP,
    output FLASH_HOLD
);
parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS = 21;
// 0 bypass; 8/16/32 supported. Manual has two ramps plus black dwell.
parameter integer FADE_FRAMES = 16;
// Set to 0 to retain the original tone-only debug configuration.
parameter integer WAV_PCM_ENABLE = 1;
parameter integer AUTO_GUIDE_DEFAULT = 0;
// Preserve the original lock-settle interval; allow shorter verification runs.
parameter [19:0] STARTUP_COUNT_MAX = 20'hfffff;
// Diagnostic build only: 1 replaces the fullscreen SDRAM picture with colour
// bars and gray/R/G/B ramps ahead of fade, brightness/contrast, OSD and HDMI.
parameter integer VIDEO_TEST_PATTERN = 0;
localparam IMAGE_WORDS = 307200;
// Keep the existing 800x525 timing. With ACTIVE_LAST=1 the first active
// horizontal count is H_FP+H_SYNC+H_BP; the OSD expects a zero-based X.
localparam VIDEO_H_ACT = 640;
localparam VIDEO_H_FP = 16;
localparam VIDEO_H_SYNC = 96;
localparam VIDEO_H_BP = 48;
localparam [11:0] VIDEO_H_BLANK = VIDEO_H_FP + VIDEO_H_SYNC + VIDEO_H_BP;
localparam VIDEO_V_FP = 10;
localparam VIDEO_V_SYNC = 2;
localparam VIDEO_V_BP = 33;
localparam [11:0] VIDEO_V_BLANK = VIDEO_V_FP + VIDEO_V_SYNC + VIDEO_V_BP;

wire sd_card_clk, ext_mem_clk, ext_mem_clk_sft, video_clk, hdmi_5x_clk;
wire sys_locked, video_locked;
wire Sdr_init_done, Sdr_init_ref_vld, Sdr_busy;
wire startup_rst_n, business_release_n;
wire rst_sys_n, rst_sd_n, rst_mem_n, rst_pixel_n;
wire ctrl_rst_n, mem_rst_n, pixel_rst_n, hdmi_rst_n;
wire pll_locked = sys_locked && video_locked;
wire player_enable;
wire player_sd_rst_n, player_mem_rst_n, player_pixel_rst_n;
wire audio_pixel_rst_n;
wire rst_sd = !rst_sd_n;
wire rst_mem = !rst_mem_n;
wire rst_pixel = !rst_pixel_n;
reg [19:0] startup_count = 20'd0;

reset_sync u_rst_startup (
    .clk(clk), .ext_rst_n(rst_n && pll_locked),
    .rst_n_sync(startup_rst_n)
);
always @(posedge clk or negedge startup_rst_n) begin
    if (!startup_rst_n) startup_count <= 20'd0;
    else if (startup_count != STARTUP_COUNT_MAX)
        startup_count <= startup_count + 1'b1;
end
// Initialization domains must run before player_enable can become true.
assign business_release_n = startup_rst_n &&
                            (startup_count == STARTUP_COUNT_MAX);
reset_sync u_rst_sys (
    .clk(clk), .ext_rst_n(business_release_n), .rst_n_sync(ctrl_rst_n)
);
reset_sync u_rst_sd (
    .clk(sd_card_clk), .ext_rst_n(business_release_n), .rst_n_sync(rst_sd_n)
);
reset_sync u_rst_mem (
    .clk(ext_mem_clk), .ext_rst_n(business_release_n), .rst_n_sync(mem_rst_n)
);
reset_sync u_rst_pixel (
    .clk(video_clk), .ext_rst_n(business_release_n && hdmi_rst_n),
    .rst_n_sync(pixel_rst_n)
);
reset_sync u_rst_hdmi (
    .clk(hdmi_5x_clk), .ext_rst_n(business_release_n),
    .rst_n_sync(hdmi_rst_n)
);
assign rst_sys_n = ctrl_rst_n;
assign rst_mem_n = mem_rst_n;
assign rst_pixel_n = pixel_rst_n;

reset_sync u_rst_player_sd (
    .clk(sd_card_clk), .ext_rst_n(rst_sd_n && player_enable),
    .rst_n_sync(player_sd_rst_n)
);
reset_sync u_rst_player_mem (
    .clk(ext_mem_clk), .ext_rst_n(mem_rst_n && player_enable),
    .rst_n_sync(player_mem_rst_n)
);
reset_sync u_rst_player_pixel (
    .clk(video_clk), .ext_rst_n(pixel_rst_n && player_enable),
    .rst_n_sync(player_pixel_rst_n)
);
sys_pll sys_pll_m0 (
    .extlock(sys_locked), .refclk(clk), .clk0_out(sd_card_clk),
    .clk1_out(ext_mem_clk), .clk2_out(ext_mem_clk_sft), .reset(1'b0)
);
video_pll video_pll_m0 (
    .extlock(video_locked), .refclk(clk), .clk0_out(video_clk),
    .clk1_out(hdmi_5x_clk), .reset(1'b0)
);

// 125 MHz SDRAM init status -> 100 MHz card domain.
(* async_reg = "true" *) reg [1:0] init_sync_sd;
always @(posedge sd_card_clk or negedge rst_sd_n) begin
    if (!rst_sd_n) init_sync_sd <= 0;
    else init_sync_sd <= {init_sync_sd[0], Sdr_init_done};
end

wire key_next_event, key_prev_event, key_auto_event;
wire key1_short_event, key2_short_event, key2_long_event, combo_long_event;
wire mode_toggle_req, focus_next_req, focus_prev_req, select_req;
wire [3:0] load_state_code;
wire [3:0] state_code_sd;
(* async_reg = "true" *) reg [3:0] state_code_sync1, state_code_sync2;
always @(posedge clk or negedge rst_sys_n) begin
    if(!rst_sys_n) begin state_code_sync1<=4'd1; state_code_sync2<=4'd1; end
    else begin state_code_sync1<=state_code_sd; state_code_sync2<=state_code_sync1; end
end
wire [6:0] seg_data_0;
wire sd_event_valid, sd_event_full;
wire [1:0] sd_event_type;
wire [31:0] sd_event_data;
reg loader_result_toggle;
reg [7:0] loader_result_error;
wire loader_done;
wire [7:0] loader_error;
wire [3:0] image_complete_mem;
wire sd_init_done;
startup_ctrl u_startup_ctrl (
    .clk(clk), .rst_n(ctrl_rst_n), .pll_locked(pll_locked),
    .sd_init_done(sd_init_done), .sdram_init_done(Sdr_init_done),
    .player_enable(player_enable)
);
wire bmp_req;
wire [31:0] bmp_lba;
wire bmp_busy;
wire [7:0] bmp_data;
wire bmp_valid;
wire bmp_done;
wire [7:0] bmp_error;

sd_card_bmp_4bank #(
    .CLK_FREQ_HZ(100_000_000),
    .SCAN_START_SECTOR(32'd0), .SCAN_MAX_SECTOR(32'd131071),
    .SCAN_TARGET_COUNT(3'd4)
) u_sd_bmp (
    .clk(sd_card_clk), .rst(rst_sd), .sdram_init_done(init_sync_sd[1]),
    // Legacy BMP scanner key inputs are deliberately held inactive.  All
    // user gestures are arbitrated by media_key_control below.
    .key_next(1'b1), .key_auto(1'b1),
    .key_next_event(key_next_event), .key_prev_event(key_prev_event),
    .key_auto_event(key_auto_event), .state_code(load_state_code),
    .bmp_width(16'd640), .bmp_height(16'd480),
    .event_valid(sd_event_valid), .event_type(sd_event_type),
    .event_data(sd_event_data), .event_full(sd_event_full),
    .loader_result_toggle(loader_result_toggle),
    .loader_result_error(loader_result_error),
    .sd_init_done(sd_init_done),
    .bmp_req(bmp_req), .bmp_lba(bmp_lba), .bmp_busy(bmp_busy),
    .bmp_data(bmp_data), .bmp_valid(bmp_valid), .bmp_done(bmp_done),
    .bmp_error(bmp_error)
);
media_key_control #(.CLK_FREQ_HZ(100_000_000), .DEBOUNCE_MS(20),
                    .LONG_PRESS_MS(1000)) u_media_key_control (
    .clk(sd_card_clk), .rst_n(player_sd_rst_n),
    .key1(key1), .key2(key2),
    .key1_short_event(key1_short_event),
    .key2_short_event(key2_short_event),
    .key2_long_event(key2_long_event),
    .combo_long_event(combo_long_event),
    .key1_stable_pressed(), .key2_stable_pressed()
);
thumbnail_controller u_thumbnail_controller (
    .clk(sd_card_clk), .rst_n(player_sd_rst_n),
    .key1_short_event(key1_short_event),
    .key2_short_event(key2_short_event),
    .combo_long_event(combo_long_event),
    .mode_toggle_req(mode_toggle_req),
    .focus_next_req(focus_next_req),
    .focus_prev_req(focus_prev_req),
    .select_req(select_req)
);
// WAV foundation: all clients share the existing SD clock/reset domain.
// sd_init_done stays high; the scanner accepts it once until reset/rearm.
wire [1:0] current_index_sys;
wire active_valid_sys;
wire display_mode_sys;
wire auto_enabled_sd;
wire player_request_busy, player_switch_pending, index_commit_sys;
wire auto_next_req, auto_next_accepted, auto_request_ready;
wire [1:0] auto_next_index;
wire audio_switch_ready;
wire wav_scan_busy, wav_scan_done;
wire [7:0] wav_scan_error, wav_spc;
wire [31:0] wav_partition_lba, wav_fat_lba, wav_data_lba, wav_max_cluster;
wire [3:0] wav_present, wav_valid_table;
wire [31:0] wav_cluster0, wav_cluster1, wav_cluster2, wav_cluster3;
wire [31:0] wav_size0, wav_size1, wav_size2, wav_size3;
wire wav_scan_req, wav_scan_sector_busy, wav_scan_sector_valid, wav_scan_sector_done;
wire [31:0] wav_scan_lba;
wire [7:0] wav_scan_byte, wav_scan_sector_error;
wire [8:0] wav_scan_byte_index;
wire wav_file_req, wav_file_sector_busy, wav_file_sector_valid, wav_file_sector_done;
wire [31:0] wav_file_lba;
wire [7:0] wav_file_sector_byte, wav_file_sector_error;
wire [8:0] wav_file_byte_index;
wire [31:0] wav_start_cluster, wav_file_size;
wire wav_resource_valid;
wire wav_file_busy, wav_file_done, wav_file_valid;
wire [7:0] wav_file_byte, wav_file_error;
wire [31:0] wav_file_offset;
wire audio_req, audio_start, audio_valid, audio_done;
wire [31:0] audio_lba;
wire [7:0] audio_data;

wav_resource_scanner u_wav_resource_scanner (
    .clk(sd_card_clk), .rst_n(rst_sd_n), .scan_start(sd_init_done),
    .scan_busy(wav_scan_busy), .scan_done(wav_scan_done), .fs_error(wav_scan_error),
    .partition_lba(wav_partition_lba), .sectors_per_cluster(wav_spc),
    .fat_start_lba(wav_fat_lba), .data_start_lba(wav_data_lba),
    .max_cluster(wav_max_cluster), .resource_present(wav_present),
    .resource_valid(wav_valid_table),
    .resource0_cluster(wav_cluster0), .resource1_cluster(wav_cluster1),
    .resource2_cluster(wav_cluster2), .resource3_cluster(wav_cluster3),
    .resource0_size(wav_size0), .resource1_size(wav_size1),
    .resource2_size(wav_size2), .resource3_size(wav_size3),
    .sector_req(wav_scan_req), .sector_lba(wav_scan_lba),
    .sector_busy(wav_scan_sector_busy), .sector_valid(wav_scan_sector_valid),
    .sector_byte(wav_scan_byte), .sector_byte_index(wav_scan_byte_index),
    .sector_done(wav_scan_sector_done), .sector_error(wav_scan_sector_error)
);
audio_file_manager u_audio_file_manager (
    .clk(sd_card_clk), .rst_n(rst_sd_n),
    .scan_done(wav_scan_done), .scan_error(wav_scan_error),
    .scanned_resource_valid(wav_valid_table),
    .resource0_cluster(wav_cluster0), .resource1_cluster(wav_cluster1),
    .resource2_cluster(wav_cluster2), .resource3_cluster(wav_cluster3),
    .resource0_size(wav_size0), .resource1_size(wav_size1),
    .resource2_size(wav_size2), .resource3_size(wav_size3),
    // Existing committed SD-domain snapshot; no crossing from pixel index.
    .wav_index(current_index_sys), .start_cluster(wav_start_cluster),
    .file_size(wav_file_size), .resource_valid(wav_resource_valid)
);
// Each session locks committed metadata; a switch stops at a sector boundary
// before clearing only the audio session, never the SD or HDMI video engines.
reg wav_session_started;
reg [1:0] wav_session_index;
reg wav_session_reset;
reg wav_sector_reserved;
reg [1:0] wav_sector_drain;
wire wav_session_sd_rst_n = rst_sd_n && !wav_session_reset;
wire pcm_session_rst_n;
wire audio_session_tail_drained;
(* async_reg = "true" *) reg [1:0] audio_tail_drained_sd;
(* async_reg = "true" *) reg [1:0] audio_stop_pixel;
(* async_reg = "true" *) reg [1:0] pcm_session_ready_sd;
reg [1:0] session_reset_hold;
wire audio_stop_pending = player_switch_pending || display_mode_sys;
reg wav_parse_start, wav_stream_start;
reg [31:0] wav_session_cluster, wav_session_size;
reg [7:0] wav_session_spc;
reg [31:0] wav_session_fat_lba, wav_session_data_lba, wav_session_max_cluster;
reg wav_session_error;
reg wav_play_enabled;
reg [31:0] wav_samples_received;
wire wav_format_valid, wav_pcm_valid;
wire signed [15:0] wav_pcm_data;
wire [31:0] wav_pcm_data_start, wav_pcm_data_length;
wire [7:0] wav_parser_error;
wire [3:0] wav_parser_state;
wire pcm_fifo_full, pcm_fifo_empty;
wire [12:0] pcm_fifo_level;
wire pcm_out_valid, pcm_rd_en;
wire signed [15:0] pcm_out_data;
wire pcm_queue_rst_n;
wire wav_underflow, wav_output_stall;
// Pixel-domain completion is converted to a persistent toggle for SD CDC.
wire wav_done;
reg wav_done_toggle_pixel;
(* async_reg = "true" *) reg [1:0] wav_done_sync_sd;
reg wav_done_seen_sd, wav_done_session;
wire wav_done_event_sd = wav_done_sync_sd[1] != wav_done_seen_sd;
reg parser_pcm_end_sd;
(* async_reg = "true" *) reg [2:0] parser_pcm_end_pixel;
(* async_reg = "true" *) reg [1:0] audio_ready_sd;
(* async_reg = "true" *) reg [1:0] wav_enable_pixel;

// FIFO and output queue share a global clear, independent of HDMI X-lock.
// Resetting only one FIFO pointer would invalidate its CDC state.
reset_sync u_rst_pcm_queue (
    .clk(video_clk), .ext_rst_n(rst_sd_n), .rst_n_sync(pcm_queue_rst_n)
);
reset_sync u_rst_pcm_session (
    .clk(video_clk), .ext_rst_n(wav_session_sd_rst_n),
    .rst_n_sync(pcm_session_rst_n)
);
assign audio_switch_ready = wav_session_reset && !pcm_session_ready_sd[1];
always @(posedge sd_card_clk or negedge rst_sd_n) begin
    if (!rst_sd_n) begin
        pcm_session_ready_sd <= 2'b00;
        audio_tail_drained_sd <= 2'b00;
        wav_session_reset <= 1'b1;
        session_reset_hold <= 2'd3;
    end else begin
        pcm_session_ready_sd <= {pcm_session_ready_sd[0], pcm_session_rst_n};
        audio_tail_drained_sd <= {audio_tail_drained_sd[0],
            audio_stop_pixel[1] && !wav_enable_pixel[1] &&
            audio_session_tail_drained};
        if (audio_stop_pending) begin
            // Existing requests and sector payloads are never cancelled.
            if (!wav_file_req && !wav_file_sector_busy &&
                !wav_sector_reserved && wav_sector_drain == 0 &&
                (audio_tail_drained_sd[1] || !audio_ready_sd[1])) begin
                wav_session_reset <= 1'b1;
                session_reset_hold <= 2'd3;
            end
        end else if (wav_session_reset) begin
            if (session_reset_hold != 0)
                session_reset_hold <= session_reset_hold - 1'b1;
            else if (audio_switch_ready && active_valid_sys &&
                     !index_commit_sys && audio_ready_sd[1])
                wav_session_reset <= 1'b0;
        end
    end
end
always @(posedge video_clk or negedge pcm_queue_rst_n) begin
    if (!pcm_queue_rst_n)
        audio_stop_pixel <= 2'b00;
    else
        audio_stop_pixel <= {audio_stop_pixel[0], audio_stop_pending};
end
always @(posedge video_clk or negedge pcm_queue_rst_n) begin
    if (!pcm_queue_rst_n)
        wav_done_toggle_pixel <= 1'b0;
    else if (wav_done)
        wav_done_toggle_pixel <= ~wav_done_toggle_pixel;
end
always @(posedge sd_card_clk or negedge rst_sd_n) begin
    if (!rst_sd_n) begin
        wav_done_sync_sd <= 2'b00;
        wav_done_seen_sd <= 1'b0;
        wav_done_session <= 1'b0;
    end else begin
        wav_done_sync_sd <= {wav_done_sync_sd[0], wav_done_toggle_pixel};
        wav_done_seen_sd <= wav_done_sync_sd[1];
        if (wav_session_reset || audio_stop_pending || index_commit_sys)
            wav_done_session <= 1'b0;
        else if (wav_done_event_sd && wav_session_started && wav_play_enabled &&
                 wav_session_index == current_index_sys && !wav_session_error)
            wav_done_session <= 1'b1;
    end
end
auto_guide_fsm u_auto_guide_fsm (
    .clk(sd_card_clk), .rst_n(player_sd_rst_n),
    .wav_done(wav_done_session),
    .auto_enable(auto_enabled_sd && WAV_PCM_ENABLE != 0 && !display_mode_sys),
    .active_index(current_index_sys), .active_valid(active_valid_sys),
    .index_commit(index_commit_sys),
    .audio_started(wav_session_started && wav_play_enabled &&
                   wav_session_index == current_index_sys &&
                   !wav_session_reset && !wav_session_error),
    .request_accepted(auto_next_accepted),
    .switch_pending(player_switch_pending),
    .auto_next_req(auto_next_req), .next_index(auto_next_index)
);
always @(posedge sd_card_clk or negedge rst_sd_n) begin
    if (!rst_sd_n)
        audio_ready_sd <= 2'b00;
    else
        audio_ready_sd <= {audio_ready_sd[0], audio_pixel_rst_n};
end
always @(posedge video_clk or negedge pcm_session_rst_n) begin
    if (!pcm_session_rst_n)
        wav_enable_pixel <= 2'b00;
    else
        wav_enable_pixel <= {wav_enable_pixel[0], wav_play_enabled};
end

// Wait for the last registered PCM write AND the parser's successful final
// file check. IDLE alone (also used before start/on errors) is insufficient.
// Three sync stages make end no earlier than the FIFO write-pointer visibility.
always @(posedge sd_card_clk or negedge wav_session_sd_rst_n) begin
    if (!wav_session_sd_rst_n)
        parser_pcm_end_sd <= 1'b0;
    else
        parser_pcm_end_sd <= wav_session_started && wav_format_valid &&
            !wav_session_error && (wav_parser_error == 8'd0) &&
            (wav_file_error == 8'd0) && (wav_parser_state == 4'd0) &&
            !wav_file_busy && !wav_pcm_valid && (wav_pcm_data_length != 32'd0) &&
            (wav_samples_received == (wav_pcm_data_length >> 1));
end
always @(posedge video_clk or negedge pcm_session_rst_n) begin
    if (!pcm_session_rst_n)
        parser_pcm_end_pixel <= 3'b000;
    else
        parser_pcm_end_pixel <= {parser_pcm_end_pixel[1:0], parser_pcm_end_sd};
end

// Reserve space before exposing !busy to the unthrottled file streamer.
// Requests are never masked. Three drain cycles cover file-byte, parser PCM,
// and final FIFO-write register boundaries after the client sector_done.
wire wav_space_for_sector = (pcm_fifo_level <= 13'd3839);
wire wav_file_flow_busy = wav_file_sector_busy || wav_file_req ||
                          wav_sector_reserved || (wav_sector_drain != 0) ||
                          audio_stop_pending || wav_session_reset ||
                          (!wav_space_for_sector && !wav_session_error);
always @(posedge sd_card_clk or negedge rst_sd_n) begin
    if (!rst_sd_n) begin
        wav_sector_reserved <= 1'b0;
        wav_sector_drain <= 2'd0;
    end else begin
        if (wav_sector_drain != 0)
            wav_sector_drain <= wav_sector_drain - 1'b1;
        if (wav_file_req)
            wav_sector_reserved <= 1'b1;
        if (wav_file_sector_done) begin
            wav_sector_reserved <= 1'b0;
            wav_sector_drain <= 2'd3;
        end
    end
end

always @(posedge sd_card_clk or negedge wav_session_sd_rst_n) begin
    if (!wav_session_sd_rst_n) begin
        wav_session_started <= 1'b0;
        wav_session_index <= 2'd0;
        wav_parse_start <= 1'b0;
        wav_stream_start <= 1'b0;
        wav_session_cluster <= 32'd0;
        wav_session_size <= 32'd0;
        wav_session_spc <= 8'd0;
        wav_session_fat_lba <= 32'd0;
        wav_session_data_lba <= 32'd0;
        wav_session_max_cluster <= 32'd0;
        wav_session_error <= 1'b0;
        wav_play_enabled <= 1'b0;
        wav_samples_received <= 32'd0;
    end else begin
        wav_parse_start <= 1'b0;
        wav_stream_start <= wav_parse_start;
        if (WAV_PCM_ENABLE != 0 && !wav_session_started && active_valid_sys &&
            !audio_stop_pending && !index_commit_sys &&
            wav_resource_valid && !wav_scan_busy && audio_ready_sd[1] &&
            pcm_session_ready_sd[1]) begin
            wav_session_started <= 1'b1;
            wav_session_index <= current_index_sys;
            wav_session_cluster <= wav_start_cluster;
            wav_session_size <= wav_file_size;
            wav_session_spc <= wav_spc;
            wav_session_fat_lba <= wav_fat_lba;
            wav_session_data_lba <= wav_data_lba;
            wav_session_max_cluster <= wav_max_cluster;
            wav_parse_start <= 1'b1;
        end
        if (wav_pcm_valid && !wav_session_error && !pcm_fifo_full)
            wav_samples_received <= wav_samples_received + 1'b1;

        if (audio_stop_pending) begin
            wav_play_enabled <= 1'b0;
        end else if (wav_parser_error != 0 || wav_file_error != 0 ||
            (wav_pcm_valid && pcm_fifo_full)) begin
            wav_session_error <= 1'b1;
            wav_play_enabled <= 1'b0;
        end else if (!wav_session_error && wav_format_valid &&
                     ((pcm_fifo_level >= 13'd2048) ||
                      (wav_pcm_data_length != 0 &&
                       wav_samples_received >= (wav_pcm_data_length >> 1) &&
                       pcm_fifo_level != 0))) begin
            // A short file may start after its data payload has reached RAM.
            // This is not a playback-complete event and never advances index.
            wav_play_enabled <= 1'b1;
        end
    end
end

fat32_file_streamer u_audio_file_streamer (
    .clk(sd_card_clk), .rst_n(wav_session_sd_rst_n), .file_start(wav_stream_start),
    .first_cluster(wav_session_cluster), .file_size(wav_session_size),
    .sectors_per_cluster(wav_session_spc), .fat_start_lba(wav_session_fat_lba),
    .data_start_lba(wav_session_data_lba), .max_cluster(wav_session_max_cluster),
    .file_busy(wav_file_busy), .file_done(wav_file_done),
    .file_valid(wav_file_valid), .file_byte(wav_file_byte),
    .file_byte_offset(wav_file_offset), .fs_error(wav_file_error),
    .sector_req(wav_file_req), .sector_lba(wav_file_lba),
    .sector_busy(wav_file_flow_busy), .sector_valid(wav_file_sector_valid),
    .sector_byte(wav_file_sector_byte), .sector_byte_index(wav_file_byte_index),
    .sector_done(wav_file_sector_done), .sector_error(wav_file_sector_error)
);
wav_parser u_wav_parser (
    .clk(sd_card_clk), .rst_n(wav_session_sd_rst_n),
    .file_byte_data(wav_file_byte), .file_byte_valid(wav_file_valid),
    .file_byte_offset(wav_file_offset), .file_size(wav_session_size),
    .file_done(wav_file_done), .parse_start(wav_parse_start),
    .wav_format_valid(wav_format_valid), .format_valid(),
    .pcm_valid(wav_pcm_valid), .pcm_data(wav_pcm_data),
    .pcm_data_start(wav_pcm_data_start), .pcm_data_length(wav_pcm_data_length),
    .parser_error(wav_parser_error), .parser_state(wav_parser_state)
);
audio_pcm_fifo u_audio_pcm_fifo (
    .wr_clk(sd_card_clk), .rd_clk(video_clk), .rst_n(wav_session_sd_rst_n),
    .pcm_in_valid(wav_pcm_valid && !wav_session_error &&
                  wav_parser_error == 0 && wav_file_error == 0),
    .pcm_in_data(wav_pcm_data),
    .pcm_rd_en(pcm_rd_en), .pcm_out_valid(pcm_out_valid),
    .pcm_out_data(pcm_out_data), .fifo_empty(pcm_fifo_empty),
    .fifo_full(pcm_fifo_full), .fifo_level(pcm_fifo_level)
);
audio_sector_client u_audio_sector_client (
    .clk(sd_card_clk), .rst_n(rst_sd_n),
    .scan_sector_req(wav_scan_req), .scan_sector_lba(wav_scan_lba),
    .scan_sector_busy(wav_scan_sector_busy), .scan_sector_valid(wav_scan_sector_valid),
    .scan_sector_byte(wav_scan_byte), .scan_sector_byte_index(wav_scan_byte_index),
    .scan_sector_done(wav_scan_sector_done), .scan_sector_error(wav_scan_sector_error),
    .file_sector_req(wav_file_req), .file_sector_lba(wav_file_lba),
    .file_sector_busy(wav_file_sector_busy), .file_sector_valid(wav_file_sector_valid),
    .file_sector_byte(wav_file_sector_byte), .file_sector_byte_index(wav_file_byte_index),
    .file_sector_done(wav_file_sector_done), .file_sector_error(wav_file_sector_error),
    .audio_req(audio_req), .audio_lba(audio_lba), .audio_start(audio_start),
    .audio_data(audio_data), .audio_valid(audio_valid), .audio_done(audio_done)
);

sd_arbiter u_sd_arbiter (
    .clk(sd_card_clk), .rst(rst_sd),
    .SD_nCS(sd_ncs), .SD_DCLK(sd_dclk), .SD_MOSI(sd_mosi), .SD_MISO(sd_miso),
    .sd_init_done(sd_init_done),
    .bmp_req(bmp_req), .bmp_lba(bmp_lba), .bmp_busy(bmp_busy),
    .bmp_data(bmp_data), .bmp_valid(bmp_valid), .bmp_done(bmp_done),
    .bmp_error(bmp_error),
    .audio_req(audio_req), .audio_lba(audio_lba), .audio_start(audio_start),
    .audio_data(audio_data), .audio_valid(audio_valid), .audio_done(audio_done)
);
seg_decoder u_seg_decoder (.bin_data(state_code_sync2), .seg_data(seg_data_0));
seg_scan u_seg_scan (
    .clk(clk), .rst_n(rst_sys_n), .seg_sel(seg_sel), .seg_data(seg_data),
    .seg_data_0({1'b1,7'b1111_111}),
    .seg_data_1({1'b1,7'b1111_111}),
    .seg_data_2({1'b1,7'b1111_111}),
    .seg_data_3({1'b1,7'b1111_111}),
    .seg_data_4({1'b1,7'b1111_111}),
    .seg_data_5({1'b1,seg_data_0})
);

// Ordered pixel/start/commit/error events cross 100 -> 125 MHz.
wire mem_event_valid;
wire [1:0] mem_event_type;
wire [31:0] mem_event_data;
media_pixel_cdc u_pixel_cdc (
    .src_clk(sd_card_clk), .src_rst_n(rst_sd_n),
    .src_valid(sd_event_valid), .src_type(sd_event_type),
    .src_data(sd_event_data), .src_full(sd_event_full),
    .dst_clk(ext_mem_clk), .dst_rst_n(rst_mem_n),
    .dst_valid(mem_event_valid), .dst_type(mem_event_type),
    .dst_data(mem_event_data)
);

wire App_rd_en, App_wr_en, Sdr_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr, App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din, Sdr_rd_dout;
wire [3:0] App_wr_dm;
wire writer_burst_request, writer_burst_running;
wire fullscreen_burst_request, fullscreen_burst_running;
wire thumbnail_burst_request, thumbnail_burst_running;
wire fullscreen_request_raw, thumbnail_request_raw;
wire [1:0] focus_index_sys;
wire cache_path_active;
(* async_reg = "true" *) reg mode_mem1, mode_mem2;
(* async_reg = "true" *) reg mode_pix1, mode_pix2;
(* async_reg = "true" *) reg [1:0] focus_pix1, focus_pix2;
always @(posedge ext_mem_clk or negedge player_mem_rst_n) begin
    if (!player_mem_rst_n) begin mode_mem1<=0; mode_mem2<=0; end
    else begin mode_mem1<=cache_path_active; mode_mem2<=mode_mem1; end
end
always @(posedge video_clk or negedge player_pixel_rst_n) begin
    if (!player_pixel_rst_n) begin
        mode_pix1<=0; mode_pix2<=0; focus_pix1<=0; focus_pix2<=0;
    end else begin
        mode_pix1<=display_mode_sys; mode_pix2<=mode_pix1;
        focus_pix1<=focus_index_sys; focus_pix2<=focus_pix1;
    end
end
// Mask only new work. An accepted burst/thumbnail line retains ownership
// until all returns have drained, even when the display mode changes.
assign fullscreen_burst_request = fullscreen_request_raw && !mode_mem2;
assign thumbnail_burst_request = thumbnail_request_raw && mode_mem2;
wire reader_burst_request, reader_burst_running;
wire fullscreen_grant, thumbnail_grant, writer_grant;
// No new burst may start in the window around the controller's internal
// refresh request, nor while it reports busy/refresh. The writer and the
// thumbnail reader already check busy/refresh themselves.
wire refresh_guard_short, refresh_guard_long;
sdram_refresh_guard u_refresh_guard (
    .clk(ext_mem_clk), .rst(rst_mem), .init_done(Sdr_init_done),
    .guard_short(refresh_guard_short), .guard_long(refresh_guard_long)
);
assign thumbnail_grant = mode_mem2 && !writer_burst_running && !fullscreen_burst_running &&
                         !refresh_guard_long;
assign fullscreen_grant = !mode_mem2 && !writer_burst_running && !thumbnail_burst_running &&
                          !thumbnail_burst_request && !refresh_guard_short &&
                          !Sdr_busy && !Sdr_init_ref_vld;
assign writer_grant = !fullscreen_burst_running && !thumbnail_burst_running &&
                      !fullscreen_burst_request && !thumbnail_burst_request &&
                      !refresh_guard_short;
media_frame_loader #(.ADDR_WIDTH(ADDR_BITS), .IMAGE_WORDS(IMAGE_WORDS))
u_loader (
    .clk(ext_mem_clk), .rst_n(rst_mem_n),
    .load_start(mem_event_valid && mem_event_type==2'b00),
    .image_index(mem_event_data[1:0]),
    .pixel_valid(mem_event_valid && mem_event_type==2'b01),
    .pixel_rgb(mem_event_data[23:0]),
    .frame_commit(mem_event_valid && mem_event_type==2'b10),
    .decode_error(mem_event_valid && mem_event_type==2'b11 ?
                  mem_event_data[7:0] : 8'd0),
    .load_busy(), .load_done(loader_done),
    .image_complete(image_complete_mem), .memory_error(loader_error),
    .burst_grant(writer_grant), .burst_request(writer_burst_request),
    .burst_running(writer_burst_running),
    .controller_init_done(Sdr_init_done), .controller_busy(Sdr_busy),
    .refresh_active(Sdr_init_ref_vld),
    .app_wr_en(App_wr_en), .app_wr_addr(App_wr_addr),
    .app_wr_mask(App_wr_dm), .app_wr_data(App_wr_din)
);

// Stable error bus precedes the synchronized completion toggle at the SD end.
always @(posedge ext_mem_clk or negedge rst_mem_n) begin
    if (!rst_mem_n) begin
        loader_result_toggle<=0; loader_result_error<=0;
    end else if(loader_done) begin
        loader_result_toggle<=~loader_result_toggle;
        loader_result_error<=loader_error;
    end
end
(* async_reg = "true" *) reg [3:0] image_sync1, image_sync2;
always @(posedge sd_card_clk or negedge rst_sd_n) begin
    if(!rst_sd_n) begin image_sync1<=0; image_sync2<=0; end
    else begin image_sync1<=image_complete_mem; image_sync2<=image_sync1; end
end

// The loader owns image_complete_mem in the ext_mem_clk domain.  Keep the
// legacy SD-domain synchronizer above for player control, but use a separate
// two-flop synchronizer for all video consumers.  The first stage is an
// implementation-only CDC register and must not drive functional logic.
(* async_reg = "true" *) reg [3:0] image_sync1_video, image_sync2_video;
always @(posedge video_clk or negedge player_pixel_rst_n) begin
    if (!player_pixel_rst_n) begin
        image_sync1_video <= 4'd0;
        image_sync2_video <= 4'd0;
    end else begin
        image_sync1_video <= image_complete_mem;
        image_sync2_video <= image_sync1_video;
    end
end

wire raw_hs, raw_vs, raw_de;
wire [11:0] raw_x, raw_y;
video_timing_ctrl #(
    .H_ACT(VIDEO_H_ACT), .H_FP(VIDEO_H_FP),
    .H_SYNC(VIDEO_H_SYNC), .H_BP(VIDEO_H_BP), .ACTIVE_LAST(1)
) u_vtc (
    .pclk(video_clk), .rst_n(rst_pixel_n),
    .hsync(raw_hs), .vsync(raw_vs), .de(raw_de),
    .x(raw_x), .y(raw_y)
);
reg raw_vs_q;
always @(posedge video_clk or negedge rst_pixel_n) begin
    if(!rst_pixel_n) raw_vs_q<=1'b1;
    else raw_vs_q<=raw_vs;
end
wire frame_start = raw_vs_q && !raw_vs;
wire [1:0] active_index;
wire active_valid;
wire frame_ready;
wire index_commit;
wire display_mode;
wire zoom_active;
wire [4:0] zoom_step;
wire [1:0] zoom_index;
assign cache_path_active = display_mode || zoom_active;
wire visual_busy = zoom_active || (display_mode && !mode_pix2);
wire [1:0] focus_index = focus_pix2;
wire [7:0] control_error_sd;
assign state_code_sd = load_state_code >= 4'd8 ? load_state_code :
                       active_valid_sys ? {2'd0,current_index_sys}+4'd1 :
                       load_state_code;
wire [7:0] transition_alpha;
wire transition_active, wipe_active;
wire [1:0] old_index;
wire [11:0] wipe_boundary;
media_player_control #(.DEFAULT_PERIOD_CYCLES(32'd300000000),
    .AUTO_GUIDE_DEFAULT(AUTO_GUIDE_DEFAULT),
    .FADE_FRAMES(FADE_FRAMES), .IMAGE_WIDTH(VIDEO_H_ACT))
u_player (
    .sys_clk(sd_card_clk), .sys_rst_n(player_sd_rst_n),
    // Route each short/combo gesture once, through the registered requests.
    .next_event(focus_next_req && !display_mode_sys), .prev_event(1'b0),
    .toggle_auto_event(select_req && !display_mode_sys),
    .key1_short_event(1'b0),
    .key2_short_event(1'b0),
    .key2_long_event(key2_long_event),
    .combo_long_event(1'b0),
    .mode_toggle_req(mode_toggle_req),
    .focus_next_req(focus_next_req),
    .focus_prev_req(focus_prev_req),
    .select_req(select_req),
    .period_cfg_valid(1'b0), .period_cfg_cycles(32'd0),
    .auto_cfg_valid(1'b0), .auto_cfg_enabled(1'b0),
    .auto_next_req(auto_next_req), .auto_next_index(auto_next_index),
    .audio_switch_ready(audio_switch_ready),
    .image_complete(image_sync2),
    .auto_enabled(auto_enabled_sd), .active_period_cycles(), .pending_index(),
    .control_error(control_error_sd), .request_busy(player_request_busy),
    .switch_pending(player_switch_pending),
    .index_commit_sys(index_commit_sys),
    .auto_next_accepted(auto_next_accepted), .auto_request_ready(auto_request_ready),
    .current_index_sys(current_index_sys),
    .active_valid_sys(active_valid_sys),
    .display_mode(display_mode_sys), .focus_index(focus_index_sys),
    .pix_clk(video_clk), .pix_rst_n(player_pixel_rst_n),
    .frame_start(frame_start), .visual_busy(visual_busy), .active_index(active_index),
    .active_valid(active_valid), .frame_ready(frame_ready),
    .index_commit(index_commit), .transition_alpha(transition_alpha),
    .transition_active(transition_active), .wipe_active(wipe_active),
    .old_index(old_index), .wipe_boundary(wipe_boundary)
);
// thumbnail_layout has no separate DE input; use out-of-range coordinates
// during HDMI blanking so blanking pixels cannot look like (0,0).
wire [9:0] active_x = raw_de ? raw_x - VIDEO_H_BLANK : 10'd640;
wire [9:0] active_y = raw_de ? raw_y - VIDEO_V_BLANK : 10'd480;
wire [1:0] thumbnail_source_index;
wire [9:0] thumbnail_source_x, thumbnail_source_y;
wire thumbnail_de, thumbnail_focus_border;
wire demand_valid, prefetch_valid;
wire [1:0] demand_index, prefetch_index;
wire [9:0] demand_row, prefetch_row;
wire [9:0] scan_y = (raw_y >= VIDEO_V_BLANK && raw_y < VIDEO_V_BLANK + 480) ?
                     raw_y - VIDEO_V_BLANK : 10'd480;
thumbnail_layout u_thumbnail_layout (
    .active_x(active_x), .active_y(active_y),
    .focus_index(focus_index), .image_complete(image_sync2_video),
    .display_mode(mode_pix2), .active_index(active_index),
    .video_clk(video_clk), .rst_n(player_pixel_rst_n),
    .frame_start(frame_start), .line_start(raw_x == 12'd0),
    .scan_x(raw_x[9:0]), .scan_y(scan_y),
    .mode_frame(display_mode), .zoom_active(zoom_active),
    .zoom_step(zoom_step), .zoom_index(zoom_index),
    .demand_valid(demand_valid), .demand_index(demand_index), .demand_row(demand_row),
    .prefetch_valid(prefetch_valid), .prefetch_index(prefetch_index), .prefetch_row(prefetch_row),
    .source_index(thumbnail_source_index), .source_x(thumbnail_source_x),
    .source_y(thumbnail_source_y), .thumb_de(thumbnail_de),
    .focus_border(thumbnail_focus_border)
);
wire [23:0] pixel_rgb;
wire pixel_valid, reader_underflow;
wire [7:0] reader_error;
wire [ADDR_BITS-1:0] fullscreen_app_rd_addr, thumbnail_app_rd_addr;
wire fullscreen_app_rd_en, thumbnail_app_rd_en;
wire fullscreen_sdram_rd_valid, thumbnail_sdram_rd_valid;
wire [23:0] thumbnail_reader_rgb;
wire thumbnail_reader_valid;
wire thumbnail_underflow;
wire [31:0] thumbnail_underflow_count, thumbnail_burst_count;
wire [7:0] thumbnail_reader_error;
media_transition_reader #(.ADDR_WIDTH(ADDR_BITS), .IMAGE_WORDS(IMAGE_WORDS),
    .IMAGE_WIDTH(VIDEO_H_ACT))
u_reader (
    .mem_clk(ext_mem_clk), .mem_rst_n(player_mem_rst_n),
    .controller_init_done(Sdr_init_done), .controller_busy(Sdr_busy),
    .refresh_active(Sdr_init_ref_vld),
    .burst_grant(fullscreen_grant), .burst_request(fullscreen_request_raw),
    .burst_running(fullscreen_burst_running),
    .app_rd_en(fullscreen_app_rd_en), .app_rd_addr(fullscreen_app_rd_addr),
    .sdram_rd_valid(fullscreen_sdram_rd_valid), .sdram_rd_data(Sdr_rd_dout),
    .pix_clk(video_clk), .pix_rst_n(player_pixel_rst_n),
    .frame_start(frame_start && frame_ready && !mode_pix2 &&
                 !display_mode && (!zoom_active || zoom_step == 5'd16)),
    .active_index(active_index), .old_index(old_index),
    .wipe_active(wipe_active), .wipe_boundary(wipe_boundary),
    .pixel_consume(!cache_path_active && raw_de && active_valid),
    .pixel_rgb(pixel_rgb), .pixel_valid(pixel_valid),
    .underflow(reader_underflow),
    .reader_busy(), .frame_read_done(), .reader_index(),
    .reader_error(reader_error),
    .display_mode(1'b0), .source_index(2'd0),
    .source_x(10'd0), .source_y(10'd0),
    .image_complete(4'd0)
);
thumbnail_reader #(.ADDR_WIDTH(ADDR_BITS), .IMAGE_WORDS(IMAGE_WORDS),
    .IMAGE_WIDTH(VIDEO_H_ACT), .LINE_WORDS(VIDEO_H_ACT), .BURST_SIZE(128))
u_thumbnail_reader (
    .mem_clk(ext_mem_clk), .mem_rst_n(player_mem_rst_n),
    .controller_init_done(Sdr_init_done), .controller_busy(Sdr_busy),
    .refresh_active(Sdr_init_ref_vld), .burst_grant(thumbnail_grant),
    .burst_request(thumbnail_request_raw), .burst_running(thumbnail_burst_running),
    .app_rd_en(thumbnail_app_rd_en), .app_rd_addr(thumbnail_app_rd_addr),
    .sdram_rd_valid(thumbnail_sdram_rd_valid), .sdram_rd_data(Sdr_rd_dout),
    .pix_clk(video_clk), .pix_rst_n(player_pixel_rst_n),
    .source_index(thumbnail_source_index), .source_x(thumbnail_source_x),
    .source_y(thumbnail_source_y), .thumbnail_de(cache_path_active && raw_de && thumbnail_de),
    .reader_enable(cache_path_active),
    .demand_valid(demand_valid), .demand_index(demand_index), .demand_row(demand_row),
    .prefetch_valid(prefetch_valid), .prefetch_index(prefetch_index), .prefetch_row(prefetch_row),
    .image_complete(image_sync2_video), .pixel_x(active_x), .pixel_y(active_y),
    .thumbnail_rgb(thumbnail_reader_rgb), .thumbnail_valid(thumbnail_reader_valid),
    .underflow(thumbnail_underflow), .underflow_count(thumbnail_underflow_count),
    .burst_count(thumbnail_burst_count), .reader_error(thumbnail_reader_error)
);
assign reader_burst_request = fullscreen_burst_request || thumbnail_burst_request;
assign reader_burst_running = fullscreen_burst_running || thumbnail_burst_running;
assign App_rd_en = thumbnail_app_rd_en || fullscreen_app_rd_en;
assign App_rd_addr = thumbnail_app_rd_en ? thumbnail_app_rd_addr : fullscreen_app_rd_addr;
assign fullscreen_sdram_rd_valid = Sdr_rd_en && fullscreen_burst_running &&
                                   !thumbnail_burst_running;
assign thumbnail_sdram_rd_valid = Sdr_rd_en && thumbnail_burst_running;
wire hs, vs, de;
wire [23:0] vout_data;
wire auto_enabled_pix;
wire [1:0] osd_focus_index;
wire [7:0] osd_error_code;
wire [7:0] combined_reader_error = (reader_error != 8'd0) ? reader_error :
                                   thumbnail_reader_error;
wire combined_reader_underflow = reader_underflow || thumbnail_underflow;
osd_status_bridge u_osd_status (
    .sd_clk(sd_card_clk), .sd_rst_n(rst_sd_n),
    .auto_enabled_sd(auto_enabled_sd), .focus_index_sd(focus_index_sys),
    .control_error_sd(control_error_sd),
    .load_state_sd(load_state_code),
    .mem_clk(ext_mem_clk), .mem_rst_n(rst_mem_n),
    .loader_error_mem(loader_error), .reader_error_mem(combined_reader_error),
    .pix_clk(video_clk), .pix_rst_n(rst_pixel_n),
    .reader_underflow_pix(combined_reader_underflow),
    .auto_enabled_pix(auto_enabled_pix), .focus_index_pix(osd_focus_index),
    .error_code_pix(osd_error_code)
);
wire [11:0] osd_active_x = raw_de ? raw_x - VIDEO_H_BLANK : 12'd0;
// An unavailable source produces black; a previous bank/row pixel must never
// stand in for an unready cache or a fullscreen FIFO being reloaded.
// Test pattern, rows 0..239: white, yellow, cyan, green, magenta, red, blue,
// black bars, 80 pixels each; rows 240..359 gray ramp; 360..399 red ramp;
// 400..439 green ramp; 440..479 blue ramp. ramp = floor(x * 51 / 128).
wire [15:0] test_x51 = active_x * 6'd51;
wire [7:0] test_ramp = test_x51[14:7];
reg [23:0] test_rgb;
always @* begin
    if (active_y < 10'd240) begin
        if (active_x < 10'd80)       test_rgb = 24'hffffff;
        else if (active_x < 10'd160) test_rgb = 24'hffff00;
        else if (active_x < 10'd240) test_rgb = 24'h00ffff;
        else if (active_x < 10'd320) test_rgb = 24'h00ff00;
        else if (active_x < 10'd400) test_rgb = 24'hff00ff;
        else if (active_x < 10'd480) test_rgb = 24'hff0000;
        else if (active_x < 10'd560) test_rgb = 24'h0000ff;
        else                         test_rgb = 24'h000000;
    end else if (active_y < 10'd360) test_rgb = {3{test_ramp}};
    else if (active_y < 10'd400)     test_rgb = {test_ramp, 16'd0};
    else if (active_y < 10'd440)     test_rgb = {8'd0, test_ramp, 8'd0};
    else                             test_rgb = {16'd0, test_ramp};
end
wire [23:0] fullscreen_rgb = (VIDEO_TEST_PATTERN != 0) ? test_rgb :
                             (active_valid && pixel_valid) ? pixel_rgb : 24'd0;
// Keep the reader -> transition -> adjustment -> OSD -> waveform -> adapter order.
// Preview uses unity gain; fullscreen retains the original fade contract.
// The focus border does not depend on a reader response being available.
wire [7:0] display_alpha = cache_path_active ? 8'hff : transition_alpha;
reg [7:0] brightness_pending, contrast_pending;
reg [7:0] brightness_active, contrast_active;
// Cut the coordinate/layout/cache-read chain before fade arithmetic. Capture
// the entire pixel transaction, including frame-committed adjustment values.
// The synchronous thumbnail RAM supplies this transaction's RGB register;
// register the other sources and selectors here without adding a pixel stage.
reg [23:0] fullscreen_rgb_q;
reg cache_path_active_q, thumbnail_focus_border_q;
wire [23:0] reader_rgb_q = cache_path_active_q ?
    (thumbnail_focus_border_q ? 24'hffff00 :
     (thumbnail_reader_valid ? thumbnail_reader_rgb : 24'd0)) :
    fullscreen_rgb_q;
reg [7:0] display_alpha_q, display_brightness_q, display_contrast_q;
reg display_de_q, display_hs_q, display_vs_q, display_active_valid_q;
reg [11:0] display_pixel_x_q, display_pixel_y_q;
always @(posedge video_clk or negedge rst_pixel_n) begin
    if (!rst_pixel_n) begin
        fullscreen_rgb_q <= 24'd0;
        cache_path_active_q <= 1'b0;
        thumbnail_focus_border_q <= 1'b0;
        display_alpha_q <= 8'd0;
        display_brightness_q <= 8'd255;
        display_contrast_q <= 8'd128;
        display_de_q <= 1'b0;
        display_hs_q <= 1'b0;
        display_vs_q <= 1'b0;
        display_active_valid_q <= 1'b0;
        display_pixel_x_q <= 12'd0;
        display_pixel_y_q <= 12'd0;
    end else begin
        fullscreen_rgb_q <= fullscreen_rgb;
        cache_path_active_q <= cache_path_active;
        thumbnail_focus_border_q <= raw_de && thumbnail_focus_border;
        display_alpha_q <= display_alpha;
        display_brightness_q <= brightness_active;
        display_contrast_q <= contrast_active;
        display_de_q <= raw_de;
        display_hs_q <= raw_hs;
        display_vs_q <= raw_vs;
        display_active_valid_q <= active_valid;
        display_pixel_x_q <= osd_active_x;
        display_pixel_y_q <= raw_de ? raw_y - VIDEO_V_BLANK : 12'd480;
    end
end
wire [23:0] pre_osd_rgb;
transition_blend u_transition_blend (
    .rgb_in(reader_rgb_q), .alpha(display_alpha_q), .rgb_out(pre_osd_rgb)
);
wire [23:0] adjusted_rgb;
wire adjusted_de, adjusted_hs, adjusted_vs;
wire [7:0] brightness_cfg;
wire [7:0] contrast_cfg;
// Synchronize each physical switch before the unchanged parameter decoder.
// Do not cross the combinationally decoded eight-bit parameter buses.
(* async_reg = "true" *) reg [3:0] bc_sw_meta, bc_sw_sync;
always @(posedge video_clk or negedge rst_pixel_n) begin
    if (!rst_pixel_n) begin
        bc_sw_meta <= 4'b0000;
        bc_sw_sync <= 4'b0000;
    end else begin
        bc_sw_meta <= {sw1, sw2, sw3, sw4};
        bc_sw_sync <= bc_sw_meta;
    end
end
brightness_contrast_control u_brightness_contrast_control (
    .sw1(bc_sw_sync[3]), .sw2(bc_sw_sync[2]),
    .sw3(bc_sw_sync[1]), .sw4(bc_sw_sync[0]),
    .brightness_cfg(brightness_cfg), .contrast_cfg(contrast_cfg)
);
// Capture a parameter pair every pixel clock; commit the previous pending
// pair together only at the existing raw-video frame boundary.
always @(posedge video_clk or negedge rst_pixel_n) begin
    if (!rst_pixel_n) begin
        {brightness_pending, contrast_pending} <= {8'd255, 8'd128};
        {brightness_active, contrast_active} <= {8'd255, 8'd128};
    end else begin
        {brightness_pending, contrast_pending} <=
            {brightness_cfg, contrast_cfg};
        if (frame_start)
            {brightness_active, contrast_active} <=
                {brightness_pending, contrast_pending};
    end
end
brightness_contrast_adjust u_brightness_contrast_adjust (
    .clk(video_clk), .rst_n(rst_pixel_n),
    .in_de(display_de_q), .in_hs(display_hs_q), .in_vs(display_vs_q),
    .rgb_in(pre_osd_rgb),
    .brightness(display_brightness_q), .contrast(display_contrast_q),
    .out_de(adjusted_de), .out_hs(adjusted_hs), .out_vs(adjusted_vs),
    .rgb_out(adjusted_rgb)
);
// Match the OSD coordinate and source-valid sideband to the adjustment stage's
// four-cycle RGB/timing pipeline after the reader boundary register.
// Coordinates use the same registered bundle as the adjustment input, so stage 4 identifies
// the pixel represented by adjusted_rgb/adjusted_de/adjusted_hs/adjusted_vs.
reg [11:0] pixel_x_s1, pixel_x_s2, pixel_x_s3, adjusted_pixel_x;
reg [11:0] pixel_y_s1, pixel_y_s2, pixel_y_s3, adjusted_pixel_y;
reg active_valid_s1, active_valid_s2, active_valid_s3, adjusted_active_valid;
always @(posedge video_clk or negedge rst_pixel_n) begin
    if (!rst_pixel_n) begin
        pixel_x_s1 <= 12'd0;
        pixel_x_s2 <= 12'd0;
        pixel_x_s3 <= 12'd0;
        adjusted_pixel_x <= 12'd0;
        pixel_y_s1 <= 12'd0;
        pixel_y_s2 <= 12'd0;
        pixel_y_s3 <= 12'd0;
        adjusted_pixel_y <= 12'd0;
        active_valid_s1 <= 1'b0;
        active_valid_s2 <= 1'b0;
        active_valid_s3 <= 1'b0;
        adjusted_active_valid <= 1'b0;
    end else begin
        pixel_x_s1 <= display_pixel_x_q;
        pixel_x_s2 <= pixel_x_s1;
        pixel_x_s3 <= pixel_x_s2;
        adjusted_pixel_x <= pixel_x_s3;
        pixel_y_s1 <= display_pixel_y_q;
        pixel_y_s2 <= pixel_y_s1;
        pixel_y_s3 <= pixel_y_s2;
        adjusted_pixel_y <= pixel_y_s3;
        active_valid_s1 <= display_active_valid_q;
        active_valid_s2 <= active_valid_s1;
        active_valid_s3 <= active_valid_s2;
        adjusted_active_valid <= active_valid_s3;
    end
end
// Chinese OSD after brightness/contrast, so dimming the exhibit never dims the
// text. Its glyph lookup starts from the bundle entering the adjustment stage
// (display_*_q), exactly four clocks ahead of adjusted_rgb, so the overlay adds
// no latency and the OSD -> waveform -> adapter order and timing are unchanged.
// All status below is in video_clk and is sampled by the OSD at its own frame
// start, one clock after the commit edge, inside vertical blanking;
// brightness_active/contrast_active are the registers the adjustment stage
// applies to that same frame.
// A wipe shows the outgoing bank over most of the screen, so its caption stays
// on old_index until the wipe ends. A manual fade commits active_index after a
// full black frame, when the new bank is the only picture being displayed.
wire [1:0] osd_caption_index = wipe_active ? old_index : active_index;
// The thumbnail focus, or the bank zooming back to fullscreen, is reported as
// a selection and never as the current fullscreen picture.
wire [1:0] osd_select_index = zoom_active ? zoom_index : osd_focus_index;
// The caption sits WAVE_GAP pixels plus its dark-bar margin above the waveform.
localparam integer WAVE_X = 64;
localparam integer WAVE_Y = 400;
localparam integer WAVE_GAP = 8;
localparam integer OSD_BAR_PAD = 4;
wire [23:0] osd_rgb_out;
wire osd_active;
cn_osd_overlay #(
    .TOP_X(16), .TOP_Y_FULL(16), .TOP_Y_THUMB(256), .LINE_PITCH(40),
    .CAPTION_X(WAVE_X), .CAPTION_Y(WAVE_Y - WAVE_GAP - OSD_BAR_PAD - 32),
    .BAR_PAD(OSD_BAR_PAD),
    .TEXT_RGB(24'hffffff), .CAPTION_RGB(24'hffe680),
    .ALERT_RGB(24'hff5050), .BAR_RGB(24'h10141c)
) u_osd_overlay (
    .clk(video_clk), .rst_n(rst_pixel_n),
    .lead_vsync(display_vs_q), .lead_de(display_de_q),
    .lead_x(display_pixel_x_q[9:0]), .lead_y(display_pixel_y_q),
    .rgb_in(adjusted_rgb),
    .thumb_view(cache_path_active), .auto_enabled(auto_enabled_pix),
    .image_valid(active_valid), .image_index(osd_caption_index),
    .select_index(osd_select_index),
    .brightness(brightness_active), .contrast(contrast_active),
    .error_code(osd_error_code),
    .rgb_out(osd_rgb_out), .osd_active(osd_active)
);
wire pcm_tap_valid;
wire signed [15:0] pcm_tap_left;
wire [7:0] volume_level;
wire [23:0] waveform_rgb_out;
promax_audio_waveform #(.WAVE_X(WAVE_X), .WAVE_Y(WAVE_Y)) u_audio_waveform (
    .clk(video_clk), .rst_n(rst_pixel_n),
    .vsync(adjusted_vs), .de(adjusted_de), .rgb_in(osd_rgb_out),
    .pixel_x(adjusted_pixel_x[9:0]), .enable(adjusted_active_valid),
    .volume_level(volume_level),
    .pcm_tap_valid(pcm_tap_valid), .pcm_tap_left(pcm_tap_left),
    .rgb_out(waveform_rgb_out), .waveform_active()
);
rgb_video_adapter u_video_adapter (
    .pclk(video_clk), .rst_n(rst_pixel_n),
    .hs_in(adjusted_hs), .vs_in(adjusted_vs), .de_in(adjusted_de),
    .rgb_in(waveform_rgb_out),
    .video_clk(), .video_hs(hs), .video_vs(vs),
    .video_de(de), .video_rgb(vout_data)
);

// Existing encrypted SDRAM controller and physical model remain unchanged.
sdram U3 (
    .Clk(ext_mem_clk), .Clk_sft(ext_mem_clk_sft), .Rst(rst_mem),
    .Sdr_init_done(Sdr_init_done), .Sdr_init_ref_vld(Sdr_init_ref_vld),
    .Sdr_busy(Sdr_busy), .App_wr_en(App_wr_en),
    .App_wr_addr(App_wr_addr), .App_wr_dm(App_wr_dm),
    .App_wr_din(App_wr_din), .App_rd_en(App_rd_en),
    .App_rd_addr(App_rd_addr), .Sdr_rd_en(Sdr_rd_en),
    .Sdr_rd_dout(Sdr_rd_dout)
);

// Retain the existing lesson-3 audio/TMDS path, with the committed bank as
// the image-dependent tone ID. RGB/DE/HS/VS are registered as one bundle.
wire [9:0] tmds_ch0_data, tmds_ch1_data, tmds_ch2_data;
wire [9:0] tmds_clk_data = 10'b1111100000;
wire [9:0] video_x;
wire x_locked;
lesson3_video_x u_video_x (
    .clk(video_clk), .rst(rst_pixel), .hs(hs),
    .video_x(video_x), .locked(x_locked)
);
wire sequence_error, tone_overflow, pcm_contract_error;
wire blip_active;
wire volume_key_event;
wire volume_init_valid, volume_init_applied;
wire [7:0] volume_init_value;
volume_flash_store u_volume_flash_store (
    .clk(video_clk), .rst_n(rst_pixel_n),
    .enable(player_enable && audio_pixel_rst_n), .volume_level(volume_level),
    .volume_init_valid(volume_init_valid), .volume_init_value(volume_init_value),
    .volume_init_applied(volume_init_applied),
    .saved_volume(), .init_done(), .save_busy(), .save_done(), .store_error(),
    .FLASH_CS(FLASH_CS), .FLASH_SCLK(FLASH_SCLK),
    .FLASH_SDI(FLASH_SDI), .FLASH_SDO(FLASH_SDO),
    .FLASH_WP(FLASH_WP), .FLASH_HOLD(FLASH_HOLD)
);
key_press_debounce #(.CLK_FREQ_HZ(25_000_000), .DEBOUNCE_MS(50)) u_key4 (
    .clk(video_clk), .rst(rst_pixel), .button_in(key_volume),
    .press_pulse(volume_key_event), .button_stable_out()
);
// Video-X lock loss asserts immediately; recovery releases on pixel edges.
reset_sync u_rst_audio_pixel (
    .clk(video_clk), .ext_rst_n(player_pixel_rst_n && x_locked),
    .rst_n_sync(audio_pixel_rst_n)
);
media_hdmi_audio_bridge u_audio_bridge (
    .pixel_clk(video_clk), .pixel_rst_n(audio_pixel_rst_n),
    .pcm_rst_n(pcm_queue_rst_n), .wav_active(wav_enable_pixel[1]),
    .audio_session_rst_n(pcm_session_rst_n),
    .fifo_empty(pcm_fifo_empty), .pcm_out_valid(pcm_out_valid),
    .pcm_out_data(pcm_out_data), .pcm_rd_en(pcm_rd_en),
    .wav_underflow(wav_underflow), .wav_output_stall(wav_output_stall),
    .parser_pcm_end(parser_pcm_end_pixel[2]), .wav_done(wav_done),
    .audio_session_tail_drained(audio_session_tail_drained),
    .video_hsync(hs), .video_vsync(vs), .video_de(de),
    .video_x(video_x), .video_rgb(vout_data),
    .media_id(active_index), .volume(active_valid ? 8'd128 : 8'd0),
    .volume_key_event(volume_key_event), .volume_level(volume_level),
    .volume_init_valid(volume_init_valid), .volume_init_value(volume_init_value),
    .volume_init_applied(volume_init_applied),
    .fade_alpha(transition_alpha), .fade_frame_start(frame_start),
    .channel0_symbol(tmds_ch0_data),
    .channel1_symbol(tmds_ch1_data),
    .channel2_symbol(tmds_ch2_data),
    .sequence_error(sequence_error), .tone_overflow(tone_overflow),
    .pcm_contract_error(pcm_contract_error),
    .blip_active(blip_active), .pcm_tap_valid(pcm_tap_valid),
    .pcm_tap_left(pcm_tap_left)
);
assign HDMI_DDC_SCL = 1'b1;
assign HDMI_DDC_SDA = 1'bz;
hdmi_phy_wrapper #(.DEVICE("EG")) u_hdmi2phy_wrapper (
    .I_pixel_clk(video_clk), .I_serial_clk(hdmi_5x_clk),
    // The existing PHY shares one reset across pixel and serial logic.
    // Keep pixel-synchronous release until its interface can split resets.
    .I_rst(rst_pixel), .I_tmds_channel_0(tmds_ch0_data),
    .I_tmds_channel_1(tmds_ch1_data), .I_tmds_channel_2(tmds_ch2_data),
    .I_tmds_channel_clk(tmds_clk_data),
    .O_tmds_ch0_p(HDMI_D0_P), .O_tmds_ch1_p(HDMI_D1_P),
    .O_tmds_ch2_p(HDMI_D2_P), .O_tmds_clk_p(HDMI_CLK_P)
);
endmodule
