`timescale 1ns/1ps

module media_player_control #(
    parameter integer FADE_FRAMES = 0,
    parameter integer IMAGE_WIDTH = 640,
    parameter integer AUTO_GUIDE_DEFAULT = 0,
    parameter [31:0] DEFAULT_PERIOD_CYCLES = 32'd250000000,
    parameter [31:0] MIN_PERIOD_CYCLES = 32'd50000000,
    parameter [31:0] MAX_PERIOD_CYCLES = 32'd3000000000
) (
    input  wire        sys_clk,
    input  wire        sys_rst_n,
    input  wire        next_event,
    input  wire        prev_event,
    input  wire        toggle_auto_event,
    input  wire        key1_short_event,
    input  wire        key2_short_event,
    input  wire        key2_long_event,
    input  wire        combo_long_event,
    // One-clock request pulses synchronous to sys_clk.
    input  wire        mode_toggle_req,
    input  wire        focus_next_req,
    input  wire        focus_prev_req,
    input  wire        select_req,
    input  wire        period_cfg_valid,
    input  wire [31:0] period_cfg_cycles,
    input  wire        auto_cfg_valid,
    input  wire        auto_cfg_enabled,
    input  wire        auto_next_req,
    input  wire [1:0]  auto_next_index,
    input  wire        audio_switch_ready,
    input  wire [3:0]  image_complete,
    output reg         auto_enabled,
    output reg  [31:0] active_period_cycles,
    output reg  [1:0]  pending_index,
    output reg  [7:0]  control_error,
    output wire        request_busy,
    output wire [1:0]  current_index_sys,
    output wire        active_valid_sys,
    output reg         index_commit_sys,
    output reg         auto_next_accepted,
    output wire        auto_request_ready,
    output wire        switch_pending,
    output reg         display_mode,
    output reg  [1:0]  focus_index,
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        frame_start,
    input  wire        visual_busy,
    output wire [1:0]  active_index,
    output wire        active_valid,
    output wire        frame_ready,
    output wire        index_commit,
    output wire [7:0]  transition_alpha,
    output wire        transition_active,
    output wire        wipe_active,
    output wire [1:0]  old_index,
    output wire [11:0] wipe_boundary
);
    localparam ERR_NONE = 8'h00;
    localparam ERR_PERIOD_RANGE = 8'h51;
    localparam ERR_REQUEST_BUSY = 8'h52;
    localparam ERR_NO_RESOURCE = 8'h53;

    reg [1:0] sys_current_index;
    reg sys_active_valid;
    reg auto_enabled_saved;
    reg mode_request_pending, mode_request_value;
    reg enter_wait_transition, exit_wait_transition;
    reg [1:0] exit_target_index;
    reg request_busy_d;
    (* async_reg = "true" *) reg visual_busy_sys1, visual_busy_sys2;
    (* async_reg = "true" *) reg audio_ready_pix1, audio_ready_pix2;
    reg frame_toggle_pix;
    (* async_reg = "true" *) reg frame_sync1, frame_sync2;
    reg frame_seen_sys;
    reg request_ever;
    reg request_toggle;
    reg ack_toggle_pix;
    // Separate commit notification: status follows the displayed index,
    // while the request ACK remains withheld until the effect is complete.
    reg commit_toggle_pix;
    (* async_reg = "true" *) reg commit_sync1, commit_sync2;
    (* async_reg = "true" *) reg [1:0] displayed_sync1, displayed_sync2;
    reg commit_seen_sys;
    (* async_reg = "true" *) reg ack_sync1, ack_sync2;
    (* async_reg = "true" *) reg [1:0] index_sync1, index_sync2;
    (* async_reg = "true" *) reg req_sync1, req_sync2;
    reg req_seen_pix;
    reg [1:0] requested_index_pix;
    reg pending_manual, requested_manual_pix;
    (* async_reg = "true" *) reg manual_sync1, manual_sync2;
    reg capture_pending_pix;
    wire transition_done;
    reg commit_pending_pix;
    assign frame_ready = active_valid || commit_pending_pix;

    assign request_busy = request_toggle != ack_sync2;
    assign current_index_sys = sys_current_index;
    assign active_valid_sys = sys_active_valid;
    wire key1_short = (key1_short_event === 1'b1);
    wire key2_short = (key2_short_event === 1'b1);
    wire key2_long  = (key2_long_event  === 1'b1);
    wire combo_long = (combo_long_event === 1'b1);
    wire mode_req = combo_long || (mode_toggle_req === 1'b1);
    wire focus_forward_req = key1_short || (focus_next_req === 1'b1);
    wire focus_backward_req = (focus_prev_req === 1'b1);
    wire focus_select_req = key2_short || (select_req === 1'b1);
    wire next_req = next_event || key1_short;
    wire prev_req = prev_event || key2_long;
    wire auto_req = toggle_auto_event || key2_short;
    wire frame_tick_sys = frame_sync2 != frame_seen_sys;
    assign switch_pending = request_busy || mode_request_pending ||
                            enter_wait_transition || exit_wait_transition ||
                            visual_busy_sys2;
    assign auto_request_ready = auto_enabled && sys_active_valid &&
        !display_mode && !switch_pending && !mode_req && !next_req &&
        !prev_req && !auto_req && !auto_cfg_valid && !period_cfg_valid &&
        image_complete[auto_next_index];

    function [2:0] next_available;
        input [1:0] current;
        input [3:0] complete;
        integer step;
        reg [1:0] candidate;
        begin
            next_available = {1'b0, current};
            for (step = 1; step <= 3; step = step + 1) begin
                candidate = current + step[1:0];
                if (!next_available[2] && complete[candidate])
                    next_available = {1'b1, candidate};
            end
        end
    endfunction

    function [2:0] prev_available;
        input [1:0] current;
        input [3:0] complete;
        integer step;
        reg [1:0] candidate;
        begin
            prev_available = {1'b0, current};
            for (step = 1; step <= 3; step = step + 1) begin
                candidate = current - step[1:0];
                if (!prev_available[2] && complete[candidate])
                    prev_available = {1'b1, candidate};
            end
        end
    endfunction

    task issue_next_request;
        input manual_request;
        reg [2:0] selected;
        begin
            selected = !sys_active_valid ?
                (image_complete[0] ? 3'b100 : image_complete[1] ? 3'b101 :
                 image_complete[2] ? 3'b110 : image_complete[3] ? 3'b111 : 3'b000) :
                next_available(pending_index, image_complete);
            if (request_busy) begin
                control_error <= ERR_REQUEST_BUSY;
            end else if (!selected[2]) begin
                if (!image_complete[pending_index]) control_error <= ERR_NO_RESOURCE;
            end else begin
                pending_index <= selected[1:0];
                pending_manual <= manual_request;
                request_toggle <= ~request_toggle;
                request_ever <= 1'b1;
            end
        end
    endtask

    task issue_prev_request;
        reg [2:0] selected;
        begin
            selected = prev_available(pending_index, image_complete);
            if (request_busy)
                control_error <= ERR_REQUEST_BUSY;
            else if (!selected[2]) begin
                if (!image_complete[pending_index]) control_error <= ERR_NO_RESOURCE;
            end
            else begin
                pending_index <= selected[1:0];
                pending_manual <= 1'b1;
                request_toggle <= ~request_toggle;
                request_ever <= 1'b1;
            end
        end
    endtask

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            auto_enabled <= (AUTO_GUIDE_DEFAULT != 0);
            active_period_cycles <= DEFAULT_PERIOD_CYCLES;
            pending_index <= 2'd0;
            pending_manual <= 1'b0;
            control_error <= ERR_NONE;
            index_commit_sys <= 1'b0;
            auto_next_accepted <= 1'b0;
            sys_current_index <= 2'd0;
            sys_active_valid <= 1'b0;
            display_mode <= 1'b0;
            focus_index <= 2'd0;
            auto_enabled_saved <= (AUTO_GUIDE_DEFAULT != 0);
            mode_request_pending <= 1'b0;
            mode_request_value <= 1'b0;
            enter_wait_transition <= 1'b0;
            exit_wait_transition <= 1'b0;
            exit_target_index <= 2'd0;
            request_busy_d <= 1'b0;
            visual_busy_sys1 <= 1'b0;
            visual_busy_sys2 <= 1'b0;
            frame_sync1 <= 1'b0;
            frame_sync2 <= 1'b0;
            frame_seen_sys <= 1'b0;
            request_ever <= 1'b0;
            request_toggle <= 1'b0;
            ack_sync1 <= 1'b0;
            ack_sync2 <= 1'b0;
            commit_sync1<=0; commit_sync2<=0; commit_seen_sys<=0;
            displayed_sync1<=0; displayed_sync2<=0;
        end else begin
            index_commit_sys <= 1'b0;
            auto_next_accepted <= 1'b0;
            visual_busy_sys1 <= visual_busy;
            visual_busy_sys2 <= visual_busy_sys1;
            ack_sync1 <= ack_toggle_pix;
            ack_sync2 <= ack_sync1;
            commit_sync1<=commit_toggle_pix; commit_sync2<=commit_sync1;
            displayed_sync1<=active_index; displayed_sync2<=displayed_sync1;
            frame_sync1 <= frame_toggle_pix;
            frame_sync2 <= frame_sync1;
            request_busy_d <= request_busy;
            if (frame_tick_sys)
                frame_seen_sys <= frame_sync2;
            if (commit_sync2!=commit_seen_sys) begin
                commit_seen_sys<=commit_sync2;
                sys_current_index<=displayed_sync2;
                sys_active_valid<=1'b1;
                index_commit_sys<=1'b1;
            end

            if (enter_wait_transition && request_busy_d && !request_busy) begin
                enter_wait_transition <= 1'b0;
                mode_request_value <= 1'b1;
                mode_request_pending <= 1'b1;
            end

            if (exit_wait_transition && request_busy_d && !request_busy) begin
                exit_wait_transition <= 1'b0;
                mode_request_value <= 1'b0;
                mode_request_pending <= 1'b1;
            end

            if (mode_request_pending && frame_tick_sys) begin
                mode_request_pending <= 1'b0;
                display_mode <= mode_request_value;
                if (mode_request_value) begin
                    focus_index <= sys_current_index;
                end else begin
                    auto_enabled <= auto_enabled_saved;
                end
            end

            if (period_cfg_valid) begin
                if (period_cfg_cycles < MIN_PERIOD_CYCLES ||
                    period_cfg_cycles > MAX_PERIOD_CYCLES) begin
                    control_error <= ERR_PERIOD_RANGE;
                end else begin
                    active_period_cycles <= period_cfg_cycles;
                end
            end

            if (auto_cfg_valid) begin
                auto_enabled <= auto_cfg_enabled;
            end

            if (auto_req && !display_mode && !mode_request_pending &&
                !enter_wait_transition && !exit_wait_transition) begin
                auto_enabled <= ~auto_enabled;
            end

            if (mode_req && !mode_request_pending &&
                !enter_wait_transition && !exit_wait_transition) begin
                if (!display_mode) begin
                    auto_enabled_saved <= auto_enabled;
                    auto_enabled <= 1'b0;
                    if (request_busy)
                        enter_wait_transition <= 1'b1;
                    else begin
                        mode_request_value <= 1'b1;
                        mode_request_pending <= 1'b1;
                    end
                end else begin
                    mode_request_value <= 1'b0;
                    mode_request_pending <= 1'b1;
                end
            end

            if (display_mode && !mode_request_pending &&
                !enter_wait_transition && !exit_wait_transition) begin
                if (focus_forward_req) begin
                    focus_index <= focus_index + 2'd1;
                end else if (focus_backward_req) begin
                    focus_index <= focus_index - 2'd1;
                end else if (focus_select_req && image_complete[focus_index] &&
                             !request_busy) begin
                    // Keep selection on the existing CDC/frame commit path.
                    exit_target_index <= focus_index;
                    pending_index <= focus_index;
                    pending_manual <= 1'b1;
                    request_toggle <= ~request_toggle;
                    request_ever <= 1'b1;
                    exit_wait_transition <= 1'b1;
                end
            end

            // Select the first successfully loaded bank without a key press.
            if (!display_mode && !mode_request_pending &&
                !enter_wait_transition && !exit_wait_transition &&
                !sys_active_valid && !request_ever && !request_busy &&
                (auto_enabled ? image_complete[0] : |image_complete)) begin
                issue_next_request(1'b0);
            end else if (!display_mode && !mode_request_pending &&
                         !enter_wait_transition && !exit_wait_transition &&
                          next_req && sys_active_valid) begin
                if (!request_busy) issue_next_request(1'b1);
            end else if (!display_mode && !mode_request_pending &&
                         !enter_wait_transition && !exit_wait_transition &&
                          prev_req && sys_active_valid) begin
                if (!request_busy) issue_prev_request();
            end else if (auto_next_req && auto_request_ready) begin
                // Exact completion-driven target; do not skip unloaded banks.
                pending_index <= auto_next_index;
                pending_manual <= 1'b0;
                request_toggle <= ~request_toggle;
                request_ever <= 1'b1;
                auto_next_accepted <= 1'b1;
            end else if (auto_next_req && !image_complete[auto_next_index]) begin
                control_error <= ERR_NO_RESOURCE;
            end
        end
    end

    // Bundled payload is held by the source until the ENTIRE transition is
    // acknowledged. One settling cycle follows toggle detection before capture.
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            index_sync1<=0; index_sync2<=0;
            manual_sync1<=0; manual_sync2<=0;
            req_sync1<=0; req_sync2<=0; req_seen_pix<=0;
            capture_pending_pix<=0; commit_pending_pix<=0;
            requested_index_pix<=0; requested_manual_pix<=0;
            ack_toggle_pix<=0; commit_toggle_pix<=0; frame_toggle_pix<=0;
            audio_ready_pix1<=0; audio_ready_pix2<=0;
        end else begin
            audio_ready_pix1<=audio_switch_ready;
            audio_ready_pix2<=audio_ready_pix1;
            if (frame_start) frame_toggle_pix <= ~frame_toggle_pix;
            if (index_commit) commit_toggle_pix<=~commit_toggle_pix;
            index_sync1<=pending_index; index_sync2<=index_sync1;
            manual_sync1<=pending_manual; manual_sync2<=manual_sync1;
            req_sync1<=request_toggle; req_sync2<=req_sync1;
            if (req_sync2 != req_seen_pix && !commit_pending_pix && !capture_pending_pix) begin
                req_seen_pix<=req_sync2;
                capture_pending_pix<=1;
            end
            if (capture_pending_pix) begin
                capture_pending_pix<=0;
                requested_index_pix<=index_sync2;
                requested_manual_pix<=manual_sync2;
                commit_pending_pix<=1;
            end
            if (transition_done) begin
                commit_pending_pix<=0;
                ack_toggle_pix<=req_seen_pix;
            end
        end
    end

    video_fade_transition #(.FADE_FRAMES(FADE_FRAMES), .IMAGE_WIDTH(IMAGE_WIDTH))
    u_transition (
        .pix_clk(pix_clk), .pix_rst_n(pix_rst_n), .frame_start(frame_start),
        // Keep an accepted manual payload pending until the zoom releases it.
        .request_valid(commit_pending_pix && !visual_busy && audio_ready_pix2), .request_index(requested_index_pix),
        .request_manual(requested_manual_pix), .request_done(transition_done),
        .active_index(active_index), .active_valid(active_valid), .index_commit(index_commit),
        .alpha_out(transition_alpha), .fade_active(transition_active),
        .wipe_active(wipe_active), .old_index(old_index), .wipe_boundary(wipe_boundary),
        .transition_kind()
    );
endmodule
