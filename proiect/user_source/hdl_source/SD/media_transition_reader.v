`timescale 1ns/1ps

// Four resident images, one physical read port, one ordered pixel FIFO.
// Preserve the inherited controller's contiguous BURST_SIZE commands. A block
// straddling the Wipe boundary is read twice: new prefix, then old suffix.
// Only retained return words enter the FIFO; no black substitute and no two
// independent readers contending for SDRAM. At most one burst is outstanding.
module media_transition_reader #(
    parameter integer ADDR_WIDTH=21,
    parameter integer IMAGE_WORDS=307200,
    parameter integer IMAGE_WIDTH=640,
    parameter integer FIFO_DEPTH=512,
    parameter integer FIFO_MARGIN=8,
    parameter integer BURST_SIZE=128,
    parameter integer BURST_GAP_CYCLES=12
) (
    input wire mem_clk, mem_rst_n,
    input wire controller_init_done, controller_busy, refresh_active,
    input wire burst_grant,
    output wire burst_request, burst_running,
    output reg app_rd_en,
    output reg [ADDR_WIDTH-1:0] app_rd_addr,
    input wire sdram_rd_valid,
    input wire [31:0] sdram_rd_data,
    input wire pix_clk, pix_rst_n, frame_start,
    input wire [1:0] active_index, old_index,
    input wire wipe_active,
    input wire [11:0] wipe_boundary,
    input wire pixel_consume,
    output wire [23:0] pixel_rgb,
    output wire pixel_valid,
    output reg underflow,
    output reg reader_busy, frame_read_done,
    output reg [1:0] reader_index,
    output reg [7:0] reader_error,
    // Thumbnail mode inputs.  These are sampled in the pixel clock domain
    // for each consumed thumbnail pixel and carried to mem_clk by an async
    // request FIFO.
    input wire display_mode,
    input wire [1:0] source_index,
    input wire [9:0] source_x,
    input wire [9:0] source_y,
    input wire [3:0] image_complete
);
    localparam integer FIFO_W=$clog2(FIFO_DEPTH);
    localparam integer BURST_W=(BURST_SIZE<2)?1:$clog2(BURST_SIZE+1);
    localparam integer THUMB_TAG_DEPTH=1024;
    localparam integer THUMB_TAG_W=10;
    localparam integer THUMB_RESP_DEPTH=64;
    localparam integer THUMB_RESP_W=6;
    localparam [1:0] IDLE=0, ISSUE=1, DRAIN=2;
    reg [1:0] state;
    reg request_pending_pix, request_toggle_pix, frame_armed_pix;
    reg frame_had_consume_pix;
    reg transition_flush_pix;
    reg reload_ack_toggle_mem, reload_ack_wait_mem;
    (* async_reg="true" *) reg reload_ack_pix1, reload_ack_pix2;
    reg reload_ack_seen_pix;
    // Atomic frame descriptor; held until next vertical blank (one full frame).
    reg [17:0] descriptor_pix;
    (* async_reg="true" *) reg request_sync1, request_sync2;
    (* async_reg="true" *) reg [17:0] desc_sync1, desc_sync2;
    reg request_seen, capture_pending, reload_pending;
    reg [17:0] reload_descriptor;
    reg [17:0] active_descriptor_mem;
    reg [1:0] old_index_mem;
    reg wipe_mem;
    reg [11:0] boundary_mem;
    reg display_mode_mem;
    reg [ADDR_WIDTH-1:0] block_offset;
    reg [11:0] block_x;
    reg second_pass;
    reg burst_split, burst_old;
    reg [1:0] burst_bank;
    reg [BURST_W-1:0] issued_in_burst, returned_in_burst;
    reg [15:0] gap_count;
    reg [3:0] reset_count;
    reg [3:0] release_mem, release_pix;
    (* async_reg="true" *) reg ready_pix_sync1, ready_pix_sync2;
    reg [ADDR_WIDTH:0] consumed_count_pix;
    reg [THUMB_TAG_DEPTH-1:0] thumb_tag_mem;
    reg [THUMB_TAG_W-1:0] thumb_tag_wr_ptr, thumb_tag_rd_ptr;
    reg [THUMB_TAG_W:0] thumb_tag_count;
    reg [31:0] thumb_resp_mem [0:THUMB_RESP_DEPTH-1];
    reg [THUMB_RESP_W-1:0] thumb_resp_wr_ptr, thumb_resp_rd_ptr;
    reg [THUMB_RESP_W:0] thumb_resp_count;
    reg [7:0] thumb_outstanding;
    // Thumbnail SDRAM accesses use the same fixed-length command contract as
    // fullscreen reads.  One request owns one burst; only target_offset is
    // retained from the returned burst.
    reg thumb_burst_active, thumb_burst_issue_done;
    reg [BURST_W-1:0] thumb_burst_issued, thumb_burst_returned;
    reg [BURST_W-1:0] thumb_target_offset;
    reg [ADDR_WIDTH-1:0] thumb_burst_base;
    reg [31:0] thumb_req_q_reg;
    reg thumb_req_valid_reg;
    reg thumb_emit_valid;
    reg [31:0] thumb_emit_data;
    wire new_request = request_sync2!=request_seen;
    wire reload_discard = new_request || capture_pending || reload_pending;
    wire legacy_continue_mem = display_mode_mem && desc_sync2[17] &&
                               desc_sync2==active_descriptor_mem;
    wire thumb_reload_discard = reload_pending ||
                               ((new_request || capture_pending) &&
                                !legacy_continue_mem);
    wire reset_request = !mem_rst_n || !pix_rst_n || (reset_count!=0);
    wire [17:0] current_descriptor_pix =
        {display_mode,active_index,old_index,wipe_active,wipe_boundary};
    wire descriptor_changed_pix = current_descriptor_pix!=descriptor_pix;
    wire start_flush_pix = frame_start &&
                           (!display_mode || !descriptor_pix[17] ||
                            descriptor_changed_pix);
    // Clear both prefetch slots as soon as the new frame starts.  Keep them
    // empty until memory has drained the old burst and reset/released the FIFO.
    wire prefetch_reset = !release_pix[3] || transition_flush_pix || start_flush_pix ||
                          (request_pending_pix && descriptor_changed_pix);
    wire fifo_ready = release_mem[3] && ready_pix_sync2;
    wire fifo_full, fifo_empty, raw_valid, raw_re, raw_re_prefetch;
    assign raw_re = raw_re_prefetch && !prefetch_reset;
    wire [FIFO_W-1:0] fifo_used;
    wire [31:0] raw_data, prefetch_data;
    wire prefetch_valid;
    wire pop = pixel_consume && !display_mode && prefetch_valid && !prefetch_reset;

    // Thumbnail request FIFO (pixel -> memory) and data FIFO (memory ->
    // pixel).  The high bit of a request is the image-complete bit; invalid
    // banks are represented in the stream and become a black output without
    // issuing an SDRAM command.
    wire [31:0] thumb_req_data;
    wire thumb_req_we, thumb_req_full, thumb_req_empty, thumb_req_valid;
    wire [31:0] thumb_req_q;
    // The FIFO instance is configured with ADDR_WIDTH_* = FIFO_W.  Keep the
    // used-count nets at that same width; a wider net leaves undriven upper
    // bits as Z in simulation, which makes the space comparison X.
    wire [FIFO_W-1:0] thumb_req_used;
    wire thumb_req_re;
    wire thumb_data_we, thumb_data_full, thumb_data_empty, thumb_data_valid;
    wire [31:0] thumb_data_din;
    wire [31:0] thumb_data_q;
    wire [FIFO_W-1:0] thumb_data_used;
    wire thumb_raw_re, thumb_raw_re_prefetch, thumb_prefetch_valid, thumb_pop;
    assign thumb_raw_re = thumb_raw_re_prefetch && !prefetch_reset;
    wire [31:0] thumb_prefetch_data;
    reg [3:0] image_complete_pix1, image_complete_pix2;
    wire [ADDR_WIDTH-1:0] thumb_pixel_addr =
        bank_base(source_index) + source_y * IMAGE_WIDTH + source_x;
    assign thumb_req_data = {image_complete_pix2[source_index],
                             {(31-ADDR_WIDTH){1'b0}}, thumb_pixel_addr};
    assign thumb_req_we = display_mode && pixel_consume && !reset_request &&
                          !prefetch_reset &&
                          !thumb_req_full;
    wire thumb_mode_mem = display_mode_mem;
    wire thumb_req_has_space = (thumb_tag_count < THUMB_TAG_DEPTH-2);
    wire thumb_out_has_space =
        (thumb_data_used + thumb_outstanding < FIFO_DEPTH-FIFO_MARGIN);
    assign thumb_req_re = thumb_mode_mem && !reset_request && fifo_ready &&
                          !thumb_reload_discard &&
                          burst_grant && !thumb_req_empty && thumb_req_has_space &&
                          thumb_out_has_space && !thumb_burst_active;
    wire thumb_tag_head = thumb_tag_mem[thumb_tag_rd_ptr];
    wire thumb_emit_ready = thumb_mode_mem && !reset_request &&
                             !thumb_reload_discard &&
                             !thumb_data_full && (thumb_tag_count != 0) &&
                             (!thumb_tag_head || (thumb_resp_count != 0));
    assign thumb_data_we = thumb_emit_ready;
    assign thumb_data_din = thumb_tag_head ? thumb_resp_mem[thumb_resp_rd_ptr] : 32'd0;
    assign thumb_pop = pixel_consume && display_mode && thumb_prefetch_valid &&
                       !prefetch_reset;
    wire crosses_boundary = wipe_mem && (block_x<boundary_mem) &&
                                      (block_x+BURST_SIZE>boundary_mem);
    wire select_old = wipe_mem && (second_pass || block_x>=boundary_mem);
    wire return_owned = state!=IDLE && returned_in_burst<issued_in_burst &&
                        returned_in_burst<BURST_SIZE;
    wire [12:0] return_x = {1'b0,block_x} + returned_in_burst;
    wire keep_return = !burst_split ||
                      (burst_old ? return_x>=boundary_mem : return_x<boundary_mem);
    wire fifo_we = sdram_rd_valid && return_owned && keep_return &&
                   !reload_discard && !reset_request && !fifo_full;
    wire thumb_accept_request = !reset_request && thumb_mode_mem &&
                                !thumb_reload_discard && thumb_req_valid &&
                                !thumb_burst_active &&
                                thumb_tag_count < THUMB_TAG_DEPTH-1;
    wire thumb_return_owned = thumb_mode_mem && thumb_burst_active &&
                              thumb_outstanding!=0 &&
                              thumb_burst_returned<thumb_burst_issued &&
                              thumb_burst_returned<BURST_SIZE;
    wire thumb_keep_response = !reset_request && thumb_return_owned &&
                              !thumb_reload_discard && sdram_rd_valid &&
                              thumb_burst_returned==thumb_target_offset &&
                              thumb_resp_count < THUMB_RESP_DEPTH-1;
    // Reserve a complete block conservatively, including discarded words.
    // FIFO used count is conservative because the read pointer crosses clocks.
    assign burst_request = thumb_mode_mem ?
                           (reader_busy && controller_init_done &&
                            !reset_request && fifo_ready && !thumb_req_empty &&
                            !thumb_burst_active && !thumb_reload_discard) :
                           (reader_busy && controller_init_done && fifo_ready &&
                            !reset_request && state==IDLE && !app_rd_en &&
                            gap_count==0 && !reload_pending && !capture_pending &&
                            !new_request && fifo_used<=FIFO_DEPTH-FIFO_MARGIN-BURST_SIZE);
    // Hold the existing top-level writer arbiter through the final read return.
    assign burst_running = thumb_mode_mem ?
                           (reader_busy && (!thumb_req_empty ||
                            thumb_req_valid || thumb_outstanding!=0 ||
                            thumb_burst_active)) :
                           (state!=IDLE || app_rd_en);
    // busy/refresh are status, NOT per-word ready. Never split a command burst.
    // As in the inherited reader, the encrypted controller handles refresh.
    function [ADDR_WIDTH-1:0] bank_base;
        input [1:0] index;
        begin
            case(index)
            0: bank_base=0;
            1: bank_base=IMAGE_WORDS;
            2: bank_base=IMAGE_WORDS*2;
            default: bank_base=IMAGE_WORDS*3;
            endcase
        end
    endfunction

    always @(posedge mem_clk or posedge reset_request)
        if(reset_request) release_mem<=0;
        else release_mem<={release_mem[2:0],1'b1};
    always @(posedge pix_clk or posedge reset_request)
        if(reset_request) release_pix<=0;
        else release_pix<={release_pix[2:0],1'b1};

    // Snapshot one pixel clock after player/transition commit, still in blank.
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if(!pix_rst_n) begin
            request_pending_pix<=0; request_toggle_pix<=0; descriptor_pix<=0;
            frame_armed_pix<=0; consumed_count_pix<=0; underflow<=0;
            frame_had_consume_pix<=0;
            image_complete_pix1<=0; image_complete_pix2<=0;
            transition_flush_pix<=1;
            reload_ack_pix1<=0; reload_ack_pix2<=0; reload_ack_seen_pix<=0;
        end else begin
            image_complete_pix1<=image_complete;
            image_complete_pix2<=image_complete_pix1;
            reload_ack_pix1<=reload_ack_toggle_mem;
            reload_ack_pix2<=reload_ack_pix1;
            if(reload_ack_pix2!=reload_ack_seen_pix) begin
                reload_ack_seen_pix<=reload_ack_pix2;
                if(!request_pending_pix && reload_ack_pix2==request_toggle_pix)
                    transition_flush_pix<=0;
            end
            if(frame_start) begin
                if(frame_armed_pix && frame_had_consume_pix && !display_mode &&
                   consumed_count_pix!=IMAGE_WORDS) underflow<=1;
                consumed_count_pix<=0;
                frame_had_consume_pix<=0;
                request_pending_pix<=1;
                if(start_flush_pix) transition_flush_pix<=1;
            end else if(request_pending_pix) begin
                descriptor_pix<=current_descriptor_pix;
                request_toggle_pix<=~request_toggle_pix;
                request_pending_pix<=0;
                frame_armed_pix<=1;
                if(!display_mode || !descriptor_pix[17] || descriptor_changed_pix)
                    transition_flush_pix<=1;
            end
            if(pixel_consume && !frame_start && !display_mode) begin
                if(frame_armed_pix) frame_had_consume_pix<=1;
                if(pixel_valid) consumed_count_pix<=consumed_count_pix+1'b1;
                else if(frame_armed_pix) underflow<=1;
            end else if(pixel_consume && !frame_start && display_mode &&
                        frame_armed_pix && !pixel_valid) begin
                underflow<=1;
            end
        end
    end

    always @(posedge mem_clk or negedge mem_rst_n) begin
        if(!mem_rst_n) begin
            state<=IDLE; request_sync1<=0; request_sync2<=0; request_seen<=0;
            desc_sync1<=0; desc_sync2<=0; capture_pending<=0;
            reload_pending<=0; reload_descriptor<=0;
            reload_ack_toggle_mem<=0; reload_ack_wait_mem<=0;
            active_descriptor_mem<=0;
            old_index_mem<=0; wipe_mem<=0; boundary_mem<=0; display_mode_mem<=0;
            block_offset<=0; block_x<=0; second_pass<=0;
            burst_split<=0; burst_old<=0; burst_bank<=0;
            issued_in_burst<=0; returned_in_burst<=0;
            gap_count<=0; reset_count<=0;
            ready_pix_sync1<=0; ready_pix_sync2<=0;
            app_rd_en<=0; app_rd_addr<=0;
            reader_busy<=0; frame_read_done<=0; reader_index<=0; reader_error<=0;
            thumb_tag_mem<=0; thumb_tag_wr_ptr<=0; thumb_tag_rd_ptr<=0;
            thumb_tag_count<=0; thumb_resp_wr_ptr<=0; thumb_resp_rd_ptr<=0;
            thumb_resp_count<=0; thumb_outstanding<=0; thumb_req_q_reg<=0;
            thumb_req_valid_reg<=0; thumb_emit_valid<=0; thumb_emit_data<=0;
            thumb_burst_active<=0; thumb_burst_issue_done<=0;
            thumb_burst_issued<=0; thumb_burst_returned<=0;
            thumb_target_offset<=0; thumb_burst_base<=0;
        end else begin
            request_sync1<=request_toggle_pix; request_sync2<=request_sync1;
            desc_sync1<=descriptor_pix; desc_sync2<=desc_sync1;
            ready_pix_sync1<=release_pix[3]; ready_pix_sync2<=ready_pix_sync1;
            app_rd_en<=0; frame_read_done<=0;
            if(reset_count!=0) reset_count<=reset_count-1'b1;
            if(gap_count!=0) gap_count<=gap_count-1'b1;
            if(reload_ack_wait_mem && reset_count==0 && fifo_ready &&
               !reload_discard) begin
                reload_ack_toggle_mem<=request_seen;
                reload_ack_wait_mem<=0;
            end
            if(reset_count!=0) begin
                thumb_tag_mem<=0; thumb_tag_wr_ptr<=0; thumb_tag_rd_ptr<=0;
                thumb_tag_count<=0; thumb_resp_wr_ptr<=0; thumb_resp_rd_ptr<=0;
                thumb_resp_count<=0; thumb_outstanding<=0;
                thumb_burst_active<=0; thumb_burst_issue_done<=0;
                thumb_burst_issued<=0; thumb_burst_returned<=0;
                thumb_target_offset<=0; thumb_burst_base<=0;
            end
            if(new_request) begin
                request_seen<=request_sync2;
                capture_pending<=1;
            end
            if(capture_pending) begin
                capture_pending<=0;
                // Fullscreen always restarts, including transition completion.
                // Preserve the legacy thumbnail stream across identical frames.
                if(legacy_continue_mem) begin
                    reload_ack_toggle_mem<=request_seen;
                end else begin
                    reload_pending<=1;
                    reload_descriptor<=desc_sync2;
                end
            end

            // Drain the entire old burst before clearing the asynchronous FIFO.
            // A late return is never reinterpreted as a pixel from the new frame.
            if(reload_pending && state==IDLE && !app_rd_en &&
               issued_in_burst==returned_in_burst && gap_count==0 &&
               !thumb_burst_active && thumb_outstanding==0 && reset_count==0 &&
               !capture_pending && !new_request) begin
                reload_pending<=0;
                reload_ack_wait_mem<=1;
                active_descriptor_mem<=reload_descriptor;
                display_mode_mem<=reload_descriptor[17];
                reader_index<=reload_descriptor[16:15];
                old_index_mem<=reload_descriptor[14:13];
                wipe_mem<=reload_descriptor[12];
                boundary_mem<=reload_descriptor[11:0];
                block_offset<=0; block_x<=0; second_pass<=0;
                issued_in_burst<=0; returned_in_burst<=0;
                reset_count<=4'd8; gap_count<=0;
                reader_busy<=1;
                if (reload_descriptor[17]) begin
                    reader_busy<=1;
                    state<=IDLE;
                end
                // reader_error is sticky until reset, so a transient fault is visible.
            end else if(!thumb_mode_mem && burst_request && burst_grant) begin
                state<=ISSUE;
                burst_bank<=select_old?old_index_mem:reader_index;
                burst_split<=crosses_boundary;
                burst_old<=select_old;
                issued_in_burst<=0; returned_in_burst<=0;
            end

            if(!thumb_mode_mem && state==ISSUE) begin
                app_rd_en<=1;
                app_rd_addr<=bank_base(burst_bank)+block_offset+issued_in_burst;
                issued_in_burst<=issued_in_burst+1'b1;
                if(issued_in_burst==BURST_SIZE-1) state<=DRAIN;
            end

            if(sdram_rd_valid && !thumb_mode_mem) begin
                if(!return_owned) reader_error<=8'h64;
                else begin
                    returned_in_burst<=returned_in_burst+1'b1;
                    if(keep_return && !reload_discard && !reset_request && fifo_full)
                        reader_error<=8'h62;
                    if(returned_in_burst==BURST_SIZE-1) begin
                        state<=IDLE;
                        gap_count<=BURST_GAP_CYCLES;
                        if(burst_split && !burst_old) second_pass<=1;
                        else begin
                            second_pass<=0;
                            if(block_offset==IMAGE_WORDS-BURST_SIZE) begin
                                reader_busy<=0;
                                frame_read_done<=1;
                            end else begin
                                block_offset<=block_offset+BURST_SIZE;
                                block_x<=(block_x==IMAGE_WIDTH-BURST_SIZE)?
                                          12'd0:block_x+BURST_SIZE;
                            end
                        end
                    end
                end
            end

            // Thumbnail requests are consumed in memory-clock order.  A tag
            // is retained for every request so unloaded banks can insert a
            // black word without disturbing the order of valid SDRAM data.
            if (thumb_accept_request) begin
                thumb_tag_mem[thumb_tag_wr_ptr] <= thumb_req_q[31];
                thumb_tag_wr_ptr <= thumb_tag_wr_ptr + 1'b1;
                if (thumb_req_q[31]) begin
                    thumb_burst_active <= 1'b1;
                    thumb_burst_issue_done <= 1'b0;
                    thumb_burst_issued <= 0;
                    thumb_burst_returned <= 0;
                    thumb_target_offset <= thumb_req_q[ADDR_WIDTH-1:0] % BURST_SIZE;
                    thumb_burst_base <= thumb_req_q[ADDR_WIDTH-1:0] -
                                        (thumb_req_q[ADDR_WIDTH-1:0] % BURST_SIZE);
                end
            end

            if (!reset_request && thumb_mode_mem && thumb_burst_active &&
                !thumb_burst_issue_done) begin
                app_rd_en <= 1'b1;
                app_rd_addr <= thumb_burst_base + thumb_burst_issued;
                thumb_burst_issued <= thumb_burst_issued + 1'b1;
                if (thumb_burst_issued == BURST_SIZE-1)
                    thumb_burst_issue_done <= 1'b1;
            end

            if (!reset_request && thumb_return_owned && sdram_rd_valid) begin
                if (thumb_keep_response) begin
                    thumb_resp_mem[thumb_resp_wr_ptr] <= sdram_rd_data;
                    thumb_resp_wr_ptr <= thumb_resp_wr_ptr + 1'b1;
                end
                thumb_burst_returned <= thumb_burst_returned + 1'b1;
                if (thumb_burst_returned == BURST_SIZE-1) begin
                    thumb_burst_active <= 1'b0;
                    thumb_burst_issue_done <= 1'b0;
                    thumb_outstanding <= thumb_outstanding - 1'b1;
                end
            end
            if(!reset_request && thumb_mode_mem && sdram_rd_valid &&
               !thumb_return_owned)
                reader_error<=8'h64;
            if(!reset_request && thumb_mode_mem && sdram_rd_valid &&
               thumb_burst_active && !thumb_reload_discard &&
               thumb_burst_returned==thumb_target_offset &&
               thumb_resp_count>=THUMB_RESP_DEPTH-1)
                reader_error<=8'h62;

            // Count maintenance must not override the earlier reset_count
            // initialization with a last-assignment-wins self assignment.
            if(reset_count==0) begin
                case ({thumb_accept_request, thumb_emit_ready})
                    2'b10: thumb_tag_count <= thumb_tag_count + 1'b1;
                    2'b01: thumb_tag_count <= thumb_tag_count - 1'b1;
                    default: thumb_tag_count <= thumb_tag_count;
                endcase
                if (thumb_emit_ready)
                    thumb_tag_rd_ptr <= thumb_tag_rd_ptr + 1'b1;

                case ({thumb_keep_response,
                       (thumb_emit_ready && thumb_tag_head)})
                    2'b10: thumb_resp_count <= thumb_resp_count + 1'b1;
                    2'b01: thumb_resp_count <= thumb_resp_count - 1'b1;
                    default: thumb_resp_count <= thumb_resp_count;
                endcase
                if (thumb_emit_ready && thumb_tag_head)
                    thumb_resp_rd_ptr <= thumb_resp_rd_ptr + 1'b1;
            end

            if (thumb_accept_request && thumb_req_q[31])
                thumb_outstanding <= thumb_outstanding + 1'b1;
        end
    end

    rfifo_32_32_512 #(
        .DATA_WIDTH_W(32), .DATA_WIDTH_R(32),
        .ADDR_WIDTH_W(FIFO_W), .ADDR_WIDTH_R(FIFO_W),
        .AL_FULL_NUM(FIFO_DEPTH-FIFO_MARGIN), .AL_EMPTY_NUM(2),
        .SHOW_AHEAD_EN(1'b0), .OUTREG_EN("NOREG")
    ) u_read_fifo (
        .rst(reset_request), .clkw(mem_clk), .clkr(pix_clk),
        .we(fifo_we), .di(sdram_rd_data), .re(raw_re && !prefetch_reset),
        .dout(raw_data), .valid(raw_valid), .full_flag(fifo_full),
        .empty_flag(fifo_empty), .afull(), .aempty(),
        .wrusedw(fifo_used), .rdusedw()
    );
    fifo_prefetch_2 #(.DATA_WIDTH(32)) u_prefetch (
        .clk(pix_clk), .rst(prefetch_reset), .raw_empty(fifo_empty),
        .raw_valid(raw_valid), .raw_data(raw_data), .raw_read_en(raw_re_prefetch),
        .consume(pop), .out_valid(prefetch_valid), .out_empty(), .out_data(prefetch_data)
    );

    rfifo_32_32_512 #(
        .DATA_WIDTH_W(32), .DATA_WIDTH_R(32),
        .ADDR_WIDTH_W(FIFO_W), .ADDR_WIDTH_R(FIFO_W),
        .AL_FULL_NUM(FIFO_DEPTH-FIFO_MARGIN), .AL_EMPTY_NUM(2),
        .SHOW_AHEAD_EN(1'b0), .OUTREG_EN("NOREG")
    ) u_thumb_request_fifo (
        .rst(reset_request), .clkw(pix_clk), .clkr(mem_clk),
        .we(thumb_req_we), .di(thumb_req_data), .re(thumb_req_re),
        .dout(thumb_req_q), .valid(thumb_req_valid), .full_flag(thumb_req_full),
        .empty_flag(thumb_req_empty), .afull(), .aempty(),
        .wrusedw(), .rdusedw(thumb_req_used)
    );

    rfifo_32_32_512 #(
        .DATA_WIDTH_W(32), .DATA_WIDTH_R(32),
        .ADDR_WIDTH_W(FIFO_W), .ADDR_WIDTH_R(FIFO_W),
        .AL_FULL_NUM(FIFO_DEPTH-FIFO_MARGIN), .AL_EMPTY_NUM(2),
        .SHOW_AHEAD_EN(1'b0), .OUTREG_EN("NOREG")
    ) u_thumb_data_fifo (
        .rst(reset_request), .clkw(mem_clk), .clkr(pix_clk),
        .we(thumb_data_we), .di(thumb_data_din), .re(thumb_raw_re && !prefetch_reset),
        .dout(thumb_data_q), .valid(thumb_data_valid), .full_flag(thumb_data_full),
        .empty_flag(thumb_data_empty), .afull(), .aempty(),
        .wrusedw(thumb_data_used), .rdusedw()
    );
    fifo_prefetch_2 #(.DATA_WIDTH(32)) u_thumb_prefetch (
        .clk(pix_clk), .rst(prefetch_reset), .raw_empty(thumb_data_empty),
        .raw_valid(thumb_data_valid), .raw_data(thumb_data_q),
        .raw_read_en(thumb_raw_re_prefetch), .consume(thumb_pop),
        .out_valid(thumb_prefetch_valid), .out_empty(),
        .out_data(thumb_prefetch_data)
    );
    assign pixel_rgb=display_mode ? thumb_prefetch_data[23:0] : prefetch_data[23:0];
    assign pixel_valid=(display_mode ? thumb_prefetch_valid : prefetch_valid) &&
                       !prefetch_reset;

    // synthesis translate_off
    initial begin
        if(IMAGE_WIDTH<1 || IMAGE_WIDTH>4095 || BURST_SIZE<1 ||
           IMAGE_WIDTH % BURST_SIZE!=0 || IMAGE_WORDS % IMAGE_WIDTH!=0)
            $fatal(1,"Reader requires whole rows and whole fixed-length blocks");
        if(FIFO_DEPTH<8 || (FIFO_DEPTH & (FIFO_DEPTH-1))!=0 ||
           FIFO_MARGIN<4 || FIFO_DEPTH-FIFO_MARGIN<BURST_SIZE)
            $fatal(1,"Invalid FIFO reservation parameters");
        if(4*IMAGE_WORDS>(2.0**ADDR_WIDTH) || BURST_GAP_CYCLES>65535)
            $fatal(1,"Reader address or gap width exceeded");
    end
    // synthesis translate_on
endmodule
