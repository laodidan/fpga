`timescale 1ns/1ps

// Read-only status collection for this project's existing error producers.
// It does not change error recovery, player control, or SDRAM transactions.
module osd_status_bridge (
    input wire sd_clk,
    input wire sd_rst_n,
    input wire auto_enabled_sd,
    input wire [1:0] focus_index_sd,
    input wire [7:0] control_error_sd,
    input wire [3:0] load_state_sd,
    input wire mem_clk,
    input wire mem_rst_n,
    input wire [7:0] loader_error_mem,
    input wire [7:0] reader_error_mem,
    input wire pix_clk,
    input wire pix_rst_n,
    input wire reader_underflow_pix,
    output wire auto_enabled_pix,
    output wire [1:0] focus_index_pix,
    output wire [7:0] error_code_pix
);
    wire [14:0] sd_status_pix;
    wire [15:0] mem_status_pix;

    // Thumbnail focus shares the auto flag's mailbox, so the two always
    // arrive in the pixel domain as one coherent player-status word.
    osd_status_cdc #(.WIDTH(15)) u_sd_status (
        .src_clk(sd_clk), .src_rst_n(sd_rst_n),
        .src_data({focus_index_sd, auto_enabled_sd, control_error_sd, load_state_sd}),
        .dst_clk(pix_clk), .dst_rst_n(pix_rst_n), .dst_data(sd_status_pix)
    );
    osd_status_cdc #(.WIDTH(16)) u_mem_status (
        .src_clk(mem_clk), .src_rst_n(mem_rst_n),
        .src_data({reader_error_mem, loader_error_mem}),
        .dst_clk(pix_clk), .dst_rst_n(pix_rst_n), .dst_data(mem_status_pix)
    );

    assign auto_enabled_pix = sd_status_pix[12];
    assign focus_index_pix = sd_status_pix[14:13];
    // Priority is explicit: pixel underflow, reader, loader, player, SD state.
    // 0x63 already denotes underflow in media_frame_reader; its underflow
    // output is in the pixel domain whereas reader_error is in the mem domain.
    // Existing SD states 8/9/A are zero-extended; normal loading states 1..4
    // are not error codes. No new parser/recovery error numbering is invented.
    assign error_code_pix = reader_underflow_pix ? 8'h63 :
                           (mem_status_pix[15:8] != 0) ? mem_status_pix[15:8] :
                           (mem_status_pix[7:0] != 0) ? mem_status_pix[7:0] :
                           (sd_status_pix[11:4] != 0) ? sd_status_pix[11:4] :
                           (sd_status_pix[3:0] >= 4'h8) ?
                               {4'h0, sd_status_pix[3:0]} : 8'h00;
endmodule
