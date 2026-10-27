`timescale 1ns/1ps

// Ordered BMP control/pixel stream, 100 MHz SD domain -> 125 MHz SDRAM domain.
// Event types: 00=start (data[1:0]=bank), 01=RGB pixel,
//              10=commit (after last pixel), 11=abort (data[7:0]=error).
module media_pixel_cdc (
    input  wire        src_clk,
    input  wire        src_rst_n,
    input  wire        src_valid,
    input  wire [1:0]  src_type,
    input  wire [31:0] src_data,
    output wire        src_full,
    input  wire        dst_clk,
    input  wire        dst_rst_n,
    output wire        dst_valid,
    output wire [1:0]  dst_type,
    output wire [31:0] dst_data
);
    wire [33:0] fifo_data;
    wire fifo_empty;
    wire fifo_full;
    wire [9:0] fifo_wr_level;
    // The source registers its event for one cycle. Reserve one entry for
    // that in-flight register so a full transition cannot discard an event.
    assign src_full = fifo_full || (fifo_wr_level >= 10'd511);
    // From the lesson's media_sdram_write_cdc.v: gray-pointer asynchronous
    // FIFO with registered read data and valid.
    async_command_fifo #(.DATA_WIDTH(34), .ADDR_WIDTH(9)) u_events (
        .wr_clk(src_clk), .wr_rst_n(src_rst_n),
        .wr_en(src_valid), .wr_data({src_type, src_data}),
        .wr_full(fifo_full), .wr_level(fifo_wr_level),
        .rd_clk(dst_clk), .rd_rst_n(dst_rst_n),
        .rd_en(!fifo_empty), .rd_data(fifo_data),
        .rd_valid(dst_valid), .rd_empty(fifo_empty), .rd_level()
    );
    assign {dst_type, dst_data} = fifo_data;
endmodule
