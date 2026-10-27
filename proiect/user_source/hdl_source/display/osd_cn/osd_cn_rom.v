`timescale 1ns/1ps

// Synchronous read-only table for the Chinese OSD (font and text ROMs).
// TD 6.2.1 maps this onto BRAM9K and copies the $readmemh contents into the
// block INIT values. A relative INIT_FILE is resolved from this file's own
// directory; a missing file stops elaboration with HDL-8007.
module osd_cn_rom #(
    parameter integer ADDR_W = 11,
    parameter integer DATA_W = 8,
    parameter INIT_FILE = "osd_cn_font.hex"
) (
    input  wire              clk,
    input  wire [ADDR_W-1:0] addr,
    output reg  [DATA_W-1:0] data
);
    (* ram_style = "block" *) reg [DATA_W-1:0] mem [0:(1<<ADDR_W)-1];

    initial $readmemh(INIT_FILE, mem);

    always @(posedge clk)
        data <= mem[addr];
endmodule

// Same table with a second synchronous read port; both ports share the
// block RAM's two physical ports instead of duplicating its contents.
module osd_cn_rom2 #(
    parameter integer ADDR_W = 11,
    parameter integer DATA_W = 8,
    parameter INIT_FILE = "osd_cn_font.hex"
) (
    input  wire              clk,
    input  wire [ADDR_W-1:0] addr_a,
    output reg  [DATA_W-1:0] data_a,
    input  wire [ADDR_W-1:0] addr_b,
    output reg  [DATA_W-1:0] data_b
);
    (* ram_style = "block" *) reg [DATA_W-1:0] mem [0:(1<<ADDR_W)-1];

    initial $readmemh(INIT_FILE, mem);

    always @(posedge clk) begin
        data_a <= mem[addr_a];
        data_b <= mem[addr_b];
    end
endmodule
