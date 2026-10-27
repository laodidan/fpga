`timescale 1ns/1ps

// Refresh admission guard for the encrypted sdr_as_ram controller.
// sdr_init_ref (visible source) counts init_cnt 0..SELF_REFRESH_INTERVAL on
// Sdr_clk, synchronously cleared by Rst or !init_done, and registers its
// one-cycle internal refresh request when init_cnt==SELF_REFRESH_INTERVAL-10.
// This module mirrors that counter with the same clock, reset and init_done,
// and reports, for the admission edge, whether a new burst would start inside
// the window around that request.  It only gates the start of new bursts; it
// never touches a burst already issuing or its returns.
`include "../user_source/hdl_source/include/global_def.v"

module sdram_refresh_guard #(
    parameter integer PRE_SHORT = 192, // up to 128-word bursts
    parameter integer PRE_LONG = 704,  // up to 640-word bursts
    parameter integer POST = 32
) (
    input  wire clk,
    input  wire rst,
    input  wire init_done,
    output reg  guard_short,
    output reg  guard_long
);
    localparam integer LAST = `SELF_REFRESH_INTERVAL;
    localparam integer REQ = `SELF_REFRESH_INTERVAL - 10;

    reg [13:0] count;
    wire [13:0] count_next = (rst || !init_done) ? 14'd0 :
                             (count == LAST) ? 14'd0 : count + 1'b1;

    // Registered from count_next, so the flag seen at an edge corresponds to
    // the controller's init_cnt value at that same edge.
    always @(posedge clk) begin
        count <= count_next;
        guard_short <= (count_next >= REQ - PRE_SHORT) || (count_next <= POST);
        guard_long <= (count_next >= REQ - PRE_LONG) || (count_next <= POST);
    end
endmodule
