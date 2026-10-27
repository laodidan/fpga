`timescale 1ns/1ps

// Latest-status mailbox, not an event queue. Changes during an outstanding
// transfer are coalesced; once the input settles, its latest value is delivered.
// The source holds the entire bus until the destination acknowledges capture.
// Request and acknowledgement use 2FF synchronizers; capture has one extra
// destination settling cycle after request detection. Both sides restart if
// either reset is asserted, with synchronous local reset release.
module osd_status_cdc #(
    parameter integer WIDTH = 8
) (
    input wire src_clk,
    input wire src_rst_n,
    input wire [WIDTH-1:0] src_data,
    input wire dst_clk,
    input wire dst_rst_n,
    output reg [WIDTH-1:0] dst_data
);
    wire reset_both_n = src_rst_n && dst_rst_n;
    (* async_reg = "true" *) reg [1:0] src_release;
    (* async_reg = "true" *) reg [1:0] dst_release;
    wire src_run_n = src_release[1];
    wire dst_run_n = dst_release[1];

    always @(posedge src_clk or negedge reset_both_n) begin
        if (!reset_both_n) src_release <= 2'b00;
        else src_release <= {src_release[0], 1'b1};
    end
    always @(posedge dst_clk or negedge reset_both_n) begin
        if (!reset_both_n) dst_release <= 2'b00;
        else dst_release <= {dst_release[0], 1'b1};
    end

    reg [WIDTH-1:0] held_data;
    reg request_toggle;
    reg sent_once;
    reg ack_toggle;
    (* async_reg = "true" *) reg ack_sync1, ack_sync2;
    (* async_reg = "true" *) reg request_sync1, request_sync2;
    reg capture_pending;
    wire src_busy = request_toggle != ack_sync2;

    always @(posedge src_clk or negedge src_run_n) begin
        if (!src_run_n) begin
            held_data <= {WIDTH{1'b0}};
            request_toggle <= 1'b0;
            sent_once <= 1'b0;
            ack_sync1 <= 1'b0;
            ack_sync2 <= 1'b0;
        end else begin
            ack_sync1 <= ack_toggle;
            ack_sync2 <= ack_sync1;
            if (!src_busy && (!sent_once || src_data != held_data)) begin
                held_data <= src_data;
                request_toggle <= ~request_toggle;
                sent_once <= 1'b1;
            end
        end
    end

    always @(posedge dst_clk or negedge dst_run_n) begin
        if (!dst_run_n) begin
            request_sync1 <= 1'b0;
            request_sync2 <= 1'b0;
            ack_toggle <= 1'b0;
            capture_pending <= 1'b0;
            dst_data <= {WIDTH{1'b0}};
        end else begin
            request_sync1 <= request_toggle;
            request_sync2 <= request_sync1;
            if (capture_pending) begin
                dst_data <= held_data;
                ack_toggle <= request_sync2;
                capture_pending <= 1'b0;
            end else if (request_sync2 != ack_toggle) begin
                capture_pending <= 1'b1;
            end
        end
    end
endmodule
