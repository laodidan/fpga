// Key events must be one-clock pulses synchronous to clk.
// Requests are registered and delayed by one clock.
module thumbnail_controller (
    input  wire clk,
    input  wire rst_n,
    input  wire key1_short_event,
    input  wire key2_short_event,
    input  wire combo_long_event,
    output reg  mode_toggle_req,
    output reg  focus_next_req,
    output reg  focus_prev_req,
    output reg  select_req
);
    // Active-low synchronous reset; no display or image state is stored.
    always @(posedge clk) begin
        if (!rst_n) begin
            mode_toggle_req <= 1'b0;
            focus_next_req  <= 1'b0;
            focus_prev_req  <= 1'b0;
            select_req      <= 1'b0;
        end else begin
            mode_toggle_req <= combo_long_event;
            focus_next_req  <= key1_short_event;
            // No previous-focus event is defined by this interface.
            focus_prev_req  <= 1'b0;
            select_req      <= key2_short_event;
        end
    end
endmodule
