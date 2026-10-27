`timescale 1ns/1ps

// Active-low KEY1/KEY2 gesture decoder.  The two keys are synchronized and
// debounced in this clock domain.  A combined gesture owns both keys from the
// first stable overlap until both keys are released, so a failed/complete
// combination can never fall back to single-key events.
module media_key_control #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer LONG_PRESS_MS = 1000
) (
    input  wire clk,
    input  wire rst_n,
    input  wire key1,
    input  wire key2,
    output reg  key1_short_event,
    output reg  key2_short_event,
    output reg  key2_long_event,
    output reg  combo_long_event,
    output wire key1_stable_pressed,
    output wire key2_stable_pressed
);
    localparam integer DEBOUNCE_CYCLES_RAW = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;
    localparam integer DEBOUNCE_CYCLES = (DEBOUNCE_CYCLES_RAW < 1) ? 1 : DEBOUNCE_CYCLES_RAW;
    localparam integer LONG_CYCLES_RAW = (CLK_FREQ_HZ / 1000) * LONG_PRESS_MS;
    localparam integer LONG_CYCLES = (LONG_CYCLES_RAW < 1) ? 1 : LONG_CYCLES_RAW;
    localparam integer DB_W = (DEBOUNCE_CYCLES < 2) ? 1 : $clog2(DEBOUNCE_CYCLES + 1);
    localparam integer LONG_W = (LONG_CYCLES < 2) ? 1 : $clog2(LONG_CYCLES + 1);

    reg key1_meta, key1_sync, key2_meta, key2_sync;
    reg key1_stable, key2_stable;
    reg key1_stable_d, key2_stable_d;
    reg [DB_W-1:0] key1_db_cnt, key2_db_cnt;
    reg [LONG_W-1:0] combo_cnt, key2_cnt;
    reg key1_pending, key2_pending, key2_long_done;
    reg combo_candidate, combo_fired;
    reg gesture_locked;

    assign key1_stable_pressed = ~key1_stable;
    assign key2_stable_pressed = ~key2_stable;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key1_meta <= 1'b1; key1_sync <= 1'b1;
            key2_meta <= 1'b1; key2_sync <= 1'b1;
            key1_stable <= 1'b1; key2_stable <= 1'b1;
            key1_stable_d <= 1'b1; key2_stable_d <= 1'b1;
            key1_db_cnt <= {DB_W{1'b0}};
            key2_db_cnt <= {DB_W{1'b0}};
            combo_cnt <= {LONG_W{1'b0}};
            key2_cnt <= {LONG_W{1'b0}};
            key1_pending <= 1'b0; key2_pending <= 1'b0;
            key2_long_done <= 1'b0;
            combo_candidate <= 1'b0; combo_fired <= 1'b0;
            gesture_locked <= 1'b0;
            key1_short_event <= 1'b0; key2_short_event <= 1'b0;
            key2_long_event <= 1'b0; combo_long_event <= 1'b0;
        end else begin
            key1_meta <= key1;
            key1_sync <= key1_meta;
            key2_meta <= key2;
            key2_sync <= key2_meta;
            key1_stable_d <= key1_stable;
            key2_stable_d <= key2_stable;

            if (key1_sync == key1_stable) begin
                key1_db_cnt <= {DB_W{1'b0}};
            end else if (key1_db_cnt >= DEBOUNCE_CYCLES-1) begin
                key1_stable <= key1_sync;
                key1_db_cnt <= {DB_W{1'b0}};
            end else begin
                key1_db_cnt <= key1_db_cnt + 1'b1;
            end
            if (key2_sync == key2_stable) begin
                key2_db_cnt <= {DB_W{1'b0}};
            end else if (key2_db_cnt >= DEBOUNCE_CYCLES-1) begin
                key2_stable <= key2_sync;
                key2_db_cnt <= {DB_W{1'b0}};
            end else begin
                key2_db_cnt <= key2_db_cnt + 1'b1;
            end

            key1_short_event <= 1'b0;
            key2_short_event <= 1'b0;
            key2_long_event <= 1'b0;
            combo_long_event <= 1'b0;

            // A committed long event owns the gesture until both keys release.
            if (gesture_locked) begin
                key1_pending <= 1'b0;
                key2_pending <= 1'b0;
                key2_long_done <= 1'b0;
                key2_cnt <= {LONG_W{1'b0}};
                combo_candidate <= 1'b0;
                combo_fired <= 1'b0;
                combo_cnt <= {LONG_W{1'b0}};
                if (key1_stable && key2_stable)
                    gesture_locked <= 1'b0;
            end else if ((!key1_stable && !key2_stable) || combo_candidate) begin
                // Stable overlap wins even on the KEY2 long threshold cycle.
                key1_pending <= 1'b0;
                key2_pending <= 1'b0;
                key2_long_done <= 1'b1;
                key2_cnt <= {LONG_W{1'b0}};
                if (!key1_stable && !key2_stable) begin
                    if (!combo_candidate) begin
                        combo_candidate <= 1'b1;
                        combo_fired <= 1'b0;
                        combo_cnt <= {LONG_W{1'b0}};
                    end else if (!combo_fired) begin
                        if (combo_cnt >= LONG_CYCLES-1) begin
                            combo_long_event <= 1'b1;
                            combo_fired <= 1'b1;
                            gesture_locked <= 1'b1;
                        end else begin
                            combo_cnt <= combo_cnt + 1'b1;
                        end
                    end
                end else begin
                    // Interrupted overlap cancels timing, but retains ownership.
                    combo_cnt <= {LONG_W{1'b0}};
                    if (key1_stable && key2_stable) begin
                        combo_candidate <= 1'b0;
                        combo_fired <= 1'b0;
                        key2_long_done <= 1'b0;
                    end
                end
            end else begin
                if (!key1_stable && key1_stable_d)
                    key1_pending <= 1'b1;
                if (!key2_stable && key2_stable_d) begin
                    key2_pending <= 1'b1;
                    key2_long_done <= 1'b0;
                    key2_cnt <= {LONG_W{1'b0}};
                end

                if (key1_stable && !key1_stable_d) begin
                    if (key1_pending)
                        key1_short_event <= 1'b1;
                    key1_pending <= 1'b0;
                end
                if (key2_stable && !key2_stable_d) begin
                    if (key2_pending && !key2_long_done)
                        key2_short_event <= 1'b1;
                    key2_pending <= 1'b0;
                    key2_long_done <= 1'b0;
                    key2_cnt <= {LONG_W{1'b0}};
                end

                if (!key2_stable && !key2_long_done) begin
                    if (key2_cnt >= LONG_CYCLES-1) begin
                        key2_long_event <= 1'b1;
                        key2_long_done <= 1'b1;
                        gesture_locked <= 1'b1;
                        key1_pending <= 1'b0;
                        key2_pending <= 1'b0;
                    end else begin
                        key2_cnt <= key2_cnt + 1'b1;
                    end
                end
            end
        end
    end
endmodule
