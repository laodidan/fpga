`timescale 1ns/1ps
module volume_flash_store #(
    parameter [23:0] SECTOR_ADDR = 24'h7FF000,
    parameter integer QUIET_CYCLES = 125000000,
    parameter integer SPI_DIV = 4,
    parameter integer STARTUP_CYCLES = 250000,
    parameter integer MAX_POLLS = 250000
) (
    input wire clk, rst_n, enable,
    input wire [7:0] volume_level,
    input wire volume_init_applied,
    output wire volume_init_valid,
    output wire [7:0] volume_init_value,
    output reg [7:0] saved_volume,
    output reg init_done,
    output wire save_busy,
    output reg save_done,
    output reg [3:0] store_error,
    output wire FLASH_CS, FLASH_SCLK, FLASH_SDI,
    output wire FLASH_WP, FLASH_HOLD,
    input wire FLASH_SDO
);
    localparam START=0, ID_REQ=1, ID_WAIT=2, READ_REQ=3, READ_WAIT=4,
        RUN=5, ERASE_REQ=6, ERASE_WAIT=7, WRITE_REQ=8, WRITE_WAIT=9,
        VERIFY_REQ=10, VERIFY_WAIT=11;
    reg [3:0] state;
    reg boot_known, restore_pending, allow_write, saved_valid, dirty, retry_block;
    reg [7:0] runtime_volume, snapshot;
    reg [31:0] quiet_count;
    wire req_ready, ctrl_done;
    wire [2:0] ctrl_error;
    wire [31:0] read_data;
    wire [23:0] jedec_id;
    wire req_valid = state == ID_REQ || state == READ_REQ ||
        state == ERASE_REQ || state == WRITE_REQ || state == VERIFY_REQ;
    wire [1:0] req_op = state == ID_REQ ? 2'd0 :
        state == ERASE_REQ ? 2'd2 : state == WRITE_REQ ? 2'd3 : 2'd1;
    wire [31:0] record = {8'h55, 8'hAA, snapshot, (8'hFF ^ snapshot)};
    wire record_valid = read_data[31:24] == 8'h55 && read_data[23:16] == 8'hAA &&
        read_data[7:0] == (8'hFF ^ read_data[15:8]);

    assign volume_init_valid = boot_known && enable && restore_pending;
    assign volume_init_value = runtime_volume;
    assign save_busy = state >= ERASE_REQ;

    w25q64_ctrl #(.SPI_DIV(SPI_DIV), .STARTUP_CYCLES(STARTUP_CYCLES),
        .MAX_POLLS(MAX_POLLS)) u_flash (
        .clk(clk), .rst_n(rst_n), .req_valid(req_valid), .req_ready(req_ready),
        .req_op(req_op), .req_addr(SECTOR_ADDR), .req_wdata(record),
        .done(ctrl_done), .error(ctrl_error), .read_data(read_data), .jedec_id(jedec_id),
        .FLASH_CS(FLASH_CS), .FLASH_SCLK(FLASH_SCLK),
        .FLASH_SDI(FLASH_SDI), .FLASH_SDO(FLASH_SDO),
        .FLASH_WP(FLASH_WP), .FLASH_HOLD(FLASH_HOLD)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= START;
            boot_known <= 1'b0;
            restore_pending <= 1'b1;
            allow_write <= 1'b0;
            saved_valid <= 1'b0;
            dirty <= 1'b0;
            retry_block <= 1'b0;
            runtime_volume <= 8'd255;
            snapshot <= 8'd255;
            quiet_count <= 32'd0;
            saved_volume <= 8'd255;
            init_done <= 1'b0;
            save_done <= 1'b0;
            store_error <= 4'd0;
        end else begin
            save_done <= 1'b0;
            // Cache survives audio-only reset; ignore its temporary default volume.
            if (!enable) begin
                restore_pending <= 1'b1;
                quiet_count <= 32'd0;
            end else if (boot_known) begin
                if (restore_pending) begin
                    if (volume_init_applied) begin
                        restore_pending <= 1'b0;
                        init_done <= 1'b1;
                    end
                end else if (volume_level != runtime_volume) begin
                    runtime_volume <= volume_level;
                    quiet_count <= 32'd0;
                    dirty <= 1'b1;
                    retry_block <= 1'b0;
                end else if (dirty && quiet_count < QUIET_CYCLES) begin
                    quiet_count <= quiet_count + 1'b1;
                end
            end

            case (state)
                START: if (enable) state <= ID_REQ;
                ID_REQ: if (req_ready) state <= ID_WAIT;
                ID_WAIT: if (ctrl_done) begin
                    if (ctrl_error != 0 || jedec_id != 24'hEF4017) begin
                        store_error <= 4'd1;
                        boot_known <= 1'b1;
                        state <= RUN;
                    end else begin
                        allow_write <= 1'b1;
                        state <= READ_REQ;
                    end
                end
                READ_REQ: if (req_ready) state <= READ_WAIT;
                READ_WAIT: if (ctrl_done) begin
                    boot_known <= 1'b1;
                    state <= RUN;
                    if (ctrl_error != 0) store_error <= 4'd3;
                    else if (!record_valid) store_error <= 4'd2;
                    else begin
                        runtime_volume <= read_data[15:8];
                        saved_volume <= read_data[15:8];
                        saved_valid <= 1'b1;
                    end
                end
                RUN: if (boot_known && enable && !restore_pending &&
                    volume_level == runtime_volume && dirty && !retry_block && allow_write) begin
                    if (saved_valid && runtime_volume == saved_volume) begin
                        dirty <= 1'b0;
                        quiet_count <= 32'd0;
                    end else if (quiet_count >= QUIET_CYCLES-1) begin
                        snapshot <= runtime_volume;
                        dirty <= 1'b0;
                        quiet_count <= 32'd0;
                        store_error <= 4'd0;
                        saved_valid <= 1'b0;
                        state <= ERASE_REQ;
                    end
                end
                ERASE_REQ: if (req_ready) state <= ERASE_WAIT;
                ERASE_WAIT: if (ctrl_done) begin
                    if (ctrl_error != 0) begin
                        store_error <= 4'd3;
                        retry_block <= 1'b1;
                        dirty <= 1'b0;
                        state <= RUN;
                    end else state <= WRITE_REQ;
                end
                WRITE_REQ: if (req_ready) state <= WRITE_WAIT;
                WRITE_WAIT: if (ctrl_done) begin
                    if (ctrl_error != 0) begin
                        store_error <= 4'd3;
                        retry_block <= 1'b1;
                        dirty <= 1'b0;
                        state <= RUN;
                    end else state <= VERIFY_REQ;
                end
                VERIFY_REQ: if (req_ready) state <= VERIFY_WAIT;
                VERIFY_WAIT: if (ctrl_done) begin
                    state <= RUN;
                    if (ctrl_error != 0 || read_data != record) begin
                        store_error <= ctrl_error != 0 ? 4'd3 : 4'd4;
                        retry_block <= 1'b1;
                        dirty <= 1'b0;
                        // Erase has destroyed the old record, so it is no longer a valid skip target.
                        saved_valid <= 1'b0;
                    end else begin
                        saved_volume <= snapshot;
                        saved_valid <= 1'b1;
                        save_done <= 1'b1;
                    end
                end
                default: state <= START;
            endcase
        end
    end
endmodule
