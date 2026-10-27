`timescale 1ns/1ps
module w25q64_ctrl #(
    parameter integer SPI_DIV = 4,
    parameter integer STARTUP_CYCLES = 250000,
    parameter integer MAX_POLLS = 250000
) (
    input wire clk, rst_n,
    input wire req_valid,
    output wire req_ready,
    input wire [1:0] req_op,
    input wire [23:0] req_addr,
    input wire [31:0] req_wdata,
    output reg done,
    output reg [2:0] error,
    output reg [31:0] read_data,
    output reg [23:0] jedec_id,
    output reg FLASH_CS, FLASH_SCLK, FLASH_SDI,
    output wire FLASH_WP, FLASH_HOLD,
    input wire FLASH_SDO
);
    localparam BOOT=0, IDLE=1, ID_SEND=2, ID_WAIT=3,
        PRE_SEND=4, PRE_WAIT=5, WREN_SEND=6, WREN_WAIT=7,
        WEL_SEND=8, WEL_WAIT=9, OP_SEND=10, OP_WAIT=11,
        POST_SEND=12, POST_WAIT=13, COMPLETE=14;
    reg [3:0] state;
    reg [1:0] op;
    reg [23:0] addr;
    reg [31:0] data;
    reg [31:0] startup_count, poll_count;
    reg frame_start;
    reg [63:0] frame_tx;
    reg [6:0] frame_bits;
    reg frame_done;
    reg [63:0] frame_rx;
    localparam SPI_IDLE=0, SPI_SETUP=1, SPI_SHIFT=2, SPI_HOLD=3, SPI_GAP=4;
    reg [2:0] spi_state;
    reg [63:0] tx_shift;
    reg [6:0] bits_left;
    reg [31:0] divider;

    assign req_ready = (state == IDLE);
    // Keep active-low WP# and HOLD# inactive in standard SPI mode.
    assign FLASH_WP = 1'b1;
    assign FLASH_HOLD = 1'b1;

    task launch;
        input [63:0] payload;
        input [6:0] count;
        begin
            frame_tx <= payload;
            frame_bits <= count;
            frame_start <= 1'b1;
        end
    endtask

    // Finish only after CS hold and high time, so consecutive commands cannot overlap.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_state <= SPI_IDLE;
            FLASH_CS <= 1'b1;
            FLASH_SCLK <= 1'b0;
            FLASH_SDI <= 1'b0;
            tx_shift <= 64'd0;
            frame_rx <= 64'd0;
            bits_left <= 7'd0;
            divider <= 32'd0;
            frame_done <= 1'b0;
        end else begin
            frame_done <= 1'b0;
            case (spi_state)
                SPI_IDLE: if (frame_start) begin
                    FLASH_CS <= 1'b0;
                    FLASH_SDI <= frame_tx[63];
                    tx_shift <= frame_tx;
                    frame_rx <= 64'd0;
                    bits_left <= frame_bits;
                    divider <= 32'd0;
                    spi_state <= SPI_SETUP;
                end
                SPI_SETUP, SPI_SHIFT: begin
                    if (divider >= SPI_DIV-1) begin
                        divider <= 32'd0;
                        spi_state <= SPI_SHIFT;
                        if (!FLASH_SCLK) begin
                            FLASH_SCLK <= 1'b1;
                            frame_rx <= {frame_rx[62:0], FLASH_SDO};
                        end else begin
                            FLASH_SCLK <= 1'b0;
                            if (bits_left == 1) begin
                                spi_state <= SPI_HOLD;
                            end else begin
                                bits_left <= bits_left - 1'b1;
                                tx_shift <= {tx_shift[62:0], 1'b0};
                                FLASH_SDI <= tx_shift[62];
                            end
                        end
                    end else divider <= divider + 1'b1;
                end
                SPI_HOLD: begin
                    if (divider >= SPI_DIV-1) begin
                        FLASH_CS <= 1'b1;
                        FLASH_SDI <= 1'b0;
                        divider <= 32'd0;
                        spi_state <= SPI_GAP;
                    end else divider <= divider + 1'b1;
                end
                SPI_GAP: begin
                    if (divider >= SPI_DIV-1) begin
                        frame_done <= 1'b1;
                        spi_state <= SPI_IDLE;
                    end else divider <= divider + 1'b1;
                end
                default: spi_state <= SPI_IDLE;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= BOOT;
            startup_count <= 32'd0;
            poll_count <= 32'd0;
            op <= 2'd0;
            addr <= 24'd0;
            data <= 32'd0;
            done <= 1'b0;
            error <= 3'd0;
            read_data <= 32'd0;
            jedec_id <= 24'd0;
            frame_start <= 1'b0;
            frame_tx <= 64'd0;
            frame_bits <= 7'd0;
        end else begin
            done <= 1'b0;
            frame_start <= 1'b0;
            case (state)
                BOOT: begin
                    if (startup_count >= STARTUP_CYCLES-1) state <= IDLE;
                    else startup_count <= startup_count + 1'b1;
                end
                IDLE: if (req_valid) begin
                    op <= req_op;
                    addr <= req_addr;
                    data <= req_wdata;
                    error <= 3'd0;
                    poll_count <= 32'd0;
                    if (req_op != 0 && (req_addr[23] ||
                        (req_op == 1 && req_addr > 24'h7FFFFC) ||
                        (req_op == 2 && req_addr[11:0] != 0) ||
                        (req_op == 3 && req_addr[7:0] > 8'd252))) begin
                        error <= 3'd1;
                        state <= COMPLETE;
                    end else state <= PRE_SEND;
                end
                ID_SEND: begin
                    launch({8'h9F, 56'd0}, 7'd32);
                    state <= ID_WAIT;
                end
                ID_WAIT: if (frame_done) begin
                    jedec_id <= frame_rx[23:0];
                    state <= COMPLETE;
                end
                PRE_SEND: begin
                    launch({8'h05, 56'd0}, 7'd16);
                    state <= PRE_WAIT;
                end
                PRE_WAIT: if (frame_done) begin
                    if (!frame_rx[0]) begin
                        if (op == 0) state <= ID_SEND;
                        else if (op == 1) state <= OP_SEND;
                        else state <= WREN_SEND;
                    end else if (poll_count >= MAX_POLLS-1) begin
                        error <= 3'd2;
                        state <= COMPLETE;
                    end else begin
                        poll_count <= poll_count + 1'b1;
                        state <= PRE_SEND;
                    end
                end
                WREN_SEND: begin
                    launch({8'h06, 56'd0}, 7'd8);
                    state <= WREN_WAIT;
                end
                WREN_WAIT: if (frame_done) state <= WEL_SEND;
                WEL_SEND: begin
                    launch({8'h05, 56'd0}, 7'd16);
                    state <= WEL_WAIT;
                end
                WEL_WAIT: if (frame_done) begin
                    if (frame_rx[1] && !frame_rx[0]) state <= OP_SEND;
                    else begin
                        error <= 3'd3;
                        state <= COMPLETE;
                    end
                end
                OP_SEND: begin
                    case (op)
                        1: launch({8'h03, addr, 32'd0}, 7'd64);
                        2: launch({8'h20, addr, 32'd0}, 7'd32);
                        3: launch({8'h02, addr, data}, 7'd64);
                        default: launch(64'd0, 7'd8);
                    endcase
                    state <= OP_WAIT;
                end
                OP_WAIT: if (frame_done) begin
                    if (op == 1) begin
                        read_data <= frame_rx[31:0];
                        state <= COMPLETE;
                    end else begin
                        poll_count <= 32'd0;
                        state <= POST_SEND;
                    end
                end
                POST_SEND: begin
                    launch({8'h05, 56'd0}, 7'd16);
                    state <= POST_WAIT;
                end
                POST_WAIT: if (frame_done) begin
                    if (!frame_rx[0]) state <= COMPLETE;
                    else if (poll_count >= MAX_POLLS-1) begin
                        error <= 3'd2;
                        state <= COMPLETE;
                    end else begin
                        poll_count <= poll_count + 1'b1;
                        state <= POST_SEND;
                    end
                end
                COMPLETE: begin
                    done <= 1'b1;
                    state <= IDLE;
                end
                default: state <= BOOT;
            endcase
        end
    end
endmodule
