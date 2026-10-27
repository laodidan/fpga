module startup_ctrl (
    input  wire clk,
    input  wire rst_n,
    input  wire pll_locked,
    input  wire sd_init_done,
    input  wire sdram_init_done,
    output reg  player_enable
);

    localparam [1:0] WAIT_PLL   = 2'd0;
    localparam [1:0] WAIT_SD    = 2'd1;
    localparam [1:0] WAIT_SDRAM = 2'd2;
    localparam [1:0] RUN        = 2'd3;

    (* ASYNC_REG = "TRUE" *) reg pll_locked_meta;
    (* ASYNC_REG = "TRUE" *) reg pll_locked_sync;
    (* ASYNC_REG = "TRUE" *) reg sd_init_done_meta;
    (* ASYNC_REG = "TRUE" *) reg sd_init_done_sync;
    (* ASYNC_REG = "TRUE" *) reg sdram_init_done_meta;
    (* ASYNC_REG = "TRUE" *) reg sdram_init_done_sync;

    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pll_locked_meta <= 1'b0;
            pll_locked_sync <= 1'b0;
        end else begin
            pll_locked_meta <= pll_locked;
            pll_locked_sync <= pll_locked_meta;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sd_init_done_meta <= 1'b0;
            sd_init_done_sync <= 1'b0;
        end else begin
            sd_init_done_meta <= sd_init_done;
            sd_init_done_sync <= sd_init_done_meta;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sdram_init_done_meta <= 1'b0;
            sdram_init_done_sync <= 1'b0;
        end else begin
            sdram_init_done_meta <= sdram_init_done;
            sdram_init_done_sync <= sdram_init_done_meta;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= WAIT_PLL;
            player_enable <= 1'b0;
        end else begin
            player_enable <= 1'b0;
            if (!pll_locked_sync) begin
                state <= WAIT_PLL;
            end else begin
                case (state)
                    WAIT_PLL: begin
                        state <= WAIT_SD;
                    end
                    WAIT_SD: begin
                        if (sd_init_done_sync)
                            state <= WAIT_SDRAM;
                    end
                    WAIT_SDRAM: begin
                        if (!sd_init_done_sync) begin
                            state <= WAIT_SD;
                        end else if (sdram_init_done_sync) begin
                            state <= RUN;
                            player_enable <= 1'b1;
                        end
                    end
                    RUN: begin
                        if (!sd_init_done_sync) begin
                            state <= WAIT_SD;
                        end else if (!sdram_init_done_sync) begin
                            state <= WAIT_SDRAM;
                        end else begin
                            player_enable <= 1'b1;
                        end
                    end
                    default: begin
                        state <= WAIT_PLL;
                    end
                endcase
            end
        end
    end

endmodule
