// Fixed 4096 x 16 dual-clock PCM FIFO. Read requests return registered data.
module audio_pcm_fifo (
    input  wire               wr_clk,
    input  wire               rd_clk,
    input  wire               rst_n,
    input  wire               pcm_in_valid,
    input  wire signed [15:0] pcm_in_data,
    input  wire               pcm_rd_en,
    output reg                pcm_out_valid,
    output reg signed [15:0]  pcm_out_data,
    output reg                fifo_empty,
    output reg                fifo_full,
    output wire [12:0]        fifo_level
);

    // Asynchronous assertion, independent two-clock synchronous release.
    (* ASYNC_REG = "TRUE" *) reg [1:0] wr_reset_pipe;
    (* ASYNC_REG = "TRUE" *) reg [1:0] rd_reset_pipe;
    wire wr_rst_n = wr_reset_pipe[1];
    wire rd_rst_n = rd_reset_pipe[1];

    always @(posedge wr_clk or negedge rst_n) begin
        if (!rst_n)
            wr_reset_pipe <= 2'b00;
        else
            wr_reset_pipe <= {wr_reset_pipe[0], 1'b1};
    end

    always @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n)
            rd_reset_pipe <= 2'b00;
        else
            rd_reset_pipe <= {rd_reset_pipe[0], 1'b1};
    end

    reg [12:0] wr_bin;
    reg [12:0] wr_gray;
    reg [12:0] rd_bin;
    reg [12:0] rd_gray;
    (* ASYNC_REG = "TRUE" *) reg [12:0] rd_gray_wr_meta;
    (* ASYNC_REG = "TRUE" *) reg [12:0] rd_gray_wr_sync;
    (* ASYNC_REG = "TRUE" *) reg [12:0] wr_gray_rd_meta;
    (* ASYNC_REG = "TRUE" *) reg [12:0] wr_gray_rd_sync;

    wire wr_accept = wr_rst_n && pcm_in_valid && !fifo_full;
    wire rd_accept = rd_rst_n && pcm_rd_en && !fifo_empty;
    wire [12:0] wr_bin_next = wr_bin + (wr_accept ? 13'd1 : 13'd0);
    wire [12:0] rd_bin_next = rd_bin + (rd_accept ? 13'd1 : 13'd0);
    wire [12:0] wr_gray_next = (wr_bin_next >> 1) ^ wr_bin_next;
    wire [12:0] rd_gray_next = (rd_bin_next >> 1) ^ rd_bin_next;
    wire wr_full_next = (wr_gray_next ==
                          {~rd_gray_wr_sync[12:11], rd_gray_wr_sync[10:0]});
    wire rd_empty_next = (rd_gray_next == wr_gray_rd_sync);

    function [12:0] gray_to_binary;
        input [12:0] gray_value;
        integer bit_index;
        begin
            gray_to_binary[12] = gray_value[12];
            for (bit_index = 11; bit_index >= 0; bit_index = bit_index - 1)
                gray_to_binary[bit_index] =
                    gray_to_binary[bit_index + 1] ^ gray_value[bit_index];
        end
    endfunction

    // The delayed read pointer makes this a conservative RAM occupancy.
    assign fifo_level = wr_bin - gray_to_binary(rd_gray_wr_sync);

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rd_gray_wr_meta <= 13'd0;
            rd_gray_wr_sync <= 13'd0;
        end else begin
            rd_gray_wr_meta <= rd_gray;
            rd_gray_wr_sync <= rd_gray_wr_meta;
        end
    end

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            wr_gray_rd_meta <= 13'd0;
            wr_gray_rd_sync <= 13'd0;
        end else begin
            wr_gray_rd_meta <= wr_gray;
            wr_gray_rd_sync <= wr_gray_rd_meta;
        end
    end

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_bin <= 13'd0;
            wr_gray <= 13'd0;
            fifo_full <= 1'b0;
        end else begin
            wr_bin <= wr_bin_next;
            wr_gray <= wr_gray_next;
            fifo_full <= wr_full_next;
        end
    end

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_bin <= 13'd0;
            rd_gray <= 13'd0;
            fifo_empty <= 1'b1;
            pcm_out_valid <= 1'b0;
        end else begin
            rd_bin <= rd_bin_next;
            rd_gray <= rd_gray_next;
            fifo_empty <= rd_empty_next;
            pcm_out_valid <= rd_accept;
        end
    end

    // No RAM or data-register reset: retain the dual-clock block RAM pattern.
    // pcm_out_data is meaningful only while pcm_out_valid is asserted.
    reg signed [15:0] pcm_ram [0:4095];
    always @(posedge wr_clk) begin
        if (wr_accept)
            pcm_ram[wr_bin[11:0]] <= pcm_in_data;
    end

    always @(posedge rd_clk) begin
        if (rd_accept)
            pcm_out_data <= pcm_ram[rd_bin[11:0]];
    end

endmodule
