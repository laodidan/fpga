// Two independent clock ports; contents and read output are not reset.
// The reader's tag/valid protocol masks uninitialized and refilling lines.
module thumbnail_line_ram (
    input  wire        wr_clk,
    input  wire        wr_en,
    input  wire [9:0]  wr_addr,
    input  wire [31:0] wr_data,
    input  wire        rd_clk,
    input  wire [9:0]  rd_addr,
    output reg  [31:0] rd_data
);
    (* ram_style = "block" *) reg [31:0] line_mem [0:1023];

    always @(posedge wr_clk) begin
        if (wr_en)
            line_mem[wr_addr] <= wr_data;
    end

    always @(posedge rd_clk) begin
        rd_data <= line_mem[rd_addr];
    end
endmodule
