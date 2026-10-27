module reset_sync (
    input  wire clk,
    input  wire ext_rst_n,
    output wire rst_n_sync
);

    // Async assert, sync deassert reset synchronizer
    (* ASYNC_REG = "TRUE" *) reg rst_sync_ff1;
    (* ASYNC_REG = "TRUE" *) reg rst_sync_ff2;

    always @(posedge clk or negedge ext_rst_n) begin
        if (!ext_rst_n) begin
            rst_sync_ff1 <= 1'b0;
            rst_sync_ff2 <= 1'b0;
        end
        else begin
            rst_sync_ff1 <= 1'b1;
            rst_sync_ff2 <= rst_sync_ff1;
        end
    end

    assign rst_n_sync = rst_sync_ff2;

endmodule