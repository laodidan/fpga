// Stage-eight logical boundary only; the actual board wrapper remains top.
module stage8_top_skeleton (
    input wire clk, rst_n, enable,
    input wire [7:0] volume_level,
    input wire volume_init_applied, FLASH_SDO,
    output wire volume_init_valid,
    output wire [7:0] volume_init_value,
    output wire FLASH_CS, FLASH_SCLK, FLASH_SDI
);
    volume_flash_store u_store (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .volume_level(volume_level), .volume_init_applied(volume_init_applied),
        .volume_init_valid(volume_init_valid), .volume_init_value(volume_init_value),
        .saved_volume(), .init_done(), .save_busy(), .save_done(), .store_error(),
        .FLASH_CS(FLASH_CS), .FLASH_SCLK(FLASH_SCLK),
        .FLASH_SDI(FLASH_SDI), .FLASH_SDO(FLASH_SDO)
    );
endmodule
