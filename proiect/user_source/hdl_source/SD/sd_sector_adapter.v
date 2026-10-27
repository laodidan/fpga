`timescale 1ns/1ps

// SD卡扇区读接口适配层。
// 将FAT32模块使用的sector_*接口转换为sd_card_top的sd_sec_*接口。
// 当前sd_card_top没有错误输出，因此sector_error固定为0。
module sd_sector_adapter (
    input  wire        clk,
    input  wire        rst_n,

    // FAT32模块侧统一扇区接口
    input  wire        sector_req,
    input  wire [31:0] sector_lba,
    output wire        sector_busy,
    output wire        sector_valid,
    output wire [7:0]  sector_byte,
    output wire [8:0]  sector_byte_index,
    output wire        sector_done,
    output wire [7:0]  sector_error,

    // sd_card_top侧扇区读接口
    output reg         sd_sec_read,
    output reg  [31:0] sd_sec_read_addr,
    input  wire        sd_sec_read_data_valid,
    input  wire [7:0]  sd_sec_read_data,
    input  wire        sd_sec_read_end
);
    reg [8:0] byte_cnt;
    reg       busy_reg;
    reg       req_armed;

    assign sector_busy       = busy_reg;
    assign sector_valid      = busy_reg && sd_sec_read_data_valid;
    assign sector_byte       = sd_sec_read_data;
    assign sector_byte_index = byte_cnt;
    assign sector_done       = busy_reg && sd_sec_read_end;
    // sd_card_top没有错误码接口；0仅表示适配层没有可报告的底层错误。
    assign sector_error      = 8'h00;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sd_sec_read      <= 1'b0;
            sd_sec_read_addr <= 32'd0;
            byte_cnt         <= 9'd0;
            busy_reg         <= 1'b0;
            req_armed         <= 1'b1;
        end else begin
            // 事务结束后，只有请求回到低电平才重新允许下一次请求。
            if (!busy_reg && !sector_req)
                req_armed <= 1'b1;

            if (sector_req && !busy_reg && req_armed) begin
                // sd_card_top在空闲状态检测该电平并启动一次读事务。
                sd_sec_read      <= 1'b1;
                sd_sec_read_addr <= sector_lba;
                byte_cnt         <= 9'd0;
                busy_reg         <= 1'b1;
                req_armed         <= 1'b0;
            end else if (sd_sec_read_end && busy_reg) begin
                sd_sec_read <= 1'b0;
                byte_cnt    <= 9'd0;
                busy_reg    <= 1'b0;
            end else if (busy_reg && sd_sec_read_data_valid) begin
                // 只在有效字节到达时计数；最后一个字节保持索引511。
                if (byte_cnt != 9'd511)
                    byte_cnt <= byte_cnt + 9'd1;
            end
        end
    end
endmodule
