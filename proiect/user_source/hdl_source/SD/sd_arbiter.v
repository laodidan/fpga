`timescale 1ns/1ps

// System-level SD sector arbiter.  The active top-level SD path contains one
// controller below this module.  A client owns one complete 512-byte read
// transaction; ownership is released only on sd_sec_read_end.
module sd_arbiter #(
    parameter integer SPI_LOW_SPEED_DIV = 248,
    parameter integer SPI_HIGH_SPEED_DIV = 0
) (
    input  wire        clk,
    input  wire        rst,

    output wire        SD_nCS,
    output wire        SD_DCLK,
    output wire        SD_MOSI,
    input  wire        SD_MISO,
    output wire        sd_init_done,

    input  wire        bmp_req,
    input  wire [31:0] bmp_lba,
    output wire        bmp_busy,
    output wire [7:0]  bmp_data,
    output wire        bmp_valid,
    output wire        bmp_done,
    output wire [7:0]  bmp_error,

    // Reserved audio client interface.  audio_req is held until audio_start.
    input  wire        audio_req,
    input  wire [31:0] audio_lba,
    output wire        audio_start,
    output wire [7:0]  audio_data,
    output wire        audio_valid,
    output wire        audio_done
);
    localparam OWNER_NONE  = 2'd0;
    localparam OWNER_BMP   = 2'd1;
    localparam OWNER_AUDIO = 2'd2;

    reg [1:0] owner;
    reg       last_grant_audio;
    reg       sd_sec_read_reg;
    reg [31:0] sd_sec_read_addr_reg;
    reg       audio_start_reg;
    reg       bmp_done_reg;
    reg       audio_done_reg;
    reg       bmp_req_armed;
    reg       audio_req_armed;

    wire [7:0] sd_data;
    wire       sd_data_valid;
    wire       sd_read_end;
    wire       grant_audio = audio_req && audio_req_armed &&
                             (!bmp_req || !last_grant_audio);
    wire       grant_bmp   = bmp_req && bmp_req_armed &&
                             (!audio_req || last_grant_audio);

    assign bmp_busy = owner == OWNER_BMP;
    assign bmp_data = sd_data;
    assign bmp_valid = (owner == OWNER_BMP) && sd_data_valid;
    assign bmp_done = bmp_done_reg;
    assign bmp_error = 8'h00;
    assign audio_start = audio_start_reg;
    assign audio_data = sd_data;
    assign audio_valid = (owner == OWNER_AUDIO) && sd_data_valid;
    assign audio_done = audio_done_reg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            owner <= OWNER_NONE;
            last_grant_audio <= 1'b0;
            sd_sec_read_reg <= 1'b0;
            sd_sec_read_addr_reg <= 32'd0;
            audio_start_reg <= 1'b0;
            bmp_done_reg <= 1'b0;
            audio_done_reg <= 1'b0;
            bmp_req_armed <= 1'b1;
            audio_req_armed <= 1'b1;
        end else begin
            audio_start_reg <= 1'b0;
            bmp_done_reg <= 1'b0;
            audio_done_reg <= 1'b0;
            if (!bmp_req)
                bmp_req_armed <= 1'b1;
            if (!audio_req)
                audio_req_armed <= 1'b1;

            case (owner)
                OWNER_NONE: begin
                    sd_sec_read_reg <= 1'b0;
                    if (sd_init_done && grant_audio) begin
                        owner <= OWNER_AUDIO;
                        last_grant_audio <= 1'b1;
                        sd_sec_read_addr_reg <= audio_lba;
                        sd_sec_read_reg <= 1'b1;
                        audio_start_reg <= 1'b1;
                        audio_req_armed <= 1'b0;
                    end else if (sd_init_done && grant_bmp) begin
                        owner <= OWNER_BMP;
                        last_grant_audio <= 1'b0;
                        sd_sec_read_addr_reg <= bmp_lba;
                        sd_sec_read_reg <= 1'b1;
                        bmp_req_armed <= 1'b0;
                    end
                end
                OWNER_BMP: begin
                    if (sd_read_end) begin
                        sd_sec_read_reg <= 1'b0;
                        owner <= OWNER_NONE;
                        bmp_done_reg <= 1'b1;
                    end
                end
                OWNER_AUDIO: begin
                    if (sd_read_end) begin
                        sd_sec_read_reg <= 1'b0;
                        owner <= OWNER_NONE;
                        audio_done_reg <= 1'b1;
                    end
                end
                default: begin
                    owner <= OWNER_NONE;
                    sd_sec_read_reg <= 1'b0;
                end
            endcase
        end
    end

    sd_card_top #(
        .SPI_LOW_SPEED_DIV(SPI_LOW_SPEED_DIV),
        .SPI_HIGH_SPEED_DIV(SPI_HIGH_SPEED_DIV)
    ) sd_card_top_m0 (
        .clk                    (clk),
        .rst                    (rst),
        .SD_nCS                 (SD_nCS),
        .SD_DCLK                (SD_DCLK),
        .SD_MOSI                (SD_MOSI),
        .SD_MISO                (SD_MISO),
        .sd_init_done           (sd_init_done),
        .sd_sec_read            (sd_sec_read_reg),
        .sd_sec_read_addr       (sd_sec_read_addr_reg),
        .sd_sec_read_data       (sd_data),
        .sd_sec_read_data_valid (sd_data_valid),
        .sd_sec_read_end        (sd_read_end),
        .sd_sec_write           (1'b0),
        .sd_sec_write_addr      (32'd0),
        .sd_sec_write_data      (8'd0),
        .sd_sec_write_data_req  (),
        .sd_sec_write_end       ()
    );
endmodule
