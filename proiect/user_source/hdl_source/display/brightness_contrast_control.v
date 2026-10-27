// Combinational brightness and contrast selection from four user switches.
module brightness_contrast_control (
    input  wire       sw1,
    input  wire       sw2,
    input  wire       sw3,
    input  wire       sw4,
    output wire [7:0] brightness_cfg,
    output wire [7:0] contrast_cfg
);

    // SW1/SW2: 00=255, 01=192, 10=128, 11=64.
    assign brightness_cfg = ({sw1, sw2} == 2'b00) ? 8'd255 :
                            ({sw1, sw2} == 2'b01) ? 8'd192 :
                            ({sw1, sw2} == 2'b10) ? 8'd128 :
                                                     8'd64;

    // SW3/SW4: 00=128, 01=160, 10=96, 11=192.
    assign contrast_cfg = ({sw3, sw4} == 2'b00) ? 8'd128 :
                          ({sw3, sw4} == 2'b01) ? 8'd160 :
                          ({sw3, sw4} == 2'b10) ? 8'd96 :
                                                   8'd192;

endmodule
