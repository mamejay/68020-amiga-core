// Gate-level simulation models for syn/gl (yosys ships these two ECP5 cells as black boxes).
// Only what the m68020 core netlist uses: DP16KD in 36-bit pseudo-dual-port mode (port A writes,
// byte enables on ADA[3:0]; port B reads, no output register) and a combinational MULT18X18D.
`include "common_sim.vh"
`include "ccu2c_sim.vh"

module MULT18X18D(input A0, input A1, input A2, input A3, input A4, input A5, input A6, input A7, input A8, input A9, input A10, input A11, input A12, input A13, input A14, input A15, input A16, input A17, input B0, input B1, input B2, input B3, input B4, input B5, input B6, input B7, input B8, input B9, input B10, input B11, input B12, input B13, input B14, input B15, input B16, input B17, input C0, input C1, input C2, input C3, input C4, input C5, input C6, input C7, input C8, input C9, input C10, input C11, input C12, input C13, input C14, input C15, input C16, input C17,
    input SIGNEDA, SIGNEDB, SOURCEA, SOURCEB, output P0, output P1, output P2, output P3, output P4, output P5, output P6, output P7, output P8, output P9, output P10, output P11, output P12, output P13, output P14, output P15, output P16, output P17, output P18, output P19, output P20, output P21, output P22, output P23, output P24, output P25, output P26, output P27, output P28, output P29, output P30, output P31, output P32, output P33, output P34, output P35);
  wire [17:0] a = {A17, A16, A15, A14, A13, A12, A11, A10, A9, A8, A7, A6, A5, A4, A3, A2, A1, A0}, b = {B17, B16, B15, B14, B13, B12, B11, B10, B9, B8, B7, B6, B5, B4, B3, B2, B1, B0};
  wire signed [36:0] sa = SIGNEDA ? {{19{a[17]}}, a} : {19'd0, a};
  wire signed [36:0] sb = SIGNEDB ? {{19{b[17]}}, b} : {19'd0, b};
  wire signed [73:0] p = sa * sb;
  assign {P35, P34, P33, P32, P31, P30, P29, P28, P27, P26, P25, P24, P23, P22, P21, P20, P19, P18, P17, P16, P15, P14, P13, P12, P11, P10, P9, P8, P7, P6, P5, P4, P3, P2, P1, P0} = p[35:0];
endmodule

module DP16KD(input DIA0, input DIA1, input DIA2, input DIA3, input DIA4, input DIA5, input DIA6, input DIA7, input DIA8, input DIA9, input DIA10, input DIA11, input DIA12, input DIA13, input DIA14, input DIA15, input DIA16, input DIA17, input ADA0, input ADA1, input ADA2, input ADA3, input ADA4, input ADA5, input ADA6, input ADA7, input ADA8, input ADA9, input ADA10, input ADA11, input ADA12, input ADA13, input CEA, OCEA, CLKA, WEA, RSTA, CSA2, CSA1, CSA0, output reg DOA0, output reg DOA1, output reg DOA2, output reg DOA3, output reg DOA4, output reg DOA5, output reg DOA6, output reg DOA7, output reg DOA8, output reg DOA9, output reg DOA10, output reg DOA11, output reg DOA12, output reg DOA13, output reg DOA14, output reg DOA15, output reg DOA16, output reg DOA17, input DIB0, input DIB1, input DIB2, input DIB3, input DIB4, input DIB5, input DIB6, input DIB7, input DIB8, input DIB9, input DIB10, input DIB11, input DIB12, input DIB13, input DIB14, input DIB15, input DIB16, input DIB17, input ADB0, input ADB1, input ADB2, input ADB3, input ADB4, input ADB5, input ADB6, input ADB7, input ADB8, input ADB9, input ADB10, input ADB11, input ADB12, input ADB13, input CEB, OCEB, CLKB, WEB, RSTB, CSB2, CSB1, CSB0, output reg DOB0, output reg DOB1, output reg DOB2, output reg DOB3, output reg DOB4, output reg DOB5, output reg DOB6, output reg DOB7, output reg DOB8, output reg DOB9, output reg DOB10, output reg DOB11, output reg DOB12, output reg DOB13, output reg DOB14, output reg DOB15, output reg DOB16, output reg DOB17);
  parameter DATA_WIDTH_A = 18, DATA_WIDTH_B = 18;
  parameter REGMODE_A = "NOREG", REGMODE_B = "NOREG", RESETMODE = "SYNC", ASYNC_RESET_RELEASE = "SYNC";
  parameter CSDECODE_A = "0b000", CSDECODE_B = "0b000", WRITEMODE_A = "NORMAL", WRITEMODE_B = "NORMAL";
  parameter CLKAMUX = "CLKA", CLKBMUX = "CLKB", GSR = "ENABLED", INIT_DATA = "STATIC";
  parameter INITVAL_00 = 320'h0;
  parameter INITVAL_01 = 320'h0;
  parameter INITVAL_02 = 320'h0;
  parameter INITVAL_03 = 320'h0;
  parameter INITVAL_04 = 320'h0;
  parameter INITVAL_05 = 320'h0;
  parameter INITVAL_06 = 320'h0;
  parameter INITVAL_07 = 320'h0;
  parameter INITVAL_08 = 320'h0;
  parameter INITVAL_09 = 320'h0;
  parameter INITVAL_0A = 320'h0;
  parameter INITVAL_0B = 320'h0;
  parameter INITVAL_0C = 320'h0;
  parameter INITVAL_0D = 320'h0;
  parameter INITVAL_0E = 320'h0;
  parameter INITVAL_0F = 320'h0;
  parameter INITVAL_10 = 320'h0;
  parameter INITVAL_11 = 320'h0;
  parameter INITVAL_12 = 320'h0;
  parameter INITVAL_13 = 320'h0;
  parameter INITVAL_14 = 320'h0;
  parameter INITVAL_15 = 320'h0;
  parameter INITVAL_16 = 320'h0;
  parameter INITVAL_17 = 320'h0;
  parameter INITVAL_18 = 320'h0;
  parameter INITVAL_19 = 320'h0;
  parameter INITVAL_1A = 320'h0;
  parameter INITVAL_1B = 320'h0;
  parameter INITVAL_1C = 320'h0;
  parameter INITVAL_1D = 320'h0;
  parameter INITVAL_1E = 320'h0;
  parameter INITVAL_1F = 320'h0;
  parameter INITVAL_20 = 320'h0;
  parameter INITVAL_21 = 320'h0;
  parameter INITVAL_22 = 320'h0;
  parameter INITVAL_23 = 320'h0;
  parameter INITVAL_24 = 320'h0;
  parameter INITVAL_25 = 320'h0;
  parameter INITVAL_26 = 320'h0;
  parameter INITVAL_27 = 320'h0;
  parameter INITVAL_28 = 320'h0;
  parameter INITVAL_29 = 320'h0;
  parameter INITVAL_2A = 320'h0;
  parameter INITVAL_2B = 320'h0;
  parameter INITVAL_2C = 320'h0;
  parameter INITVAL_2D = 320'h0;
  parameter INITVAL_2E = 320'h0;
  parameter INITVAL_2F = 320'h0;
  parameter INITVAL_30 = 320'h0;
  parameter INITVAL_31 = 320'h0;
  parameter INITVAL_32 = 320'h0;
  parameter INITVAL_33 = 320'h0;
  parameter INITVAL_34 = 320'h0;
  parameter INITVAL_35 = 320'h0;
  parameter INITVAL_36 = 320'h0;
  parameter INITVAL_37 = 320'h0;
  parameter INITVAL_38 = 320'h0;
  parameter INITVAL_39 = 320'h0;
  parameter INITVAL_3A = 320'h0;
  parameter INITVAL_3B = 320'h0;
  parameter INITVAL_3C = 320'h0;
  parameter INITVAL_3D = 320'h0;
  parameter INITVAL_3E = 320'h0;
  parameter INITVAL_3F = 320'h0;
  reg [35:0] mem [0:511];
  integer i;
  initial begin
    for (i = 0; i < 512; i = i + 1) mem[i] = 36'h0;
    if (DATA_WIDTH_A != 36 || DATA_WIDTH_B != 36 || REGMODE_B != "NOREG" || CLKAMUX != "CLKA" || CLKBMUX != "CLKB")
      $fatal(1, "DP16KD model: only 36-bit PDP, NOREG, plain clocks");
  end
  wire [35:0] di = {{DIB17, DIB16, DIB15, DIB14, DIB13, DIB12, DIB11, DIB10, DIB9, DIB8, DIB7, DIB6, DIB5, DIB4, DIB3, DIB2, DIB1, DIB0}, {DIA17, DIA16, DIA15, DIA14, DIA13, DIA12, DIA11, DIA10, DIA9, DIA8, DIA7, DIA6, DIA5, DIA4, DIA3, DIA2, DIA1, DIA0}};
  wire [8:0]  wa = {ADA13, ADA12, ADA11, ADA10, ADA9, ADA8, ADA7, ADA6, ADA5};
  wire [8:0]  ra = {ADB13, ADB12, ADB11, ADB10, ADB9, ADB8, ADB7, ADB6, ADB5};
  wire [3:0]  be = {ADA3, ADA2, ADA1, ADA0};
  reg  [35:0] q;
  always @(posedge CLKA) if (CEA && WEA) begin
    if (be[0]) mem[wa][8:0]   <= di[8:0];
    if (be[1]) mem[wa][17:9]  <= di[17:9];
    if (be[2]) mem[wa][26:18] <= di[26:18];
    if (be[3]) mem[wa][35:27] <= di[35:27];
  end
  always @(posedge CLKB) if (CEB) q <= mem[ra];     // read-before-write on a collision
  always @* {{DOB17, DOB16, DOB15, DOB14, DOB13, DOB12, DOB11, DOB10, DOB9, DOB8, DOB7, DOB6, DOB5, DOB4, DOB3, DOB2, DOB1, DOB0}, {DOA17, DOA16, DOA15, DOA14, DOA13, DOA12, DOA11, DOA10, DOA9, DOA8, DOA7, DOA6, DOA5, DOA4, DOA3, DOA2, DOA1, DOA0}} = q;
endmodule
