`include "mux.v"
`include "register.v"
`include "binary_decoder.v"

module RegisterFile(
    output [31:0] PA, PB, // 2 Output Ports
    input [31:0] PW, 
    input [4:0] RA, RB, RW, // Read Inputs
    input LE, clk   
);

wire [31:0] E;
wire [31:0] I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10,
		I11, I12, I13, I14, I15, I16, I17, I18, I19, I20,
		I21, I22, I23, I24, I25, I26, I27, I28, I29, I30,
		I31;

  // Instantiate 16 Register modules
//Register reg0 (.Q(I0),  .D(PW), .clk(clk), .Ld(E[0]));
//Register reg1 (.Q(I1),  .D(PW), .clk(clk), .Ld(E[1]));
//Register reg2 (.Q(I2),  .D(PW), .clk(clk), .Ld(E[2]));
//Register reg3 (.Q(I3),  .D(PW), .clk(clk), .Ld(E[3]));
//Register reg4 (.Q(I4),  .D(PW), .clk(clk), .Ld(E[4]));
//Register reg5 (.Q(I5),  .D(PW), .clk(clk), .Ld(E[5]));
//Register reg6 (.Q(I6),  .D(PW), .clk(clk), .Ld(E[6]));
//Register reg7 (.Q(I7),  .D(PW), .clk(clk), .Ld(E[7]));
//Register reg8 (.Q(I8),  .D(PW), .clk(clk), .Ld(E[8]));
//Register reg9 (.Q(I9),  .D(PW), .clk(clk), .Ld(E[9]));
//Register reg10 (.Q(I10), .D(PW), .clk(clk), .Ld(E[10]));
//Register reg11 (.Q(I11), .D(PW), .clk(clk), .Ld(E[11]));
//Register reg12 (.Q(I12), .D(PW), .clk(clk), .Ld(E[12]));
//Register reg13 (.Q(I13), .D(PW), .clk(clk), .Ld(E[13]));
//Register reg14 (.Q(I14), .D(PW), .clk(clk), .Ld(E[14]));
//Register reg15 (.Q(I15), .D(PW), .clk(clk), .Ld(E[15]));
//Register reg16 (.Q(I16), .D(PW), .clk(clk), .Ld(E[16]));
//Register reg17 (.Q(I17), .D(PW), .clk(clk), .Ld(E[17]));
//Register reg18 (.Q(I18), .D(PW), .clk(clk), .Ld(E[18]));
//Register reg19 (.Q(I19), .D(PW), .clk(clk), .Ld(E[19]));
//Register reg20 (.Q(I20), .D(PW), .clk(clk), .Ld(E[20]));
//Register reg21 (.Q(I21), .D(PW), .clk(clk), .Ld(E[21]));
//Register reg22 (.Q(I22), .D(PW), .clk(clk), .Ld(E[22]));
//Register reg23 (.Q(I23), .D(PW), .clk(clk), .Ld(E[23]));
//Register reg24 (.Q(I24), .D(PW), .clk(clk), .Ld(E[24]));
//Register reg25 (.Q(I25), .D(PW), .clk(clk), .Ld(E[25]));
//Register reg26 (.Q(I26), .D(PW), .clk(clk), .Ld(E[26]));
//Register reg27 (.Q(I27), .D(PW), .clk(clk), .Ld(E[27]));
//Register reg28 (.Q(I28), .D(PW), .clk(clk), .Ld(E[28]));
//Register reg29 (.Q(I29), .D(PW), .clk(clk), .Ld(E[29]));
//Register reg30 (.Q(I30), .D(PW), .clk(clk), .Ld(E[30]));
//Register reg31 (.Q(I31), .D(PW), .clk(clk), .Ld(E[31]));

BinaryDecoder Decoder (
        E, RW, LE
    );

	mux_32x1 muxPA(
		PA, RA, 
		32'b0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10,
		I11, I12, I13, I14, I15, I16, I17, I18, I19, I20,
		I21, I22, I23, I24, I25, I26, I27, I28, I29, I30,
		I31);

	mux_32x1 muxPB(
		PB, RB, 
		32'b0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10,
		I11, I12, I13, I14, I15, I16, I17, I18, I19, I20,
		I21, I22, I23, I24, I25, I26, I27, I28, I29, I30,
		I31);

endmodule
