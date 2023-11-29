`include "mux.v"

module test_mux_32x1;
	//Selection lines (5)
	reg [4:0] S;

	//Inputs (32)
	reg [31:0] I0;
	reg [31:0] I1;
	reg [31:0] I2;
	reg [31:0] I3;
	reg [31:0] I4;
	reg [31:0] I5;
	reg [31:0] I6;
	reg [31:0] I7;
	reg [31:0] I8;
	reg [31:0] I9;
	reg [31:0] I10;
	reg [31:0] I11;
	reg [31:0] I12;
	reg [31:0] I13;
	reg [31:0] I14;
	reg [31:0] I15;
	reg [31:0] I16;
	reg [31:0] I17;
	reg [31:0] I18;
	reg [31:0] I19;
	reg [31:0] I20;
	reg [31:0] I21;
	reg [31:0] I22;
	reg [31:0] I23;
	reg [31:0] I24;
	reg [31:0] I25;
	reg [31:0] I26;
	reg [31:0] I27;
	reg [31:0] I28;
	reg [31:0] I29;
	reg [31:0] I30;
	reg [31:0] I31;

	//Output
	wire [31:0] Y;

	mux_32x1 mux3 (Y, S, I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15, I16, I17, I18, I19, I20, I21, I22, I23, I24, I25, I26, I27, I28, I29, I30, I31);

	initial begin
		S = 5'b00;

		I0 = 32'd0;
		I1 = 32'd1;
		I2 = 32'd2;
		I3 = 32'd3;
		I4 = 32'd4;
		I5 = 32'd5;
		I6 = 32'd6;
		I7 = 32'd7;
		I8 = 32'd8;
		I9 = 32'd9;
		I10 = 32'd10;
		I11 = 32'd11;
		I12 = 32'd12;
		I13 = 32'd13;
		I14 = 32'd14;
		I15 = 32'd15;
		I16 = 32'd16;
		I17 = 32'd17;
		I18 = 32'd18;
		I19 = 32'd19;
		I20 = 32'd20;
		I21 = 32'd21;
		I22 = 32'd22;
		I23 = 32'd23;
		I24 = 32'd24;
		I25 = 32'd25;
		I26 = 32'd26;
		I27 = 32'd27;
		I28 = 32'd28;
		I29 = 32'd29;
		I30 = 32'd30;
		I31 = 32'd31;
		
		$display ("\n32x1 Mux Test");
		
		$display ("S   	Y");

		$monitor ("%b   %h", S, Y);

		repeat (31) #1 begin
			S = S + 1;
		end

	end
endmodule