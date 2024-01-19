module mux_32x1  (
	output reg [31:0] Y, input [4:0] S, 
	input [31:0] I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15,
				 I16, I17, I18, I19, I20, I21, I22, I23, I24, I25, I26, I27, I28, I29, I30, I31
				 );


always @ (*)
	begin
	case (S)
		5'b00000: Y = I0;
		5'b00001: Y = I1;
		5'b00010: Y = I2;
		5'b00011: Y = I3;
		5'b00100: Y = I4;
		5'b00101: Y = I5;
		5'b00110: Y = I6;
		5'b00111: Y = I7;
		5'b01000: Y = I8;
		5'b01001: Y = I9;
		5'b01010: Y = I10;
		5'b01011: Y = I11;
		5'b01100: Y = I12;
		5'b01101: Y = I13;
		5'b01110: Y = I14;
		5'b01111: Y = I15;
		5'b10000: Y = I16;
		5'b10001: Y = I17;
		5'b10010: Y = I18;
		5'b10011: Y = I19;
		5'b10100: Y = I20;
		5'b10101: Y = I21;
		5'b10110: Y = I22;
		5'b10111: Y = I23;
		5'b11000: Y = I24;
		5'b11001: Y = I25;
		5'b11010: Y = I26;
		5'b11011: Y = I27;
		5'b11100: Y = I28;
		5'b11101: Y = I29;
		5'b11110: Y = I30;
		5'b11111: Y = I31;
	endcase
	end
endmodule

module  mux_4x1  (
	output reg [31:0] Y, input [1:0] S, 
	input [31:0] I0, I1, I2, I3
				 );
always @ (*) begin
	case (S)
		2'b00: Y = I0;
		2'b01: Y = I1;
		2'b10: Y = I2;
		2'b11: Y = I3;
	endcase
	end
endmodule

module  mux_3x1 (
	output reg [31:0] Y, input [2:0] S, 
	input [31:0] I0, I1, I2
				 );

always @ (*) begin
	case (S)
		3'b000: Y = I0;
		3'b001: Y = I1;
		3'b010: Y = I2;
	endcase
	end
endmodule

module mux_2x1 (
	output reg [31:0] Y, input S, 
	input [31:0] I0, I1
				 );

always @ (*) begin
	case (S)
		1'b0: Y = I0;
		1'b1: Y = I1;
	endcase
	end
endmodule

module TA_Mux (
	output reg [31:0] Y, input S, 
	input [31:0] I0, I1
				 );

always @ (*) begin
	case (S)
		1'b0: Y = I0;
		1'b1: Y = I1;
	endcase
	end
endmodule

module WB_Destination (
	input wire [4:0] rs,
	input wire [4:0] rt,
	input wire [4:0] rd,
	input wire [2:0] E, //CHANGE TO ENABLE SIGNAL
	output reg [4:0] destination
);

always @ (*) begin
	if (E == 3'b011) begin
		destination = 5'b11111;
	end
	else if (E == 3'b010) begin // change up a bit
		destination = rt;
	end
	else if (E == 3'b001)begin // change up a bit
		destination = rs;
	end
	else if (E == 3'b100)begin
		destination = rd;
	end
end

endmodule

module HI_MUX (
	input HI_Enable,
	input [31:0] HI,
	output reg [31:0] Y
);

always @ (*) begin
	if (HI_Enable) begin
		Y = HI;
	end
	else begin
		Y = 32'b0;
	end
end

endmodule

module LO_MUX (
	input LO_Enable,
	input [31:0] LO,
	output reg [31:0] Y
);

always @ (*) begin
	if (LO_Enable) begin
		Y = LO;
	end
	else begin
		Y = 32'b0;
	end
end

endmodule

module PC_Mux(
    input [31:0] nPC,
    input [31:0] TA,
    input [31:0] jump_target,
    input [1:0] select,
    output reg [31:0] Out
);
    always @(*) begin
        case (select)
            2'b00: Out = nPC;
            2'b01: Out = TA;
            //2'b10: Out = jump_target;
			//2'b10: Out =nPC;
            default: Out = 32'b0;
        endcase
    end
endmodule



module mux_32_Monitor (
    output reg [31:0] PA, PB,
    output reg [31:0] Y0, Y1, Y2, Y3, Y4, Y5, Y6, Y7, Y8, Y9,
    output reg [31:0] Y10, Y11, Y12, Y13, Y14, Y15, Y16, Y17, Y18, Y19,
    output reg [31:0] Y20, Y21, Y22, Y23, Y24, Y25, Y26, Y27, Y28, Y29,
    output reg [31:0] Y30, Y31,
    input [4:0] rs, rt,
    input [31:0] R0, R1, R2, R3, R4, R5, R6, R7, R8, R9,
    input [31:0] R10, R11, R12, R13, R14, R15, R16, R17, R18, R19,
    input [31:0] R20, R21, R22, R23, R24, R25, R26, R27, R28, R29,
    input [31:0] R30, R31
);
    
always @ (*)
begin
    PA = rs; Y0 = R0; Y1 = R1; Y2 = R2; Y3 = R3; Y4 = R4; Y5 = R5; Y6 = R6; Y7 = R7; Y8 = R8; Y9 = R9;
    Y10 = R10; Y11 = R11; Y12 = R12; Y13 = R13; Y14 = R14; Y15 = R15; Y16 = R16; Y17 = R17; Y18 = R18; Y19 = R19;
    Y20 = R20; Y21 = R21; Y22 = R22; Y23 = R23; Y24 = R24; Y25 = R25; Y26 = R26; Y27 = R27; Y28 = R28; Y29 = R29;
    Y30 = R30; Y31 = R31; PB = rt;
end

endmodule