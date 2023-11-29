module Operand2_Handler(
    input [31:0] PB,
    input [31:0] HI,
    input [31:0] LO,
    input [31:0] PC,
    input [15:0] imm16,
    input [2:0] S,
    output reg [31:0] N
);

always @(*) begin
    case(S)
        3'b000: // S2=0, S1=0, S0=0 (PB)
            N = PB;
        3'b001: // S2=0, S1=0, S0=1 (HI)
            N = HI;
        3'b010: // S2=0, S1=1, S0=0 (LO)
            N = LO;
        3'b011: // S2=0, S1=1, S0=1 (PC)
            N = PC;
        3'b100: // S2=1, S1=0, S0=0 (imm16, sign-extended)
            N = {{16{imm16[15]}}, imm16};
        3'b101: // S2=1, S1=0, S0=1 (imm16 || 0x0000)
            N = {imm16, 16'b0};
        default: // S2=1, S1=1, S0=0 and S0=1 (Not used)
            N = 32'b0; // Undefined operation, output 0
    endcase
end

endmodule
