module ALU (
    input wire [31:0] A,
    input wire [31:0] B,
    input wire [3:0] Opcode,
    output reg [31:0] Out,
    output reg Z,
    output reg N
);

always @(*) begin // @(*) means "always when any of the inputs change"
    case(Opcode)
        // A + B
        4'b0000:
            Out = A + B;
        // A - B
        4'b0001:
            Out = A - B;
        // A and B
        4'b0010:
            Out = A & B;
        // A or B
        4'b0011:
            Out = A | B;
        // A xor B
        4'b0100:
            Out = A ^ B;
        // A nor B
        4'b0101:
            Out = ~(A | B);
        // Shift Left Logical (B) A positions
        4'b0110:
            Out = A << B;
        // Shift Right Logical (B) A positions
        4'b0111:
            Out = A >> B;
        // Shift Right Arithmetic (B) A positions
        4'b1000:
            Out = $signed(A) >>> B;
        // If (A < B) then Out = 1, else Out = 0
        4'b1001:
            if (A < B)
                Out = 1;
            else
                Out = 0;
        // Pass A
        4'b1010:
            Out = A;
        // Pass B
        4'b1011:
            Out = B;
        // B + 8
        4'b1100:
            Out = B + 32'b00000000000000000000000000001000;
        // Not Used
        4'b1101, 4'b1110, 4'b1111:
            Out = 32'b0; // Undefined operation, output 0
        default:
            Out = 32'b0; // Undefined operation, output 0
    endcase

    // Generate flags
    if(Opcode == 4'b0000 || Opcode == 4'b0001 || Opcode == 4'b1001 || Opcode == 4'b1010 || Opcode == 4'b1011) begin
        Z = (Out == 32'b0) ? 1'b1 : 1'b0; //If Out is 0, then Z = 1
        N = (Out[31] == 1'b1) ? 1'b1 : 1'b0; //If Out is negative, then N = 1
    end
    else begin
        Z = 1'b0;
        N = 1'b0;
    end
end 

endmodule
