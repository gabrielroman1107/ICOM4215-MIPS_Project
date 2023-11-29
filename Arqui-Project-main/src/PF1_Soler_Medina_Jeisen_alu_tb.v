`include "PF1_Soler_Medina_Jeisen_alu.v"

module ALU_tb;

    // Inputs
    reg [31:0] A;
    reg [31:0] B;
    reg [3:0] Opcode;

    // Outputs
    wire [31:0] Out;
    wire Z;
    wire N;

    // Instantiate the ALU module
    ALU uut (
        .A(A),
        .B(B),
        .Opcode(Opcode),
        .Out(Out),
        .Z(Z),
        .N(N)
    );

    initial begin
        // Test case 1: A + B
        A = 32'h00000001;
        B = 32'h00000002;
        Opcode = 4'b0000;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: 3

        // Test case 2: A - B
        A = 32'h00000003;
        B = 32'h00000005;
        Opcode = 4'b0001;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: 1

        // Test case 3: A and B
        A = 32'h0000000F;
        B = 32'h000000FF;
        Opcode = 4'b0010;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: 15

        // Test case 4: A or B
        A = 32'h0000000F;
        B = 32'h000000F0;
        Opcode = 4'b0011;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: 255

        // Test case 5: A xor B
        A = 32'h0000000F;
        B = 32'h000000F0;
        Opcode = 4'b0100;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: 241

        // Test case 6: A nor B
        A = 32'h0000000F;
        B = 32'h000000F0;
        Opcode = 4'b0101;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: -16

        // Test case 7: Shift Left Logical (B) A positions
        A = 32'h0000000F;
        B = 5;
        Opcode = 4'b0110;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: 480

        // Test case 8: Shift Right Logical (B) A positions
        A = 32'h0000000F;
        B = 5;
        Opcode = 4'b0111;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: 0

        // Test case 9: Shift Right Arithmetic (B) A positions
        A = 32'h80000000;
        B = 1;
        Opcode = 4'b1000;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: 2147483644

        // Test case 10: If (A < B) then Out = 1, else Out = 0
        A = 32'h00000004;
        B = 32'h00000002;
        Opcode = 4'b1001;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: 1

        // Test case 11: Pass A
        A = 32'h0000000F;
        B = 32'h000000F0;
        Opcode = 4'b1010;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: 15

        // Test case 12: Pass B
        A = 32'h0000000F;
        B = 32'h000000F0;
        Opcode = 4'b1011;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: -16

        // Test case 13: B + 8
        A = 32'h0000000F;
        B = 32'h000000F0;
        Opcode = 4'b1100;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: -248

        // Test case 14: Undefined operation
        A = 32'h0000000F;
        B = 32'h000000F0;
        Opcode = 4'b1101;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: 0

        // Test case 15: Undefined operation
        A = 32'h0000000F;
        B = 32'h000000F0;
        Opcode = 4'b1110;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: 0

        // Test case 16: Undefined operation
        A = 32'h0000000F;
        B = 32'h000000F0;
        Opcode = 4'b1111;
        #10;
        $display("Op: %b, A: %d (%b), B: %d (%b), Out: %d (%b), Z: %b, N: %b",
            Opcode, A, A, B, B, Out, Out, Z, N);
        // Expected output: 0

        $finish;
    end

endmodule