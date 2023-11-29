module Operand2_Handler_tb;

    // Inputs
    reg [31:0] PB;
    reg [31:0] HI;
    reg [31:0] LO;
    reg [31:0] PC;
    reg [15:0] imm16;
    reg [2:0] S;

    // Outputs
    wire [31:0] N; // N is the output of the module

    // Instantiate the module you want to test
    Operand2_Handler uut ( // uut = Unit Under Test
        .PB(PB),
        .HI(HI),
        .LO(LO),
        .PC(PC),
        .imm16(imm16),
        .S(S),
        .N(N)
    );

    initial begin
        // Assign values to PB, HI, LO, and PC
        PB = 32'b00100010001101010100011001111000; 
        HI = 32'b10101011110011011110111100000001;
        LO = 32'b01111110110111001011101010001001; 
        PC = 32'b11111110110111001011101010011000; 

        // Initialize imm16 with the number 1110110001000100
        imm16 = 16'b1110110001000100;

        // Loop through all combinations of S (0 to 3)
        for (S = 0; S <= 6; S = S + 1) begin
            #10;  // Add a delay if needed
            
            // Display the binary values of S and N
            $display("S: %b, N: %b", S, N);
        end
        
        #10 
        S = 3'b111; // Set S to the 7th combination
        $display("S: %b, N: %b", S, N); // Display the binary values of S and N

        // Change the value of imm16
        imm16 = 16'b0110110001000100;  // Change imm16 by the number 0110110001000100

        // Display the value of N for the 100th combination of S (assuming S is 3 bits)
        S = 3'b100;  // Set S to the 100th combination
        #10;  // Add a delay if needed
        
        // Display the binary values of S and N
        $display("S: %b, N: %b", S, N);
        
        // Finish the simulation
        $finish;
    end

endmodule
