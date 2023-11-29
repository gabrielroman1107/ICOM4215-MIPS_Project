`include "instructionMemory.v"

module InstructionMemory_tb;
    reg [8:0] A;
    wire [31:0] I;
    
    wire [31:0] instruction_output; // Wire to capture instruction memory output
    
    InstructionMemory instruction_memory (
        .A(A),
        .I(instruction_output)
    );
    
    initial begin
        // Load the test file into memory (example: instructions.txt)
        $readmemb("instructions.txt", instruction_memory.mem);
    
        for (A = 0; A <= 48; A = A + 4) begin
            #1
            $display("A=%d, I= %08b\n", A, instruction_output);
        end
        
        $finish;
    end
endmodule
