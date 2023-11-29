module InstructionMemory(
    input wire [8:0] A, // Address input
    output wire [31:0] I // Instruction output
);

reg [7:0] mem [0:511]; // ROM memory with 512 locations, each 8 bits wide

reg [31:0] instruction_output; // Internal variable to capture instruction output

always @(A) begin
    // Reading operation
    instruction_output = {mem[A], mem[A+1], mem[A+2], mem[A+3]};
end

assign I = instruction_output; // Assign the internal variable to the output
endmodule

