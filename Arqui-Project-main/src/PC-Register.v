module PC_Register (
    input clk,
    input reset,
    input [31:0] pc_in,
    input le,
    output reg [31:0] pc_out
);
    // PC register logic
    always @(posedge clk) begin
        if (reset) begin
            // Reset: Set all bits to 0
            pc_out <= 32'b0;
        end else if (le) begin
            pc_out <= pc_in;
        end
    end

endmodule

module NPC_PC_Handler(
    input branch_signal,
    input jump_signal,
    output reg [1:0] pc_source_select
);
    always @(*) begin
        // if (jump_signal == 1)                        pc_source_select = 2'b10; // Jump instruction
        if (branch_signal || jump_signal)                 pc_source_select = 2'b01; // Branch instruction
        else                             pc_source_select = 2'b00; // Normal execution
    end
endmodule