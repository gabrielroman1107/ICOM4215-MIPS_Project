`include "PC-Register.v"
// `include "instructionMemory.v"
`include "Adder+4.v"
`include "NPC-Register.v"

module IF_Stage (
    input wire clk,
    input wire reset,
    input  [31:0] instruction_in,
    output reg [31:0] instruction_reg
);
    // // Instantiate NPC Register
    // NPC_Register npc (
    //     .clk(clk),
    //     .reset(reset),
    //     .npc_in(adder.adder_out),
    //     .npc_out()
    // );

    // // Instantiate PC
    // PC_Register pc (
    //     .clk(clk),
    //     .reset(reset),
    //     .pc_in(npc.npc_out),
    //     .pc_out()
    // );

    // // Instantiate Adder+4 
    // Adder_4 adder (
    //     .adder_in(npc.npc_out),
    //     .adder_out()
    // );

    // Instantiate Instruction Memory


    // always @(posedge clk) begin
    //     instruction_reg <= imem.I;
    // end

    // Fetch stage logic
always @(posedge clk) begin
    instruction_reg <= instruction_in;

 end

endmodule
