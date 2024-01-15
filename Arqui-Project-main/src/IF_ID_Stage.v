module IF_ID_Stage ( //IF_ID
    input wire clk,
    input wire reset,
    input  [31:0] instruction_in,
    // input wire load_enable,
    input [31:0] pc,
    // input logic_box,
    output reg [31:0] instruction_reg,
    output reg [4:0] instruction_rs, // bit 25:21 de instruction
    output reg [4:0] instruction_rt, // bit 20:16 de instruction
    output reg [15:11] instruction_rd, // bit 15:11 de instruction
    output reg [15:0] instruction_imm16, // bit 15:0 de instruction
    output reg [31:26] instruction_opcode, // bit 31:26 de instruction
    output reg [5:0] instruction_shamt, // bit 10:6 de instruction
    output reg [5:0] instruction_funct, // bit 5:0 de instruction
    output reg [25:0] instruction_address_26, // bit 25:0 de instruction

    // output reg [25:0] address_26, // bit 25:0 de instruction 
    output reg [31:0] PC //entrada desde PC
    // output reg [25:21] rs, //bit 25:21
    // output reg [20:16] rt, //bit 20:16
    // output reg [15:0] imm16, //bit 15:0
    // output reg [31:26] opcode, //bit 31:26
    // output reg [15:11] rd //bit 15:11
);

    // Fetch stage logic
always @(posedge clk) begin
    if (reset) begin
        instruction_reg <= 32'b0;
        PC <= 32'b0;
end else begin
        instruction_reg <= instruction_in;
        instruction_rs <= instruction_in[25:21];
        instruction_rt <= instruction_in[20:16];
        instruction_rd <= instruction_in[15:11];
        instruction_imm16 <= instruction_in[15:0];
        instruction_opcode <= instruction_in[31:26];
        instruction_shamt <= instruction_in[10:6];
        instruction_funct <= instruction_in[5:0];
        instruction_address_26 <= instruction_in[25:0];
        PC <= pc;
    end
end

endmodule
