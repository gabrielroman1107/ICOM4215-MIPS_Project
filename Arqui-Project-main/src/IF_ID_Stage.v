module IF_ID_Stage ( //IF_ID
    input wire clk,
    input wire reset,
    input  [31:0] instruction_in,
    // input wire load_enable,
    input [31:0] pc,
    // input logic_box,
    output reg [31:0] instruction_reg,
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
        PC <= pc;
    end
end

endmodule
