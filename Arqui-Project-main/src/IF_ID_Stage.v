module IF_ID_Stage ( //IF_ID
    input wire clk,
    input wire reset,
    input  [31:0] instruction_in,
    input wire load_enable,
    input [8:0] pc,
    //input logic_box,
    output reg [31:0] instruction_reg,
    output reg [25:0] address_26, // bit 25:0 de instruction 
    output reg [8:0] PC, //bit8:0  entrada desde PC
    output reg [25:21] rs, //bit 25:21
    output reg [20:16] rt, //bit 20:16
    output reg [15:0] imm16, //bit 15:0
    output reg [31:26] opcode, //bit 31:26
    output reg [15:11] rd //bit 15:11
);

    // Fetch stage logic
always @(posedge clk) begin
    if (reset) begin
        instruction_reg <= 32'b0;
        address_26 <= 26'b0;
        PC <= 9'b0;
        rs <= 6'b0;
        rt <= 6'b0;
        imm16 <= 16'b0;
        opcode <= 6'b0;
        rd <= 6'b0;

    end else if (load_enable) begin
    instruction_reg <= instruction_in;

    if(instruction_in[31:26] == 6'b0) begin //R-type instructions
        opcode <= instruction_in[31:26];
        rd <= instruction_in[15:11];
        rs <= instruction_in[25:21];
        rt <= instruction_in[20:16];

    end else if(instruction_in[31:26] == 6'b000011) begin //J-type instructions
        address_26 <= instruction_in[25:0];
        opcode <= instruction_in[31:26];

    end else begin //I-type instructions
        opcode <= instruction_in[31:26];
        rs <= instruction_in[25:21];
        rt <= instruction_in[20:16];
        imm16 <= instruction_in[15:0];
end
 end
end

endmodule