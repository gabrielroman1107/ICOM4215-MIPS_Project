
module ID_EX_Stage ( //ID/EX
    input clk,
    input reset,
    input wire [24:0] control_signals,
    input wire [15:0] id_ex_imm16,
    input wire [31:0] ex_instruction,
    input wire [31:0] PA,
    input wire [31:0] PB,
    input wire [31:0] PC,
    input wire [31:0] RS_Address,
    input wire [4:0] destination,
    output reg [24:0] control_signals_out,
    output reg [15:0] id_ex_imm16_out,
    output reg [31:0] PA_out,
    output reg [31:0] PB_out,
    output reg [31:0] PC_out,
    output reg [31:0] RS_Address_out,
    output reg [31:0] instruction_reg,
    output reg [4:0] destination_out
);
    // Execute stage logic
    always @(posedge clk ) begin
        if (reset) begin
        // Inicializar registros en caso de reset'
        control_signals_out <= 25'b0;
        id_ex_imm16_out <= 16'b0;
        PA_out <= 32'b0;
        PB_out <= 32'b0;
        PC_out <= 32'b0;
        destination_out <= 5'b0;
        RS_Address_out <= 32'b0;
        end else begin
        control_signals_out <= control_signals;
        id_ex_imm16_out <= id_ex_imm16;
        PA_out <= PA;
        PB_out <= PB;
        PC_out <= PC;
        destination_out <= destination;
        RS_Address_out <= RS_Address;
        instruction_reg <= ex_instruction;
        end
    end

endmodule