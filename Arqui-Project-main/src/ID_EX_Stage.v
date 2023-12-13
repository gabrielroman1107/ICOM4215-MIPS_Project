
module ID_EX_Stage ( //ID/EX
    input clk,
    input reset,
    input wire [21:0] control_signals,
    output reg [21:0] control_signals_out,
    output reg [3:0] alu_op_reg,
    output reg branch_reg,
    output reg load_instr_reg,
    output reg rf_enable_reg,
    output reg SourceOperand_3bits,
    output reg ta_instr_reg
);
    // Execute stage logic
    always @(posedge clk ) begin
        if (reset) begin
        // Inicializar registros en caso de reset'
        control_signals_out <= 22'b0;
        end else begin
        control_signals_out = control_signals;
        end
    end

endmodule