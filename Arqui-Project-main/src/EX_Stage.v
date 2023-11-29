module EX_Stage (
    input clk,
    input reset,
    input [17:0] control_signals,
    output reg [17:0] control_signals_out
);

     reg [2:0] alu_op_reg;
     reg branch_reg;
     reg load_instr_reg;
     reg rf_enable_reg;
     reg SourceOperand_3bits;
    reg ta_instr_reg;
   
    // Execute stage logic
    always @(posedge clk ) begin
        if (reset) begin
        // Inicializar registros en caso de reset'
            alu_op_reg = 4'b000;
            branch_reg = 1'b0;
            load_instr_reg = 1'b0;
            rf_enable_reg = 1'b0;
            SourceOperand_3bits = 3'b000;
            ta_instr_reg = 1'b0;
        end else begin
            // Lógica de la etapa EX, como operaciones aritméticas y lógicas
            SourceOperand_3bits = control_signals[17:15];
            alu_op_reg = control_signals[14:11];
            branch_reg = control_signals[10];
            load_instr_reg = control_signals[9];
            rf_enable_reg = control_signals[8];
            ta_instr_reg = control_signals[7];

            
        control_signals_out[17:15] <= SourceOperand_3bits;   
        control_signals_out[14:11] <= alu_op_reg;
        control_signals_out[10] <= branch_reg;
        control_signals_out[9] <= load_instr_reg;
        control_signals_out[8] <= rf_enable_reg;
        control_signals_out[7] <= ta_instr_reg;
      
        

        control_signals_out = {SourceOperand_3bits, alu_op_reg, branch_reg, load_instr_reg, rf_enable_reg, ta_instr_reg};
        end

    end

endmodule