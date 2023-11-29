module WB_Stage (
    input clk,
    input reset,
    input [17:0] control_signals,
    output reg [17:0] control_signals_out

);
//   input [5:0] opcode,
//     input [4:0] rs,
//     input [4:0] rt,
//     input [4:0] rd,
//     input [15:0] immediate,
//     input [5:0] funct,
//     input [2:0] alu_op_reg,
//     input [31:0] mem_result,
//     output reg [31:0] result_out

  reg rf_enable_reg;
  reg hi_enable_reg;
  reg lo_enable_reg;
  reg ta_instr_reg;

    always @(posedge clk) begin
        if (reset) begin
            // Inicializar registros en caso de reset
            rf_enable_reg = 1'b0;
            hi_enable_reg = 1'b0;
            lo_enable_reg = 1'b0;
            ta_instr_reg = 1'b0;
        end else begin
            // result_reg <= mem_result; 
            rf_enable_reg = control_signals[8];
            ta_instr_reg = control_signals[7];
            hi_enable_reg = control_signals[1];
            lo_enable_reg = control_signals[0];

            control_signals_out[8] <= rf_enable_reg;
            control_signals_out[7] <= ta_instr_reg;
            control_signals_out[1] <= hi_enable_reg;
            control_signals_out[0] <= lo_enable_reg;

            control_signals_out = {rf_enable_reg, ta_instr_reg, hi_enable_reg, lo_enable_reg};
        end
    end

endmodule