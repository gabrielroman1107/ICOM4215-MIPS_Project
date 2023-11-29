module MEM_Stage (
    input clk,
    input reset,
    input [17:0] control_signals,
    output reg [17:0] control_signals_out

);
    // input [5:0] opcode,
    // input [4:0] rs,
    // input [4:0] rt,
    // input [4:0] rd,
    // input [15:0] immediate,
    // input [5:0] funct,
    // input [2:0] alu_op_reg,
    // input [31:0] result_reg,
    // output reg [31:0] mem_result

    reg mem_size_reg;
    reg mem_se_reg;
    reg mem_rw_reg;
    reg mem_enable_reg;
    reg load_instr_reg;
    reg rf_enable_reg;
    reg ta_instr_reg;


    // Memory stage logic
    always @(posedge clk) begin
     if (reset) begin
            // Inicializar registros en caso de reset
            mem_size_reg = 2'b00;
            mem_se_reg = 1'b0;
            mem_rw_reg = 1'b0; 
            mem_enable_reg = 1'b0;
            load_instr_reg = 1'b0;
            rf_enable_reg = 1'b0;
            ta_instr_reg = 1'b0;

        end else begin
            // Lógica de la etapa MEM, como acceso a memoria (load o store)
            mem_size_reg = control_signals[6:5];
            mem_se_reg = control_signals[3];
            mem_rw_reg = control_signals[4];
            mem_enable_reg = control_signals[2];
            load_instr_reg = control_signals[9];
            rf_enable_reg = control_signals[8];
            ta_instr_reg = control_signals[7];

            control_signals_out[9] <= load_instr_reg;
            control_signals_out[8] <= rf_enable_reg;
            control_signals_out[7] <= ta_instr_reg;
            control_signals_out[6:5] <= mem_size_reg;
            control_signals_out[4] <= mem_rw_reg;
            control_signals_out[3] <= mem_se_reg;
            control_signals_out[2] <= mem_enable_reg;
            //17-8
           
            control_signals_out = {load_instr_reg, rf_enable_reg, ta_instr_reg, mem_size_reg, mem_rw_reg, mem_se_reg, mem_enable_reg};
    


        end
    end

endmodule