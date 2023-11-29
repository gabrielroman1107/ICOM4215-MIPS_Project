`include "ID_Mux.v"
`include "control-unit.v"

module ID_Stage (
    input wire clk,
    input wire reset,
    input wire [31:0] instruction_reg,
    // output wire [16:0] control_signals,

    // input [16:0] control_signals,
    output reg [17:0] control_signals_out

);

reg ta_instr_reg;

always @(posedge clk) begin
    if (reset) begin
      
        ta_instr_reg = 1'b0;
    end else begin
        ta_instr_reg = instruction_reg[7];
        control_signals_out <= instruction_reg;
        
    end

    // control_signals_out <= instruction_reg;
    //  control_signals_out[7] <= ta_instr_reg;

end
//    wire S;

    // // Instantiate Control Unit
    // PPU_Control_Unit control_unit(
    //     .instruction(instruction_reg),
    //     .control_signals()
    // );

    // // Instantiate Mux
    // ID_Mux mux(
    //     .input_0(control_unit.control_signals),
    //     .S(S),
    //     .mux_control_signals()
    // );

    // // Connect the output of the mux to the control_signals output of the ID_Stage module
    // // assign control_signals = mux_output;
    // assign control_signals = mux.mux_control_signals;

endmodule