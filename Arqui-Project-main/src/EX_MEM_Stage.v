module EX_MEM_Stage (
    input clk,
    input reset,
    input [21:0] control_signals,
    input [31:0] alu_result,
    output reg [21:0] control_signals_out,
    output reg [31:0] alu_result_out

);

    // Memory stage logic
    always @(posedge clk) begin
     if (reset) begin
            // Reset all registers
            control_signals_out <= 22'b0;
        end else begin
            // Update all registers
            control_signals_out = control_signals;
            alu_result_out = alu_result;
        end
    end

endmodule