module EX_MEM_Stage (
    input clk,
    input reset,
    input [23:0] control_signals,
    input [31:0] alu_result,
    input [4:0] destination,
    input wire [31:0] PB,
    output reg [23:0] control_signals_out,
    output reg [4:0] destination_out,
    output reg [31:0] PB_out,
    output reg [31:0] alu_result_out

);

    // Memory stage logic
    always @(posedge clk) begin
     if (reset) begin
            // Reset all registers
            control_signals_out <= 24'b0;
            PB_out <= 32'b0;
            alu_result_out <= 32'b0;
            destination_out <= 5'b0;

        end else begin
            // Update all registers
            control_signals_out <= control_signals;
            PB_out <= PB;
            alu_result_out <= alu_result;
            destination_out <= destination;

        end
    end

endmodule