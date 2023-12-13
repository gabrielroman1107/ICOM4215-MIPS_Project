module MEM_WB_Stage (
    input clk,
    input reset,
    input [21:0] control_signals,
    output reg [21:0] control_signals_out
);


    always @(posedge clk) begin
        if (reset) begin
            // Reset all registers
            control_signals_out <= 22'b0;
        end else begin
            // Update all registers
            control_signals_out = control_signals;
        end
    end

endmodule