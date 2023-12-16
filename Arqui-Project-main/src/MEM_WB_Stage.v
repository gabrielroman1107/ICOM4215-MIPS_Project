module MEM_WB_Stage (
    input clk,
    input reset,
    input [23:0] control_signals,
    input [31:0] mem_mux_out,
    input wire [4:0] destination,
    output reg [31:0] mem_wb_out,
    output reg [23:0] control_signals_out,
    output reg [4:0] destination_out
);


    always @(posedge clk) begin
        if (reset) begin
            // Reset all registers
            control_signals_out <= 22'b0;
            mem_wb_out <= 32'b0;
            destination_out <= 5'b0;
        end else begin
            // Update all registers
            control_signals_out <= control_signals;
            mem_wb_out <= mem_mux_out;
            destination_out <= destination;
        end
    end

endmodule