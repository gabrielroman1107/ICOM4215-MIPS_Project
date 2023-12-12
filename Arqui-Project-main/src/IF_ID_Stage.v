module IF_ID_Stage ( //IF_ID
    input wire clk,
    input wire reset,
    input  [31:0] instruction_in,
    output reg [31:0] instruction_reg
);

    // Fetch stage logic
always @(posedge clk) begin
    if (reset) begin
        // Reset: Set all bits to 0
        instruction_reg <= 32'b0;
    end else begin
        instruction_reg <= instruction_in;
    end
end

endmodule
