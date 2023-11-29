module NPC_Register (
    input clk,
    input reset,
    input [31:0] npc_in,
    output reg [31:0] npc_out
);
    reg le_pc = 1'b1;
	reg le_npc = 1'b1;

    always @(posedge clk) begin
        if (reset) begin
            npc_out <= 32'b00000000000000000000000000000100;
        end else if (le_npc) begin
            npc_out <= npc_in;
        end
    end

endmodule