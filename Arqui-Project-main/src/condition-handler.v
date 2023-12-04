
module condition_handler (
input wire Z,
input wire N,
input wire [5:0] opcode,
input wire [4:0] rs,
input wire [4:0] rt,
input ID_branch_instr,
output reg branch_out
);

        always @(*) begin
                // Condition handling logic
                case (opcode)
                6'b000000: // BEQ
                    branch_out <= (Z == 1);
                6'b000001: // BNE
                    branch_out <= (Z == 0);
                6'b000010: // BLT
                    branch_out <= (N == 1);
                6'b000011: // BGE
                    branch_out <= (N == 0);
                // Add more conditions as needed
                default:
                    branch_out <= 0; // Default case
            endcase
        end

endmodule
