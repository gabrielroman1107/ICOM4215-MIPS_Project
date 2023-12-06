
// module condition_handler (
// input wire Z,
// input wire N,
// input wire [5:0] opcode,
// input wire [4:0] rs,
// input wire [4:0] rt,
// input ID_branch_instr,
// output reg branch_out
// );

//         always @(*) begin
//                 // Condition handling logic
//                 case (opcode)
//                 6'b000000: // BEQ
//                     branch_out <= (Z == 1);
//                 6'b000001: // BNE
//                     branch_out <= (Z == 0);
//                 6'b000010: // BLT
//                     branch_out <= (N == 1);
//                 6'b000011: // BGE
//                     branch_out <= (N == 0);
//                 default:
//                     branch_out <= 0; // Default case
//             endcase
//         end

// endmodule


module MIPS_Condition_Handler (
    input wire [5:0] opcode,
    input wire [4:0] rs_value,
    input wire [4:0] rt_value,
    input wire [15:0] imm16,
    input wire [31:0] pc,
    input wire control_unit_B_Instr,
    output wire branch_target_pc,
    output wire branch_taken
  );

 

  parameter I_TYPE = 6'b000001,
            B_BEQ_OP = 6'b000100,
            BAL_BGEZALOP = 6'b000001,
            BGTZ_OP    = 6'b000111,
            BLEZ_OP    = 6'b000110,
            BNE_OP     = 6'b000101,
            J        = 6'b000010,
            JAL      = 6'b000011,
            JALR_FUNCT = 6'b001001,
            JR_FUNCT = 6'b001000;

  parameter BGEZ_RT    = 5'b00001,
          BGEZAL_RT  = 5'b10001,
          BLTZ_RT    = 5'b00000,
          BLTZAL_RT  = 5'b10000,
          BAL_RT     = 5'b10001,
          TEQI_RT    = 5'b01100,
          TGEI_RT    = 5'b01000,
          TGEIU_RT   = 5'b01001,
          TLTI_RT    = 5'b01010,
          TLTIU_RT   = 5'b01011,
          TNEI_RT    = 5'b01110;
  
    // Default values
    wire branch_target_pc_wire = 0; // Default value for the branch target PC
    wire branch_taken_wire = 0; // Default value for branch taken signal

    //J JAL JALR JR
  
    // Check conditions for each branch instruction
    always @* begin
    if(control_unit_B_Instr == 1) begin
      case ({opcode})
        // B or BEQ
        B_BEQ_OP: begin
          if (rs_value == rt_value) begin
            branch_target_pc = pc + 4 + (imm16 << 2);
            branch_taken = 1;
          end else begin
            branch_target_pc = pc + 4; // ASK
            branch_taken = 0;
          end
        end

        // BAL or BGEZAL
        BAL_BGEZALOP: begin
            if (rs_value >= 0) begin //BGEZAL
                //register 31 = pc + 8
                branch_target_pc = pc + 4 + (imm16 << 2);
                branch_taken = 1;
            end else begin //BAL
                //register 31 = pc + 8
                branch_target_pc = pc + 4 + (imm16 << 2);
                branch_taken = 1;
            end
        end
        // BNE
        BNE_OP: begin
          if (rs_value != rt_value) begin
            branch_target_pc = pc + 4 + (imm16 << 2);
            branch_taken = 1;
          end
        end
        // BGTZ
        BGTZ_OP: begin
          if (rs_value > 0) begin
            branch_target_pc = pc + 4 + (imm16 << 2);
            branch_taken = 1;
          end
        end
        // BLEZ
        BLEZ_OP: begin
          if (rs_value <= 0) begin
            branch_target_pc = pc + 4 + (imm16 << 2);
            branch_taken = 1;
          end
        end
        // BLTZ is with RT SO GOTTA CHECK OPCODE AND RT
        BLTZ_RT: begin
          if (rs_value < 0) begin
            branch_target_pc = pc + 4 + (imm16 << 2);
            branch_taken = 1;
          end
        end
        // BGEZ is with RT SO GOTTA CHECK OPCODE AND RT
        BGEZ_RT: begin
          if (rs_value >= 0) begin
            branch_target_pc = pc + 4 + (imm16 << 2);
            branch_taken = 1;
          end
        end
        
        // BLTZAL is with RT SO GOTTA CHECK OPCODE AND RT
        BLTZAL_RT: begin
          if (rs_value < 0) begin
            branch_target_pc = pc + 4 + (imm16 << 2);
            branch_taken = 1;
          end
        end



        // default: begin
        //   // Default case, no branch taken
        //   branch_target_pc = pc + 4;
        //   branch_taken = 0;
        // end
      endcase
      if (opcode == J) begin

        end 
    end else begin
            branch_target_pc = pc + 4;
            branch_taken = 0;
        end
end
  
  endmodule