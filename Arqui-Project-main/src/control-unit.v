
module PPU_Control_Unit (
    input wire [31:0] instruction,
    output reg [23:0] control_signals

);

    reg [2:0] ID_SourceOperand_3bits;
    reg [3:0] ID_ALU_OP;
    reg ID_Load_Instr;
    reg ID_RF_Enable;
    reg ID_B_Instr;
    reg ID_TA_Instr;
    reg [1:0] ID_MEM_Size;
    reg ID_MEM_RW;
    reg ID_MEM_SE;
    reg ID_Enable_HI;
    reg ID_Enable_LO;
    reg ID_MEM_Enable;
    reg Unconditional_Jump;
    reg [1:0] Destination_Register; // 00 do nothing, 01 writes in rd, 10 writes in rt, 11 writes in r31
    reg Conditional_Unconditional_Jump;        
    reg Mux_Rs_Addr; 
    reg Addr_MUX; 

// Opcode values
    parameter R_TYPE1 = 6'b000000;
    parameter R_TYPE2 = 6'b011100;
    parameter R_TYPE3 = 6'b010000;
    parameter I_TYPE = 6'b000001;

// Function values for R type1
    parameter ADD_FUNCT = 6'b100000, 
     ADDU_FUNCT = 6'b100001,
     SUB_FUNCT = 6'b100010,
     SUBU_FUNCT = 6'b100011,
     SLT_FUNCT = 6'b101010,
     SLTU_FUNCT = 6'b101011,
     AND_FUNCT = 6'b100100,
     OR_FUNCT = 6'b100101,
     XOR_FUNCT = 6'b100110,
     NOR_FUNCT = 6'b100111,
     SLL_FUNCT = 6'b000000,
     SLLV_FUNCT = 6'b000100,
     SRA_FUNCT = 6'b000011,
     SRAV_FUNCT = 6'b000111,
     SRL_FUNCT = 6'b000010,
     SRLV_FUNCT = 6'b000110,
     MFHI_FUNCT = 6'b010000,
     MFLO_FUNCT = 6'b010010,
     MOVN_FUNCT = 6'b001011,
     MOVZ_FUNCT = 6'b001010,
     MTHI_FUNCT = 6'b010001,
     MTLO_FUNCT = 6'b010011,
     JALR_FUNCT = 6'b001001,
     JR_FUNCT = 6'b001000,
     TEQ_FUNCT = 6'b110100,
     TGE_FUNCT = 6'b110000,
     TGEU_FUNCT = 6'b110001,
     TLT_FUNCT = 6'b110010,
     TLTU_FUNCT = 6'b110011,
     TNE_FUNCT = 6'b110110;
    

// Function values for R type2
    parameter CLO_FUNCT = 6'b100001;
    parameter CLZ_FUNCT = 6'b100000;

// Function values for R type3
    parameter MFC0_FUNCT = 5'b00000;
    parameter MTC0_FUNCT = 5'b00100;

// B Case
    parameter B_Case = 6'b000100;



//rt values I types
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

// 6-bit opcodes I types
parameter ADDI_OP    = 6'b001000,
          ADDIU_OP   = 6'b001001,
          ANDI_OP    = 6'b001100,
          BGTZ_OP    = 6'b000111,
          BLEZ_OP    = 6'b000110,
          BNE_OP     = 6'b000101,
          LB_OP      = 6'b100000,
          LBU_OP     = 6'b100100,
          LH_OP      = 6'b100001,
          LHU_OP     = 6'b100101,
          SLTI_OP    = 6'b001010,
          SLTIU_OP   = 6'b001011,
          ORI_OP     = 6'b001101,
          XORI_OP    = 6'b001110,
          LW_OP      = 6'b100011,
          SD_OP      = 6'b101011,
          SB_OP      = 6'b101000,
          SH_OP      = 6'b101001,
          SW_OP      = 6'b101011,
          LUI_OP     = 6'b001111;
    
// J types
parameter J_OP       = 6'b000010,
          JAL_OP     = 6'b000011;
    
always @ (instruction) begin

    if (instruction == 32'b0 | instruction == 32'bx) begin

        control_signals <= 22'b0;

        ID_SourceOperand_3bits = 3'b0;
        ID_ALU_OP = 4'b0;
        ID_Load_Instr = 1'b0;
        ID_RF_Enable = 1'b0;
        ID_B_Instr = 1'b0;
        ID_TA_Instr = 1'b0;
        ID_MEM_Size = 2'b0;
        ID_MEM_RW = 1'b0;
        ID_MEM_SE = 1'b0;
        ID_Enable_HI = 1'b0;
        ID_Enable_LO = 1'b0;
        ID_MEM_Enable = 1'b0;
        Conditional_Unconditional_Jump = 1'b0; //bit 21
        Unconditional_Jump = 1'b0; //bit 20
        Destination_Register = 2'b00; //bit 18-19
        Mux_Rs_Addr = 1'b0; //bit 22
        Addr_MUX = 1'b0; //bit 23

        $display("Keyword: NOP");

    end else if (instruction[31:26] == ADDIU_OP) begin

        ID_SourceOperand_3bits = 3'b100;
        ID_ALU_OP = 4'b0000;
        ID_Load_Instr = 1'b0;
        ID_RF_Enable = 1'b1;
        ID_B_Instr = 1'b0;
        ID_TA_Instr = 1'b0;
        ID_MEM_Size = 2'b00;
        ID_MEM_RW = 1'b0;
        ID_MEM_SE = 1'b0;
        ID_Enable_HI = 1'b0;
        ID_Enable_LO = 1'b0;
        ID_MEM_Enable = 1'b0;
        Conditional_Unconditional_Jump = 1'b0; //bit 21
        Unconditional_Jump = 1'b0; //bit 20
        Destination_Register = 2'b10; //bit 18-19
        Mux_Rs_Addr = 1'b0; //bit 22
        Addr_MUX = 1'b0; //bit 23

    end else if ((instruction[31:26] == R_TYPE1) && (instruction[5:0] == SUBU_FUNCT)) begin
        ID_SourceOperand_3bits = 3'b000;
        ID_ALU_OP = 4'b0001;
        ID_Load_Instr = 1'b0;
        ID_RF_Enable = 1'b1;
        ID_B_Instr = 1'b0;
        ID_TA_Instr = 1'b0;
        ID_MEM_Size = 2'b00;
        ID_MEM_RW = 1'b0; 
        ID_MEM_SE = 1'b0;
        ID_Enable_HI = 1'b0;
        ID_Enable_LO = 1'b0;
        ID_MEM_Enable = 1'b0;
        Conditional_Unconditional_Jump = 1'b0; //bit 21
        Unconditional_Jump = 1'b0; //bit 20
        Destination_Register = 2'b01; //bit 18-19
        Mux_Rs_Addr = 1'b0; //bit 22
        Addr_MUX = 1'b0; //bit 23

    end else if ((instruction[31:26] == R_TYPE1) && (instruction[5:0] == ADDU_FUNCT)) begin
        ID_SourceOperand_3bits = 3'b000;
        ID_ALU_OP = 4'b0000;
        ID_Load_Instr = 1'b0;
        ID_RF_Enable = 1'b1;
        ID_B_Instr = 1'b0;
        ID_TA_Instr = 1'b0;
        ID_MEM_Size = 2'b00;
        ID_MEM_RW = 1'b0; 
        ID_MEM_SE = 1'b0;
        ID_Enable_HI = 1'b0;
        ID_Enable_LO = 1'b0;
        ID_MEM_Enable = 1'b0;
        Conditional_Unconditional_Jump = 1'b0; //bit 21
        Unconditional_Jump = 1'b0; //bit 20
        Destination_Register = 2'b01; //bit 18-19
        Mux_Rs_Addr = 1'b0; //bit 22
        Addr_MUX = 1'b0; //bit 23

    end else if (instruction[31:26] == LBU_OP) begin 
        // Handle LBU_OP case
        ID_SourceOperand_3bits = 3'b100;
        ID_ALU_OP = 4'b0000; 
        ID_Load_Instr = 1'b1;
        ID_RF_Enable = 1'b1;
        ID_B_Instr = 1'b0;
        ID_TA_Instr = 1'b0;
        ID_MEM_Size = 2'b00;
        ID_MEM_RW = 1'b0;
        ID_MEM_SE = 1'b0;  //0 
        ID_Enable_HI = 1'b0;
        ID_Enable_LO = 1'b0;
        ID_MEM_Enable = 1'b1;
        Conditional_Unconditional_Jump = 1'b0; //bit 21
        Unconditional_Jump = 1'b0; //bit 20
        Destination_Register = 2'b10; //bit 18-19
        Mux_Rs_Addr = 1'b0; //bit 22
        Addr_MUX = 1'b0; //bit 23


    end else if (instruction[31:26] == BGTZ_OP) begin // Handle BGTZ_OP case
        
        ID_SourceOperand_3bits = 3'b000;
        ID_ALU_OP = 4'b1010;
        ID_Load_Instr = 1'b0;
        ID_RF_Enable = 1'b0;
        ID_B_Instr = 1'b1;
        ID_TA_Instr = 1'b1;
        ID_MEM_Size = 2'b00;
        ID_MEM_RW = 1'b0;
        ID_MEM_SE = 1'b0;
        ID_Enable_HI = 1'b0;
        ID_Enable_LO = 1'b0;
        ID_MEM_Enable = 1'b0;
        Conditional_Unconditional_Jump = 1'b0; //bit 21
        Unconditional_Jump = 1'b0; //bit 19
        Destination_Register = 2'b0; //bit 18-19
        Mux_Rs_Addr = 1'b1; //bit 22
        Addr_MUX = 1'b0; //bit 23


    end else if (instruction[31:26] == JAL_OP) begin// Handle JAL_OP case
        
        ID_SourceOperand_3bits = 3'b011;
        ID_ALU_OP = 4'b1100;
        ID_Load_Instr = 1'b0;
        ID_RF_Enable = 1'b1;
        ID_B_Instr = 1'b0;
        ID_TA_Instr = 1'b1; //1
        ID_MEM_Size = 2'b00;
        ID_MEM_RW = 1'b0;
        ID_MEM_SE = 1'b0;
        ID_Enable_HI = 1'b0;
        ID_Enable_LO = 1'b0;
        ID_MEM_Enable = 1'b0;
        Conditional_Unconditional_Jump = 1'b1; //bit 21
        Unconditional_Jump = 1'b1; //bit 20
        Destination_Register = 2'b11; //bit 18-19
        Mux_Rs_Addr = 1'b0; //bit 22
        Addr_MUX = 1'b0; //bit 23


    end else if (instruction[31:26] == LUI_OP) begin// Handle LUI_OP case
        
        ID_SourceOperand_3bits = 3'b101;
        ID_ALU_OP = 4'b1011;
        ID_Load_Instr = 1'b0;
        ID_RF_Enable = 1'b1;
        ID_B_Instr = 1'b0;
        ID_TA_Instr = 1'b0;
        ID_MEM_Size = 2'b00;
        ID_MEM_RW = 1'b0;
        ID_MEM_SE = 1'b0;
        ID_Enable_HI = 1'b0;
        ID_Enable_LO = 1'b0;
        ID_MEM_Enable = 1'b0;
        Conditional_Unconditional_Jump = 1'b0; //bit 21
        Unconditional_Jump = 1'b0; //bit 20
        Destination_Register = 2'b10; //bit 18-19
        Mux_Rs_Addr = 1'b0; //bit 22
        Addr_MUX = 1'b0; //bit 23

    end else if ((instruction[31:26] == R_TYPE1) && (instruction[5:0] == JR_FUNCT)) begin
         ID_SourceOperand_3bits = 3'b000;
        ID_ALU_OP = 4'b0000;
        ID_Load_Instr = 1'b0;
        ID_RF_Enable = 1'b0;
        ID_B_Instr = 1'b0;
        ID_TA_Instr = 1'b0;
        ID_MEM_Size = 2'b00;
        ID_MEM_RW = 1'b0;
        ID_MEM_SE = 1'b0;
        ID_Enable_HI = 1'b0;
        ID_Enable_LO = 1'b0;
        ID_MEM_Enable = 1'b0;
        Conditional_Unconditional_Jump = 1'b1; //bit 21
        Unconditional_Jump = 1'b1; //bit 20
        Destination_Register = 2'b0; //bit 18-19
        Mux_Rs_Addr = 1'b1; //bit 22
        Addr_MUX = 1'b0; //bit 23
// 00 do nothing, 01 writes in rd, 10 writes in rt, 11 writes in r31
        
    end else if (instruction[31:26] == SB_OP) begin
        // Handle SB_OP case
        ID_SourceOperand_3bits = 3'b000;
        ID_ALU_OP = 4'b0000;
        ID_Load_Instr = 1'b0;
        ID_RF_Enable = 1'b0;
        ID_B_Instr = 1'b0;
        ID_TA_Instr = 1'b0;
        ID_MEM_Size = 2'b00;
        ID_MEM_RW = 1'b1;
        ID_MEM_SE = 1'b0;
        ID_Enable_HI = 1'b0;
        ID_Enable_LO = 1'b0;
        ID_MEM_Enable = 1'b1;
        Conditional_Unconditional_Jump = 1'b0; //bit 21
        Unconditional_Jump = 1'b0; //bit 20
        Destination_Register = 2'b00; //bit 18-19
        Mux_Rs_Addr = 1'b0; //bit 22
        Addr_MUX = 1'b0; //bit 23
// 00 do nothing, 01 writes in rd, 10 writes in rt, 11 writes in r31


    end else if(instruction[31:26] == BGEZ_RT)begin
        ID_SourceOperand_3bits = 3'b000;
        ID_ALU_OP = 4'b1010; 
        ID_Load_Instr = 1'b0;
        ID_RF_Enable = 1'b0;
        ID_B_Instr = 1'b1;
        ID_TA_Instr = 1'b1;
        ID_MEM_Size = 2'b00;
        ID_MEM_RW = 1'b0;
        ID_MEM_SE = 1'b0;
        ID_Enable_HI = 1'b0;
        ID_Enable_LO = 1'b0;
        ID_MEM_Enable = 1'b0;
        Conditional_Unconditional_Jump = 1'b1; 
        Unconditional_Jump = 1'b0; // bit 20
        Destination_Register = 2'b0; //bit 18-19
        Mux_Rs_Addr = 1'b0; //bit 22
        Addr_MUX = 1'b0; //bit 23


    end else if(instruction[31:26] == B_Case)begin
        ID_SourceOperand_3bits = 3'b000;
        ID_ALU_OP = 4'b0000;
        ID_Load_Instr = 1'b0;
        ID_RF_Enable = 1'b0;
        ID_B_Instr = 1'b1;
        ID_TA_Instr = 1'b1;
        ID_MEM_Size = 2'b00;
        ID_MEM_RW = 1'b0;
        ID_MEM_SE = 1'b0;
        ID_Enable_HI = 1'b0;
        ID_Enable_LO = 1'b0;
        ID_MEM_Enable = 1'b0;
        Conditional_Unconditional_Jump = 1'b1; 
        Unconditional_Jump = 1'b0; // bit 20
        Destination_Register = 2'b0; //bit 18-19
        Mux_Rs_Addr = 1'b0; //bit 22
        Addr_MUX = 1'b0; //bit 23

    end 

    
                // control_signals is a signal that represents the control signals for the MIPS processor.
        // The control signals are packed into a single vector, where each bit represents a specific control signal.
        // The order of the control signals is as follows:
        // Bit 0: ID_Enable_LO
        // Bit 1: ID_Enable_HI
        // Bit 2: ID_MEM_Enable
        // Bit 3: ID_MEM_SE
        // Bit 4: ID_MEM_RW
        // Bit 5-6: ID_MEM_Size
        // Bit 7: ID_TA_Instr
        // Bit 8: ID_RF_Enable
        // Bit 9: ID_Load_Instr
        // Bit 10: ID_B_Instr
        // Bit 11-14: ID_ALU_OP
        // Bit 15-17: ID_SourceOperand_3bits
        // Bit 18-19: Destination_Register
        // Bit 20: Unconditional_Jump actually just for jump or not
        // Bit 21: Conditional_Unconditional_Jump actually just for conditional jump or branch
        // Bit 22: Mux_Rs_Addr
        // Bit 23: Addr_MUX

    control_signals = {Addr_MUX, Mux_Rs_Addr, Conditional_Unconditional_Jump, Unconditional_Jump, Destination_Register, ID_SourceOperand_3bits, ID_ALU_OP, ID_B_Instr, ID_Load_Instr, ID_RF_Enable, ID_TA_Instr, ID_MEM_Size, ID_MEM_RW, ID_MEM_SE, ID_MEM_Enable, ID_Enable_HI, ID_Enable_LO};
    
    
    case (instruction[31:26])
     R_TYPE1: case(instruction[5:0])
        ADDU_FUNCT: $display("Keyword: ADDU");
        SUB_FUNCT: $display("Keyword: SUB"); 
        SUBU_FUNCT: $display("Keyword: SUBU"); 
        SLT_FUNCT: $display("Keyword: SLT");
        SLTU_FUNCT: $display("Keyword: SLTU");
        AND_FUNCT: $display("Keyword: AND");
        OR_FUNCT: $display("Keyword: OR");
        XOR_FUNCT: $display("Keyword: XOR");
        NOR_FUNCT: $display("Keyword: NOR");
        //SLL_FUNCT: $display("Keyword: SLL");
        SLLV_FUNCT: $display("Keyword: SLLV");
        SRA_FUNCT: $display("Keyword: SRA");
        SRAV_FUNCT: $display("Keyword: SRAV");
        SRL_FUNCT: $display("Keyword: SRL");
        SRLV_FUNCT: $display("Keyword: SRLV");
        MFHI_FUNCT: $display("Keyword: MFHI");
        MFLO_FUNCT: $display("Keyword: MFLO");
        MOVN_FUNCT: $display("Keyword: MOVN");
        MOVZ_FUNCT: $display("Keyword: MOVZ");
        MTHI_FUNCT: $display("Keyword: MTHI");
        MTLO_FUNCT: $display("Keyword: MTLO");
        JALR_FUNCT: $display("Keyword: JALR");
        JR_FUNCT: $display("Keyword: JR");
        TEQ_FUNCT: $display("Keyword: TEQ");
        TGE_FUNCT: $display("Keyword: TGE");
        TGEU_FUNCT: $display("Keyword: TGEU");
        TLT_FUNCT: $display("Keyword: TLT");
        TLTU_FUNCT: $display("Keyword: TLTU");
        TNE_FUNCT: $display("Keyword: TNE");
     endcase

    R_TYPE2: case(instruction[5:0])
            CLO_FUNCT: $display("Keyword: CLO");
            CLZ_FUNCT: $display("Keyword: CLZ");
        endcase

    R_TYPE3: case(instruction[5:0])
            MFC0_FUNCT: $display("Keyword: MFC0");
            MTC0_FUNCT: $display("Keyword: MTC0");
        endcase
    
    I_TYPE: case(instruction[20:16])
            BGEZ_RT: $display("Keyword: BGEZ");
            BGEZAL_RT: $display("Keyword: BGEZAL");
            BLTZ_RT: $display("Keyword: BLTZ");
            BLTZAL_RT: $display("Keyword: BLTZAL");
            BAL_RT: $display("Keyword: BAL");
            TEQI_RT: $display("Keyword: TEQI");
            TGEI_RT: $display("Keyword: TGEI");
            TGEIU_RT: $display("Keyword: TGEIU");
            TLTI_RT: $display("Keyword: TLTI");
            TLTIU_RT: $display("Keyword: TLTIU");
            TNEI_RT: $display("Keyword: TNEI");
        endcase

        ADDI_OP: $display("Keyword: ADDI");   
        ADDIU_OP: $display("Keyword: ADDIU");   
        ANDI_OP: $display("Keyword: ANDI"); 
        // BEQ_OP: $display("Keyword: BEQ");    
        BGTZ_OP: $display("Keyword: BGTZ");    
        BLEZ_OP: $display("Keyword: BLEZ");   
        BNE_OP: $display("Keyword: BNE");     
        LB_OP: $display("Keyword: LB");    
        LBU_OP: $display("Keyword: LBU");    
        LH_OP: $display("Keyword: LH");      
        LHU_OP: $display("Keyword: LHU");     
        SLTI_OP: $display("Keyword: SLTI");    
        SLTIU_OP: $display("Keyword: SLTIU");
        ORI_OP: $display("Keyword: ORI");  
        XORI_OP: $display("Keyword: XORI");  
        LW_OP: $display("Keyword: LW");    
        SD_OP: $display("Keyword: SD");
        SB_OP: $display("Keyword: SB");
        SH_OP: $display("Keyword: SH");
        SW_OP: $display("Keyword: SW");
        // B_OP: $display("Keyword: B");
        J_OP: $display("Keyword: J");
        JAL_OP: $display("Keyword: JAL");
        LUI_OP: $display("Keyword: LUI");

    B_Case: begin
        if(instruction[25:21] == instruction[20:16] && instruction[25:21] != 5'b0 && instruction[25:21] != 5'b0) begin
            $display("Keyword: BEQ");
        end else begin
            $display("Keyword: B");
        end
    end

    default: $display("Keyword: Unknown");
endcase

    end


endmodule

