`include "IF_ID_Stage.v"
`include "ID_EX_Stage.v"
`include "EX_MEM_Stage.v"
`include "MEM_WB_Stage.v"
`include "control-unit.v"
`include "ID_Mux.v"
`include "instructionMemory.v"
`include "hazardforwarding.v"
`include "registerFile.v"
`include "dataMemory.v"
`include "ALU.v"
`include "Operand2Handler.v"
`include "condition-handler.v"
`include "PC-Register.v"
`include "NPC-Register.v"
`include "Adders.v"
`include "Sign_Extenders.v"


module system_control (

    
);
 // Declare wires and registers
reg clk;
reg reset;
reg S;

    wire [31:0] npc_wire_out;
    wire [31:0] pc_wire_in;
    wire [31:0] pc_wire_out;
    wire [31:0] adder_wire_out;
    

    reg [8:0] address;
    wire [31:0] DataMEMOut;
    wire [16:0] control_signals_wire;
    wire [16:0] mux_out_wire;
    wire [31:0] instruction_wire_out;
    reg [7:0] data;

    wire [31:0] I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10,
		I11, I12, I13, I14, I15, I16, I17, I18, I19, I20,
		I21, I22, I23, I24, I25, I26, I27, I28, I29, I30,
		I31;
    
    wire [31:0] Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, Q10, 
        Q11, Q12, Q13, Q14, Q15, Q16, Q17, Q18, Q19, Q20, 
        Q21, Q22, Q23, Q24, Q25, Q26, Q27, Q28, Q29, Q30, Q31;

    wire [31:0] E;

    integer i, finished, pc_count;
    

 // Instantiate NPC Register
    NPC_Register npc (      //load_enable set to on for simplicity                                     //DONE
        .clk(clk),
        .reset(reset),
        .npc_in(adder.adder_out),
        .npc_out(npc_wire_out)
    );

    // Instantiate PC
    PC_Register pc (       //load_enable set to on for simplicity                                       //DONE
        .clk(clk),
        .reset(reset),
        .pc_in(npc_wire_out),
        .pc_out(pc_wire_out)
    );

    // Instantiate Adder+4 
    Adder_4 adder (                                              //DONE
        .adder_in(npc_wire_out),
        .adder_out()
    );

    //Instantiate IF_Stage
    IF_ID_Stage if_id_stage(                                        //DONE
    .clk(clk),
    .reset(reset),
    .instruction_in(instruction_wire_out),
    .pc(pc.pc_out),
    .instruction_reg(),
    .PC()
    );


    // Instantiate Control Unit
    PPU_Control_Unit control_unit(                                   //DONE
        .instruction(if_id_stage.instruction_reg),
        .control_signals()
    );
            // Instantiate Mux
    ID_Mux mux(                                                      //DONE
        .input_0(control_unit.control_signals),
        .S(HazardForwardingUnit.nop_signal),
        .mux_control_signals()
    );

    //Instantiate EX_Stage
    ID_EX_Stage id_ex_stage(                                        //DONE
        .clk(clk),
        .reset(reset),
        .control_signals(mux.mux_control_signals),
        .destination(wb_destination_mux.destination),
        .PA(muxA.Y),
        .PB(muxB.Y),
        .PC(if_id_stage.PC),
        .RS_Address(RS_address_mux.Y),
        .destination_out(),
        .RS_Address_out(),
        .PA_out(),
        .PB_out(),
        .PC_out(),
        .control_signals_out()
    );

    //Instantiate MEM_Stage
    EX_MEM_Stage ex_mem_stage(                                        //DONE
        .clk(clk),
        .reset(reset),
        .control_signals(id_ex_stage.control_signals_out),
        .PB(id_ex_stage.PB_out),
        .alu_result(ex_alu.Out),
        .destination(id_ex_stage.destination_out),
        .destination_out(),
        .alu_result_out(),
        .PB_out(),
        .control_signals_out()
    );

    //Instantiate WB_Stage
    MEM_WB_Stage mem_wb_stage(                                          //DONE
        .clk(clk),
        .reset(reset),
        .control_signals(ex_mem_stage.control_signals_out),
        .destination(id_ex_stage.destination_out),
        .mem_mux_out(MemMux.Y),
        .destination_out(),
        .control_signals_out(),
        .mem_wb_out()
    );
        // Instantiate Instruction Memory
    InstructionMemory imem(                                                  //DONE
        .A(pc.pc_out[8:0]),
        .I(instruction_wire_out)
    );

    // Instantiate Hazard Forwarding Unit
    HAZARD_FORWARDING_UNIT HazardForwardingUnit(                              //DONE
        .id_rs(if_id_stage.instruction_reg[25:21]),
        .id_rt(if_id_stage.instruction_reg[20:16]),
        .ex_rf_enable(id_ex_stage.control_signals_out[8]), 
        .mem_rf_enable(ex_mem_stage.control_signals_out[8]), 
        .wb_rf_enable(mem_wb_stage.control_signals_out[8]), 
        .ex_load_instruction(id_ex_stage.control_signals_out[9]),
        .mem_load_instruction(ex_mem_stage.control_signals_out[9]),
        .ex_destination(id_ex_stage.destination_out), 
        .mem_destination(ex_mem_stage.destination_out),
        .wb_destination(mem_wb_stage.destination_out),
        .pa_selector(), 
        .pb_selector(),
        .hazard_type(),
        .load_enable(),
        .pc_enable(), 
        .nop_signal()
    );

    // Instantiate Register File /TODO: Check if this is correct                  //DONE
    RegisterFile register_file(
        .clk(clk),
        .LE(mem_wb_stage.control_signals_out[8]), //this come from write back
        .PW(mem_wb_stage.mem_wb_out),
        .RW(mem_wb_stage.destination_out),
        .RA(if_id_stage.instruction_reg[25:21]),
        .RB(if_id_stage.instruction_reg[20:16]),
        .PA(),
        .PB()
    );

    WB_Destination wb_destination_mux(                                                      //DONE
        .rt(if_id_stage.instruction_reg[20:16]),
        .rd(if_id_stage.instruction_reg[15:11]),
        .E(mux.mux_control_signals[19:18]),
        .destination()
    );

    // Instantiate Data Memory /TODO: Check if this is correct
    DataMemory datamem(                                                           //DONE
        .A(ex_mem_stage.alu_result_out[8:0]),
        .DI(ex_mem_stage.PB_out),
        .Size(ex_mem_stage.control_signals_out[6:5]), // Data size: 00 (byte), 01 (halfword), 10 (word)
        .R_W(ex_mem_stage.control_signals_out[4]), // Read/Write signal: 0 (Read), 1 (Write)
        .E(ex_mem_stage.control_signals_out[2]), // Enable signal
        .SE(ex_mem_stage.control_signals_out[3]), // Sign extension signal for halfword and byte operations
        .DO(DataMEMOut) // Data output 
    );

    // Instantiate ALU
    ALU ex_alu(                                      // DONE
        .A(id_ex_stage.PA_out),
        .B(source_operand_handler.N),
        .Opcode(id_ex_stage.control_signals_out[14:11]), // ALU operation code
        .Z(),   // Zero flag
        .N(),   // Negative flag
        .Out()
    );

    // Instantiate MUX
    mux_4x1 muxA(                                   //DONE
        .I0(register_file.PA),
        .I1(MemMux.Y),
        .I2(mem_wb_stage.mem_wb_out),
        .I3(ex_alu.Out),
        .S(HazardForwardingUnit.pa_selector),
        .Y()
    );

    // Instantiate MUX
    mux_4x1 muxB(                                   // DONE
        .I0(register_file.PB),
        .I1(MemMux.Y),
        .I2(mem_wb_stage.mem_wb_out),
        .I3(ex_alu.Out),
        .S(HazardForwardingUnit.pb_selector),
        .Y()
    );

    mux_2x1 MemMux(                                 // DONE
        .I0(ex_mem_stage.alu_result_out),
        .I1(DataMEMOut),
        .S(ex_mem_stage.control_signals_out[2]),
        .Y()
    );

    // Instantiate Source Operand Handler                    //DONE
    Operand2_Handler source_operand_handler(
        .PB(id_ex_stage.PB_out),
        .HI(hi_mux.Y), 
        .LO(lo_mux.Y), 
        .PC(id_ex_stage.PC_out),
        .imm16(if_id_stage.instruction_reg[15:0]),
        .S(id_ex_stage.control_signals_out[17:15]),
        .N()
    );

    //Instantiate HI Mux
    HI_MUX hi_mux(                                                 //DONE
        .HI_Enable(mem_wb_stage.control_signals_out[1]),
        .HI(if_id_stage.instruction_reg),
        .Y()
    );

    //Instantiate LO Mux
    LO_MUX lo_mux(                                                //DONE
        .LO_Enable(mem_wb_stage.control_signals_out[0]),
        .LO(if_id_stage.instruction_reg),
        .Y()
    );
    
    // Instantiate TA Mux                                                //DONE
    mux_2x1 TA_Mux(
        .I0(RS_address_mux.Y), //ID_TA_Instr
        .I1(id_ex_stage.RS_Address_out),
        .S(condition_handler.Condition_Handler_Out),           
        .Y()
    );


    // Instantiate RS Address Mux
    mux_2x1 RS_address_mux(
        .I0(register_file.PA), //FROM RS
        .I1(PCadder.Out), //FROM ADDER
        .S(mux.mux_control_signals[22]),                                         //DONE?
        .Y()
    );

    SE_4addr26 SE_4addr26(                             //DONE
        .extend(if_id_stage.instruction_reg[25:0]),
        .extended()
    );

    SE_4imm16 SE_4imm16(                                //DONE
        .extend(if_id_stage.instruction_reg[15:0]),
        .extended()
    );

    mux_2x1 Base_Addr_Mux(
        .I0(SE_4addr26.extended),
        .I1(SE_4imm16.extended),
        .S(mux.mux_control_signals[23]),                                       //DONE?
        .Y()
    );

    PC_Mux pc_mux(
        .nPC(npc.npc_out),
        .TA(TA_Mux.Y),
        .select(npc_pc_handler.pc_source_select),               //DONE
        .Out()
    );


    NPC_PC_Handler npc_pc_handler(                                 //DONE
        .branch_signal(condition_handler.Condition_Handler_Out),
        .jump_signal(control_unit.control_signals[20]),                         //DONE??
        .pc_source_select()
    );

    ALU PCadder(   //PC + Base Addr Mux                         //DONE
        .A(if_id_stage.PC),
        .B(Base_Addr_Mux.Y),
        .Opcode(4'b0000), // ALU Operation Code For Sum
        .Out()
    );

   // Instantiate Condition Handler
    Condition_Handler condition_handler(                                    //DONE
        .instruction(if_id_stage.instruction_reg),
        .branch_instruction(id_ex_stage.control_signals_out[10]),
        .Z(ex_alu.Z),
        .N(ex_alu.N),
        .Condition_Handler_Out()
    );

// Register R0 (.Q(Q0),  .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[0]));
// Register R1 (.Q(Q1),  .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[1]));
// Register R2 (.Q(Q2),  .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[2]));
// Register R3 (.Q(Q3),  .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[3]));
// Register R4 (.Q(Q4),  .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[4]));
// Register R5 (.Q(Q5),  .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[5]));
// Register R6 (.Q(Q6),  .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[6]));
// Register R7 (.Q(Q7),  .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[7]));
// Register R8 (.Q(Q8),  .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[8]));
// Register R9 (.Q(Q9),  .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[9]));
// Register R10 (.Q(Q10), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[10]));
// Register R11 (.Q(Q11), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[11]));
// Register R12 (.Q(Q12), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[12]));
// Register R13 (.Q(Q13), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[13]));
// Register R14 (.Q(Q14), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[14]));
// Register R15 (.Q(Q15), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[15]));
// Register R16 (.Q(Q16), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[16]));
// Register R17 (.Q(Q17), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[17]));
// Register R18 (.Q(Q18), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[18]));
// Register R19 (.Q(Q19), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[19]));
// Register R20 (.Q(Q20), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[20]));
// Register R21 (.Q(Q21), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[21]));
// Register R22 (.Q(Q22), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[22]));
// Register R23 (.Q(Q23), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[23]));
// Register R24 (.Q(Q24), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[24]));
// Register R25 (.Q(Q25), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[25]));
// Register R26 (.Q(Q26), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[26]));
// Register R27 (.Q(Q27), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[27]));
// Register R28 (.Q(Q28), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[28]));
// Register R29 (.Q(Q29), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[29]));
// Register R30 (.Q(Q30), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[30]));
// Register R31 (.Q(Q31), .D(mem_wb_stage.mem_wb_out), .clk(clk), .Ld(E[31]));

// BinaryDecoder decoder(
//     .E(E),
//     .C(wb_destination_mux.destination),
//     .RF(control_unit.control_signals[8])
// ); 


// mux_32_Monitor mux_32_1_Monitor(
//     //outputs
//    .PA(), .PB(), .Y0(), .Y1(), .Y2(), .Y3(), .Y4(), .Y5(), .Y6(), .Y7(),
//   .Y8(), .Y9(), .Y10(), .Y11(), .Y12(), .Y13(), .Y14(), .Y15(),
//   .Y16(), .Y17(), .Y18(), .Y19(), .Y20(), .Y21(), .Y22(), .Y23(),
//   .Y24(), .Y25(), .Y26(), .Y27(), .Y28(), .Y29(), .Y30(), .Y31(),

//   //inputs
//   .rs(if_id_stage.instruction_reg[25:21]), .rt(if_id_stage.instruction_reg[20:16]), .R0(Q0), .R1(Q1), .R2(Q2), .R3(Q3), .R4(Q4), 
//   .R5(Q5), .R6(Q6), .R7(Q7), .R8(Q8), .R9(Q9), .R10(Q10), .R11(Q11), .R12(Q12), .R13(Q13), .R14(Q14), .R15(Q15),
//   .R16(Q16), .R17(Q17), .R18(Q18), .R19(Q19), .R20(Q20), .R21(Q21), .R22(Q22), .R23(Q23),
//   .R24(Q24), .R25(Q25), .R26(Q26), .R27(Q27), .R28(Q28), .R29(Q29), .R30(Q30), .R31(Q31)
// );

initial begin
    clk = 1'b0; // Initialize the clock
    reset = 1'b1; // Reset the circuit
    finished = 1'b0;


    forever #2 clk = ~clk;

end

 initial begin
    $readmemb("precargas/phase4.txt", imem.mem);
    $readmemb("precargas/phase4.txt", datamem.mem);
    
    $monitor("\n\nPC: %0d, Data Mem Address: %0d, \n\nR5: %0d, R6: %0d, R16: %0d, R17: %0d, R18: %0d, \n\nWB Out: %0d,\n\nData Memory Out: %0d\n======================================================", 
    pc_wire_out, datamem.A, register_file.I5, register_file.I6, register_file.I16, register_file.I17, register_file.I18, mem_wb_stage.mem_wb_out, DataMEMOut);

    // $monitor("\n PC=%d, nPC=%d\n Input0 (PA Register File) PA Mux:%b,\n Input1 (Output DataMem after MUX) PA MUX:%b,\n Input2 (WB Output) PA MUX: %b,\n Input3 (EX_ALU Output)PA Mux: %b\n\n Output PA Mux:%b\n ============================================================ \
    // \n InputA (MUX PA OUT) EX_ALU: %b,\n InputB (S2H Out) EX_ALU: %b,\n Opcode (ID/EX Control signal[14:11])EX_ALU : %b,\n Output ALU: %b, \n Z:%b & N:%b, \n\n Source Operand Handler: \n PB: %b, HI: %b,\n LO: %b, imm16: %b,\n SOH Opcode (control_signal_out[17:15]): %b, Output: %b\n============================================================",
    // pc.pc_out, npc.npc_out, muxA.I0, muxA.I1, muxA.I2, muxA.I3, muxA.Y,id_ex_stage.PA_out, source_operand_handler.N, id_ex_stage.control_signals_out[14:11], ex_alu.Out, ex_alu.Z, ex_alu.N, source_operand_handler.PB, source_operand_handler.HI, source_operand_handler.LO, source_operand_handler.imm16, source_operand_handler.S, source_operand_handler.N );

    //  $monitor("\n InputA (MUX PA OUT) EX_ALU: %b,\n InputB (S2H Out) EX_ALU: %b,\n Opcode (ID/EX Control signal[14:11])EX_ALU : %b,\n Output ALU: %b, \n Z:%b & N:%b\n =======================",
    //  id_ex_stage.PA_out, source_operand_handler.N, id_ex_stage.control_signals_out[14:11], ex_alu.Out, ex_alu.Z, ex_alu.N);

    //$monitor("PC= %d \nControl Unit Signal Output= %b\n Mux Output= %b\n ID_EX Control Signal= %b \n EX_MEM Control Signal = %b\n MEM_WB Control Signal= %b\n\n", pc_wire_out, control_unit.control_signals, mux.mux_control_signals, id_ex_stage.control_signals_out, ex_mem_stage.control_signals_out, mem_wb_stage.control_signals_out);
 
end

always @(HazardForwardingUnit.hazard_type) begin
    case (HazardForwardingUnit.hazard_type)
    2'b01: $display("Load-Use Hazard");
    2'b10: $display("Read-After-Write Hazard (PA)");
    2'b11: $display("Read-After-Write Hazard (PB)");
    default: $display("No Hazard");
endcase
    
end
     

initial fork
    #3 reset = 1'b0; // Remove the reset
    S = 1'b0;    
    #100 $display("\nDataMemory contents at Address 52: %b\n\n", datamem.mem[52]);
   #100 $finish;
join






// always @(posedge clk) begin

//     // Apply S signal
//     #40 S = 1'b1;

//     // Simulate until time 48
    
//   end

  //Display information at each clock cycle
//   always @(posedge clk) begin

//     // //testing if clock and reset are working
//     // $display("\nClock=%0d, Reset=%0d", clk, reset);

  
//     // Print keyword, PC, nPC, and control signals
//     $display("\nInstruction MEM OUT=%b", instruction_wire_out);
//     // $display("\nIF:\nPC=%0d nPC=%0d Instruction Reg=%b",  pc.pc_out, npc.npc_out, if_id_stage.instruction_reg);
//     $display("\nIF/ID:\nInstruction= %b\nPC=%0d, nPC=%0d", if_id_stage.instruction_reg, pc.pc_out, npc.npc_out);

//     $display("\nControl Unit Signal Output= %b", control_unit.control_signals);

//     $display("ID_Enable_LO=%b", control_unit.control_signals[0]);
//     $display("ID_Enable_HI=%b", control_unit.control_signals[1]);
//     $display("ID_MEM_Enable=%b", control_unit.control_signals[2]);
//     $display("ID_MEM_SE=%b", control_unit.control_signals[3]);
//     $display("ID_MEM_RW=%b", control_unit.control_signals[4]);
//     $display("ID_MEM_Size=%b", control_unit.control_signals[6:5]);
//     $display("ID_TA_Instr=%b", control_unit.control_signals[7]);
//     $display("ID_RF_Enable=%b", control_unit.control_signals[8]);
//     $display("ID_Load_Instr=%b", control_unit.control_signals[9]);
//     $display("ID_B_Instr=%b", control_unit.control_signals[10]);
//     $display("ID_ALU_OP=%b", control_unit.control_signals[14:11]);
//     $display("ID_SourceOperand_3bits=%b", control_unit.control_signals[17:15]);
//     $display("Destination_Register=%b", control_unit.control_signals[19:18]);
//     $display("Unconditional_Jump=%b", control_unit.control_signals[20]);
//     $display("Conditional_Unconditional_Jump=%b", control_unit.control_signals[21]);
//     $display("Mux_Rs_Addr=%b", control_unit.control_signals[22]);
//     $display("Addr_MUX=%b", control_unit.control_signals[23]);
    
//     $display("\nID/EX:\nControl Signal= %b", id_ex_stage.control_signals_out);
//     $display("\nID/EX_Addr_MUX=%b, ID/EX_Mux_Rs_Addr=%b, ID/EX_Conditional_Unconditional_Jump=%b, ID/EX_Unconditional_Jump=%b, ID/EX_Destination_Register=%b,\nID/EX_SourceOperand_3bits=%b, ID/EX_ALU_OP=%b, ID/EX_B_Instr=%b, ID/EX_Load_Instr=%b, ID/EX_RF_Enable=%b,  \nID/EX_TA_Instr=%b, ID/EX_MEM_Size=%b, ID/EX_MEM_RW=%b, ID/EX_MEM_SE=%b, ID/EX_MEM_Enable=%b, ID/EX_Enable_HI=%b, ID/EX_Enable_LO=%b", 
//     id_ex_stage.control_signals_out[23], id_ex_stage.control_signals_out[22], id_ex_stage.control_signals_out[21], id_ex_stage.control_signals_out[20],id_ex_stage.control_signals_out[19:18], id_ex_stage.control_signals_out[17:15],id_ex_stage.control_signals_out[14:11], id_ex_stage.control_signals_out[10], id_ex_stage.control_signals_out[9], id_ex_stage.control_signals_out[8], 
//     id_ex_stage.control_signals_out[7], id_ex_stage.control_signals_out[6:5], id_ex_stage.control_signals_out[4], id_ex_stage.control_signals_out[3], id_ex_stage.control_signals_out[2], id_ex_stage.control_signals_out[1], id_ex_stage.control_signals_out[0]);

//     // $display("ID/EX_Enable_LO=%b, ID/EX_Enable_HI=%b, ID/EX_MEM_Enable=%b, ID/EX_MEM_SE=%b, ID/EX", id_ex_stage.control_signals_out[0]);
    
//     $display("\nEX/MEM:\nControl Signal=%b", ex_mem_stage.control_signals_out);
//     $display("\nEX/MEM_Addr_MUX=%b, EX/MEM_Mux_Rs_Addr=%b, EX/MEM_Conditional_Unconditional_Jump=%b, EX/MEM_Unconditional_Jump=%b, EX/MEM_Destination_Register=%b,\nEX/MEM_SourceOperand_3bits=%b, EX/MEM_ALU_OP=%b, EX/MEM_B_Instr=%b, EX/MEM_Load_Instr=%b, EX/MEM_RF_Enable=%b,  \nEX/MEM_TA_Instr=%b, EX/MEM_MEM_Size=%b, EX/MEM_MEM_RW=%b, EX/MEM_MEM_SE=%b, EX/MEM_MEM_Enable=%b, EX/MEM_Enable_HI=%b, EX/MEM_Enable_LO=%b", 
//     ex_mem_stage.control_signals_out[23], ex_mem_stage.control_signals_out[22], ex_mem_stage.control_signals_out[21], ex_mem_stage.control_signals_out[20],ex_mem_stage.control_signals_out[19:18], ex_mem_stage.control_signals_out[17:15],ex_mem_stage.control_signals_out[14:11], ex_mem_stage.control_signals_out[10], ex_mem_stage.control_signals_out[9], ex_mem_stage.control_signals_out[8], 
//     ex_mem_stage.control_signals_out[7], ex_mem_stage.control_signals_out[6:5], ex_mem_stage.control_signals_out[4], ex_mem_stage.control_signals_out[3], ex_mem_stage.control_signals_out[2], ex_mem_stage.control_signals_out[1], ex_mem_stage.control_signals_out[0]);
    
//     $display("\nMEM/WB:\nControl Signal=%b", mem_wb_stage.control_signals_out);
//     $display("\nMEM/WB_Addr_MUX=%b, MEM/WB_Mux_Rs_Addr=%b, MEM/WB_Conditional_Unconditional_Jump=%b, MEM/WB_Unconditional_Jump=%b, MEM/WB_Destination_Register=%b,\nMEM/WB_SourceOperand_3bits=%b, MEM/WB_ALU_OP=%b, MEM/WB_B_Instr=%b, MEM/WB_Load_Instr=%b, MEM/WB_RF_Enable=%b,  \nMEM/WB_TA_Instr=%b, MEM/WB_MEM_Size=%b, MEM/WB_MEM_RW=%b, MEM/WB_MEM_SE=%b, MEM/WB_MEM_Enable=%b, MEM/WB_Enable_HI=%b, MEM/WB_Enable_LO=%b", 
//     mem_wb_stage.control_signals_out[23], mem_wb_stage.control_signals_out[22], mem_wb_stage.control_signals_out[21], mem_wb_stage.control_signals_out[20],mem_wb_stage.control_signals_out[19:18], mem_wb_stage.control_signals_out[17:15],mem_wb_stage.control_signals_out[14:11], mem_wb_stage.control_signals_out[10], mem_wb_stage.control_signals_out[9], mem_wb_stage.control_signals_out[8], 
//     mem_wb_stage.control_signals_out[7], mem_wb_stage.control_signals_out[6:5], mem_wb_stage.control_signals_out[4], mem_wb_stage.control_signals_out[3], mem_wb_stage.control_signals_out[2], mem_wb_stage.control_signals_out[1], mem_wb_stage.control_signals_out[0]);    
//     $display("===================================================================================================================================\n");
//     // // Print DataOut
//     // $display("\nDataOut=%b", DataMEMOut);


//   end


endmodule
