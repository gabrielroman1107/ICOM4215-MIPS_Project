`include "IF_ID_Stage.v"
`include "ID_EX_Stage.v"
`include "EX_MEM_Stage.v"
`include "MEM_WB_Stage.v"
`include "control-unit.v"
`include "ID_Mux.v"
`include "instructionMemory.v"
`include "hazarding-unit.v"
`include "registerFile.v"
`include "dataMemory.v"
`include "ALU.v"
`include "Operand2Handler.v"
// `include "condition-handler.v"
`include "PC-Register.v"
`include "NPC-Register.v"
`include "Adder+4.v"


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
    wire [31:0] DataOut;
    wire [16:0] control_signals_wire;
    wire [16:0] mux_out_wire;
    wire [31:0] instruction_wire_out;
    reg [7:0] data;

 // Instantiate NPC Register
    NPC_Register npc (
        .clk(clk),
        .reset(reset),
        .npc_in(adder.adder_out),
        .npc_out()
    );

    // Instantiate PC
    PC_Register pc (
        .clk(clk),
        .reset(reset),
        .pc_in(npc.npc_out),
        .pc_out()
    );

    // Instantiate Adder+4 
    Adder_4 adder (
        .adder_in(npc.npc_out),
        .adder_out()
    );


    //Instantiate IF_Stage
    IF_ID_Stage if_id_stage(
        //inputs
        .clk(clk),
        .reset(reset),
        .instruction_in(instruction_wire_out),
        .load_enable(1'b1),
        .pc(pc.pc_out[8:0]),
    
        //outputs
        .instruction_reg(),
        .address_26(),
        .PC(),
        .rs(),
        .rt(),
        .imm16(),
        .opcode(),
        .rd()
    );


    // Instantiate Control Unit
    PPU_Control_Unit control_unit(
        .instruction(if_id_stage.instruction_reg),
        .control_signals(),
        .ID_SourceOperand_3bits(),
        .ID_ALU_OP(),   
        .ID_Load_Instr(),
        .ID_RF_Enable(),
        .ID_B_Instr(),
        .ID_TA_Instr(),
        .ID_MEM_Size(),
        .ID_MEM_RW(),
        .ID_MEM_SE(),
        .ID_Enable_HI(),
        .ID_Enable_LO(),
        .ID_MEM_Enable()
    );
            // Instantiate Mux
    ID_Mux mux(
        .input_0(control_unit.control_signals),
        .S(S),
        .mux_control_signals(),
        .ID_branch_instr()
    );

    //Instantiate EX_Stage
    ID_EX_Stage id_ex_stage(
        .clk(clk),
        .reset(reset),
        .control_signals(mux.mux_control_signals),
        .control_signals_out(),
        .alu_op_reg(),
        .branch_reg(),
        .load_instr_reg(),
        .rf_enable_reg(),
        .SourceOperand_3bits(),
        .ta_instr_reg()
    );

    //Instantiate MEM_Stage
    EX_MEM_Stage ex_mem_stage(
        .clk(clk),
        .reset(reset),
        .control_signals(id_ex_stage.control_signals_out),
        .control_signals_out()
    );

    //Instantiate WB_Stage
    MEM_WB_Stage mem_wb_stage(
        .clk(clk),
        .reset(reset),
        .control_signals(ex_mem_stage.control_signals_out),
        .control_signals_out()
    );
        // Instantiate Instruction Memory
    InstructionMemory imem(
        .A(pc.pc_out[8:0]),
        .I(instruction_wire_out)
    );

    // // Instantiate Hazard Forwarding Unit /TODO: Check if this is correct
    // hazard_forwarding_unit hazard_forwarding_unit(
    //     .forwardMX1(),
    //     .forwardMX2(),
    //     .forwardMX3(),
    //     .nPC_LE(),
    //     .PC_LE(),
    //     .IF_ID_LE(),
    //     .CU_S(),
    //     .EX_Register_File_Enable(),
    //     .MEM_Register_File_Enable(),
    //     .WB_Register_File_Enable(),
    //     .EX_RD(),
    //     .MEM_RD(),
    //     .WB_RD(),
    //     .ID_rs1(),
    //     .ID_rs2(),
    //     .ID_rd(),
    //     .EX_load_instr(),
    //     .ID_store_instr()
    // );

    // Instantiate Register File /TODO: Check if this is correct
    RegisterFile register_file(
        .clk(clk),
        .LE(control_unit.ID_Load_Instr),
        .PW(datamem.DO),
        .RW(if_id_stage.instruction_reg[15:11]),
        .RA(if_id_stage.instruction_reg[25:21]),
        .RB(if_id_stage.instruction_reg[20:16]),
        .PA(),
        .PB()
    );

    // Register reg1 (.Q(register_file.I1),  .D(PW), .clk(clk), .Ld(E[1]));
    // Register reg3 (.Q(register_file.I3),  .D(PW), .clk(clk), .Ld(E[3]));
    // Register reg4 (.Q(register_file.I4),  .D(PW), .clk(clk), .Ld(E[4]));
    // Register reg5 (.Q(register_file.I5),  .D(PW), .clk(clk), .Ld(E[5]));
    // Register reg8 (.Q(register_file.I8),  .D(PW), .clk(clk), .Ld(E[8]));
    // Register reg10 (.Q(register_file.I10),  .D(PW), .clk(clk), .Ld(E[10]));
    // Register reg11 (.Q(register_file.I11),  .D(PW), .clk(clk), .Ld(E[11]));
    // Register reg12 (.Q(register_file.I12),  .D(PW), .clk(clk), .Ld(E[12]));


    // Instantiate Data Memory /TODO: Check if this is correct
    DataMemory datamem(
        .A(pc.pc_out[8:0]),
        .DI(register_file.PB),
        .Size(ex_mem_stage.control_signals_out[6:5]), // Data size: 00 (byte), 01 (halfword), 10 (word)
        .R_W(ex_mem_stage.control_signals_out[4]), // Read/Write signal: 0 (Read), 1 (Write)
        .E(ex_mem_stage.control_signals_out[2]), // Enable signal
        .SE(ex_mem_stage.control_signals_out[3]), // Sign extension signal for halfword and byte operations
        .DO() // Data output 
    );

    // Instantiate ALU
    ALU ex_alu(
        .A(muxA.Y),
        .B(register_file.PB),
        .Opcode(id_ex_stage.alu_op_reg),
        .Z(),   // Zero flag
        .N(),   // Negative flag
        .Out()
    );

    // Instantiate MUX
    mux_4x1 muxA(
        .I0(register_file.PA),
        .I1(datamem.DO), //CHANGE THIS
        .I2(ex_alu.Out),
        .I3(ex_alu.Out),
        .S(hazard_forwarding_unit.forwardMX1),
        .Y()
    );

    // Instantiate MUX
    mux_4x1 muxB(
        .I0(register_file.PA),
        .I1(datamem.DO), //CHANGE THIS
        .I2(ex_alu.Out),
        .I3(ex_alu.Out),
        .S(hazard_forwarding_unit.forwardMX2),
        .Y()
    );

    // Instantiate MUX 2x1 WB
    mux_2x1 muxWB(
        .I0(ex_alu.Out),
        .I1(datamem.DO), //CHANGE THIS
        .S(ex_mem_stage.control_signals_out[1]),
        .Y()
    );



// Instantiate PC+8 ALU
    ALU pc8_alu(
        .B(pc.pc_out),
        .Opcode(control_unit.ID_ALU_OP),
        .Out()
    );

    // Instantiate Source Operand Handler
    Operand2_Handler source_operand_handler(
        .PB(),
        .HI(),
        .LO(),
        .PC(pc.pc_out),
        .imm16(if_id_stage.instruction_reg[15:0]),
        .S(control_unit.ID_SourceOperand_3bits),
        .N()
    );

    // // Instantiate Condition Handler
    // condition_handler condition_handler(
    //     .Z(ex_alu.Z),
    //     .N(ex_alu.N),
    //     .ID_branch_instr(control_unit.ID_B_Instr),
    //     .opcode(if_id_stage.instruction_reg[31:26]),
    //     .rs(if_id_stage.instruction_reg[25:21]),
    //     .rt(if_id_stage.instruction_reg[20:16]),
    //     .branch_out()
    // );

initial forever 
    #2 clk = ~clk;

initial begin

    // $monitor("PC=%0d, Data Memory Address=%0d, r1=%0d, r3=%0d, r4=%0d, r5=%0d, r8=%0d, r10=%0d, r11=%0d, r12=%0d",
    // pc.pc_out, datamem.A, reg1, reg3, reg4, reg5, reg8, reg10, reg11, reg12);

    $readmemb("precargas/instructions.txt", imem.mem);
end

  initial fork
    clk = 1'b0; // Initialize the clock
    reset = 1'b1; // Reset the circuit
    #3 reset = 1'b0; // Remove the reset
    S = 1'b0; 
    #40 S = 1'b1; // Set the S signal
    #48 $finish;
join

// always @(posedge clk) begin

//     // Apply S signal
//     #40 S = 1'b1;

//     // Simulate until time 48
    
//   end

  // Display information at each clock cycle
  always @(posedge clk) begin

    // //testing if clock and reset are working
    // $display("\nClock=%0d, Reset=%0d", clk, reset);

  
    // Print keyword, PC, nPC, and control signals
    // $display("\nInstruction=%b", instruction_wire_out);
    // $display("\nIF:\nPC=%0d nPC=%0d Instruction Reg=%b",  pc.pc_out, npc.npc_out, if_id_stage.instruction_reg);
    $display("\nIF/ID:\nInstruction=%b\nPC=%0d, nPC=%0d", if_id_stage.instruction_reg, pc.pc_out, npc.npc_out);
    $display("\nControl Signal Output =%b", control_unit.control_signals);
    $display("\nID/EX:\nControl Signal=%b", mux.mux_control_signals);
    $display("\nID/EX_SourceOperand_3bits=%b, ID/EX_ALU_OP=%b, ID/EX_B_Instr=%b, ID/EX_Load_Instr=%b, ID/EX_RF_Enable=%b,  \nID/EX_TA_Instr=%b, ID/EX_MEM_Size=%b, ID/EX_MEM_RW=%b, ID/EX_MEM_SE=%b, ID/EX_MEM_Enable=%b, ID/EX_Enable_HI=%b, ID/EX_Enable_LO=%b", mux.mux_control_signals[17:15],mux.mux_control_signals[14:11], mux.mux_control_signals[10], mux.mux_control_signals[9], mux.mux_control_signals[8], mux.mux_control_signals[7], mux.mux_control_signals[6:5], mux.mux_control_signals[4], mux.mux_control_signals[3], mux.mux_control_signals[2], mux.mux_control_signals[1], mux.mux_control_signals[0]);
    
    $display("\nEX/MEM:\nControl Signal=%b", ex_mem_stage.control_signals_out);
    $display("\nEX/MEM_SourceOperand_3bits=%b, EX/MEM_ALU_OP=%b, EX/MEM_B_Instr=%b, EX/MEM_Load_Instr=%b, EX/MEM_RF_Enable=%b,  \nEX/MEM_TA_Instr=%b, EX/MEM_MEM_Size=%b, EX/MEM_MEM_RW=%b, EX/MEM_MEM_SE=%b, EX/MEM_MEM_Enable=%b, EX/MEM_Enable_HI=%b, EX/MEM_Enable_LO=%b", ex_mem_stage.control_signals_out[17:15],ex_mem_stage.control_signals_out[14:11], ex_mem_stage.control_signals_out[10], ex_mem_stage.control_signals_out[9], ex_mem_stage.control_signals_out[8], ex_mem_stage.control_signals_out[7], ex_mem_stage.control_signals_out[6:5], ex_mem_stage.control_signals_out[4], ex_mem_stage.control_signals_out[3], ex_mem_stage.control_signals_out[2], ex_mem_stage.control_signals_out[1], ex_mem_stage.control_signals_out[0]);
    
    $display("\nMEM/WB:\nControl Signal=%b", mem_wb_stage.control_signals_out);
    $display("\nMEM/WB_SourceOperand_3bits=%b, MEM/WB_ALU_OP=%b, MEM/WB_B_Instr=%b, MEM/WB_Load_Instr=%b, MEM/WB_RF_Enable=%b,  \nMEM/WB_TA_Instr=%b, MEM/WB_MEM_Size=%b, MEM/WB_MEM_RW=%b, MEM/WB_MEM_SE=%b, MEM/WB_MEM_Enable=%b, MEM/WB_Enable_HI=%b, MEM/WB_Enable_LO=%b", mem_wb_stage.control_signals_out[17:15],mem_wb_stage.control_signals_out[14:11], mem_wb_stage.control_signals_out[10], mem_wb_stage.control_signals_out[9], mem_wb_stage.control_signals_out[8], mem_wb_stage.control_signals_out[7], mem_wb_stage.control_signals_out[6:5], mem_wb_stage.control_signals_out[4], mem_wb_stage.control_signals_out[3], mem_wb_stage.control_signals_out[2], mem_wb_stage.control_signals_out[1], mem_wb_stage.control_signals_out[0]);
    $display("======================================================================================================================================\n");
    // // Print DataOut
    // $display("\nDataOut=%b", DataOut);

    // Print control signals of EX, MEM, and WB stages
    // $display("\nEX: %b MEM: %b WB: %b", dut.alu_op_reg, dut.mem_enable_reg, dut.rf_enable_reg);
  
  // 4 displays total: keyword, Pc, Npc,  control signal(desglosado) from ID to EX, EX to MEM, MEM to WB

  end


endmodule
