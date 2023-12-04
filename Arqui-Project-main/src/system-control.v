`include "IF_Stage.v"
`include "ID_Stage.v"
`include "EX_Stage.v"
`include "MEM_Stage.v"
`include "WB_Stage.v"
`include "instructionMemory.v"
`include "hazarding-unit.v"
`include "registerFile.v"
`include "dataMemory.v"
`include "ALU.v"
`include "Operand2Handler.v"
`include "condition-handler.v"


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
    IF_Stage if_stage(
        .clk(clk),
        .reset(reset),
        .instruction_in(instruction_wire_out),
        .instruction_reg()
    );

    //Instantiate ID_Stage
    ID_Stage id_stage(
        .clk(clk),
        .reset(reset),
        // .S(S),
        .instruction_reg(if_stage.instruction_reg),
        .control_signals_out()
    );

    //Instantiate EX_Stage
    EX_Stage ex_stage(
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
    MEM_Stage mem_stage(
        .clk(clk),
        .reset(reset),
        .control_signals(ex_stage.control_signals_out),
        .control_signals_out()
    );

    //Instantiate WB_Stage
    WB_Stage wb_stage(
        .clk(clk),
        .reset(reset),
        .control_signals(mem_stage.control_signals_out),
        .control_signals_out()
    );

    // Instantiate Control Unit
        PPU_Control_Unit control_unit(
            .instruction(id_stage.instruction_reg),
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


        // Instantiate Instruction Memory
    InstructionMemory imem(
        .A(pc.pc_out[8:0]),
        .I(instruction_wire_out)
    );

    // Instantiate Hazard Forwarding Unit /TODO: Check if this is correct
    hazard_forwarding_unit hazard_forwarding_unit(
        .forwardMX1(),
        .forwardMX2(),
        .forwardMX3(),
        .nPC_LE(),
        .PC_LE(),
        .IF_ID_LE(),
        .CU_S(),
        .EX_Register_File_Enable(),
        .MEM_Register_File_Enable(),
        .WB_Register_File_Enable(),
        .EX_RD(),
        .MEM_RD(),
        .WB_RD(),
        .ID_rs1(),
        .ID_rs2(),
        .ID_rd(),
        .EX_load_instr(),
        .ID_store_instr()
    );

    // Instantiate Register File /TODO: Check if this is correct
    RegisterFile register_file(
        .clk(clk),
        .LE(control_unit.ID_Load_Instr),
        .PW(datamem.DO),
        .RW(if_stage.instruction_reg[15:11]),
        .RA(if_stage.instruction_reg[25:21]),
        .RB(if_stage.instruction_reg[20:16]),
        .PA(),
        .PB()
    );

    // Instantiate Data Memory /TODO: Check if this is correct
    DataMemory datamem(
        .A(pc.pc_out[8:0]),
        .DI(register_file.PB),
        .Size(mem_stage.control_signals_out[6:5]), // Data size: 00 (byte), 01 (halfword), 10 (word)
        .R_W(mem_stage.control_signals_out[4]), // Read/Write signal: 0 (Read), 1 (Write)
        .E(mem_stage.control_signals_out[2]), // Enable signal
        .SE(mem_stage.control_signals_out[3]), // Sign extension signal for halfword and byte operations
        .DO() // Data output 
    );

    // Instantiate ALU
    ALU ex_alu(
        .A(muxA.Y),
        .B(register_file.PB),
        .Opcode(ex_stage.alu_op_reg),
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
        .S(mem_stage.control_signals_out[1]),
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
        .imm16(if_stage.instruction_reg[15:0]),
        .S(control_unit.ID_SourceOperand_3bits),
        .N()
    );

    // Instantiate Condition Handler
    condition_handler condition_handler(
        .Z(ex_alu.Z),
        .N(ex_alu.N),
        .ID_branch_instr(control_unit.ID_B_Instr),
        .opcode(if_stage.instruction_reg[31:26]),
        .rs(if_stage.instruction_reg[25:21]),
        .rt(if_stage.instruction_reg[20:16]),
        .branch_out()
    );


initial begin
    $readmemb("precargas/phase4.txt", imem.mem);
end

always begin
    #2 clk = ~clk;
end

  initial fork
    clk = 1'b0; // Initialize the clock
    reset = 1'b1; // Reset the circuit
    #3 reset = 1'b0; // Remove the reset
    S = 1'b0; 
    #40 S = 1'b1; // Set the S signal
    #56 $finish;
join

always @(posedge clk) begin

    // Apply S signal
    #40 S = 1'b1;

    // Simulate until time 48
    
  end

  // Display information at each clock cycle
  always @(posedge clk) begin

    // //testing if clock and reset are working
    // $display("\nClock=%0d, Reset=%0d", clk, reset);

  
    // Print keyword, PC, nPC, and control signals
    $display("\nInstruction=%b", instruction_wire_out);
    $display("\nIF:\nPC=%0d nPC=%0d Instruction Reg=%b",  pc.pc_out, npc.npc_out, if_stage.instruction_reg);
    $display("\nID:\nControl Signals=%b", id_stage.control_signals_out);
    $display("\nID_SourceOperand_3bits=%b, ID_ALU_OP=%b, ID_B_Instr=%b, ID_Load_Instr=%b, ID_RF_Enable=%b,  \nID_TA_Instr=%b, ID_MEM_Size=%b, ID_MEM_RW=%b, ID_MEM_SE=%b, ID_MEM_Enable=%b, ID_Enable_HI=%b, ID_Enable_LO=%b", id_stage.control_signals_out[17:15],id_stage.control_signals_out[14:11], id_stage.control_signals_out[10], id_stage.control_signals_out[9], id_stage.control_signals_out[8], id_stage.control_signals_out[7], id_stage.control_signals_out[6:5], id_stage.control_signals_out[4], id_stage.control_signals_out[3], id_stage.control_signals_out[2], id_stage.control_signals_out[1], id_stage.control_signals_out[0]);
    
    $display("\nEX:\nControl Signals=%b", ex_stage.control_signals_out[17:7]);
    $display("\nEX_SourceOperand_3bits=%b, EX_ALU_OP=%b, EX_B_Instr=%b,\nEX_Load_Instr=%b, EX_RF_Enable=%b, EX_TA_Instr=%b", ex_stage.control_signals_out[17:15],ex_stage.control_signals_out[14:11], ex_stage.control_signals_out[10], ex_stage.control_signals_out[9], ex_stage.control_signals_out[8], ex_stage.control_signals_out[7]);
    
    $display("\nMEM:\nControl Signals=%b", mem_stage.control_signals_out[9:2]);
    $display("\nMEM_Load_Instr=%b, MEM_RF_Enable=%b, MEM_TA_Instr=%b, MEM_MEM_Size=%b, MEM_MEM_RW=%b, MEM_MEM_SE=%b, MEM_MEM_Enable=%b", mem_stage.control_signals_out[9], mem_stage.control_signals_out[8], mem_stage.control_signals_out[7], mem_stage.control_signals_out[6:5], mem_stage.control_signals_out[4], mem_stage.control_signals_out[3], mem_stage.control_signals_out[2]);
    $display("\nWB:\nControl Signals=%b", wb_stage.control_signals_out[8:0]);
    $display("\nWB_RF_Enable=%b, WB_TA_Instr=%b, WB_HI=%b, WB_LO=%b", wb_stage.control_signals_out[8], wb_stage.control_signals_out[7], wb_stage.control_signals_out[1],wb_stage.control_signals_out[0]);
    $display("**************************************************************************");

    // // Print DataOut
    // $display("\nDataOut=%b", DataOut);

    // Print control signals of EX, MEM, and WB stages
    // $display("\nEX: %b MEM: %b WB: %b", dut.alu_op_reg, dut.mem_enable_reg, dut.rf_enable_reg);
  
  // 4 displays total: keyword, Pc, Npc,  control signal(desglosado) from ID to EX, EX to MEM, MEM to WB

  end


endmodule
