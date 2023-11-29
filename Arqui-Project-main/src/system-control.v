`include "IF_Stage.v"
`include "ID_Stage.v"
`include "EX_Stage.v"
`include "MEM_Stage.v"
`include "WB_Stage.v"
`include "instructionMemory.v"
`include "hazarding-unit.v"
`include "registerFile.v"
`include "dataMemory.v"


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
    integer fi, fo, code, i;

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
            .mux_control_signals()
        );


        // Instantiate Instruction Memory
    InstructionMemory imem(
        .A(pc.pc_out),
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
        .A(pc.pc_out),
        .DI(register_file.PB),
        .Size(mem_stage.control_signals_out[6:5]), // Data size: 00 (byte), 01 (halfword), 10 (word)
        .R_W(mem_stage.control_signals_out[4]), // Read/Write signal: 0 (Read), 1 (Write)
        .E(mem_stage.control_signals_out[2]), // Enable signal
        .SE(mem_stage.control_signals_out[3]), // Sign extension signal for halfword and byte operations
        .DO() // Data output 
    );

    // Instantiate ALU
    ALU ex_alu(
        .A(muxA.mux_out),
        .B(register_file.PB),
        .Opcode(ex_stage.alu_op_reg),
        .Out()
    );

    // Instantiate MUX
    Mux muxA(
        .input_0(register_file.PA),
        .input_1(datamem.DO),
        .input_2(ex_alu.ALU_OUT),
        .input_3(pc.pc_out),
        .input_4(npc.npc_out),
        .input_5(),
        .input_6(),
        .input_7(),
        .S(ex_stage.SourceOperand_3bits),
        .mux_out()
    );

    // Instantiate MUX
    Mux muxB(
        .input_0(register_file.PA),
        .input_1(datamem.DO),
        .input_2(ex_alu.ALU_OUT),
        .input_3(pc.pc_out),
        .input_4(npc.npc_out),
        .input_5(),
        .input_6(),
        .input_7(),
        .S(ex_stage.SourceOperand_3bits),
        .mux_out()
    );

    // Instantiate Source Operand Handler
    SourceOperandHandler source_operand_handler(
        .PB(),
        .HI(),
        .LO(),
        .PC(pc.pc_out),
        .imm16(if_stage.instruction_reg[15:0]),
        .S(control_unit.ID_SourceOperand_3bits),
        .N()
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
    $display("\nEX:\nControl Signals=%b", ex_stage.control_signals_out);
    $display("\nMEM:\nControl Signals=%b", mem_stage.control_signals_out);
    $display("\nWB:\nControl Signals=%b", wb_stage.control_signals_out);
    $display("**************************************************************************");

    // // Print DataOut
    // $display("\nDataOut=%b", DataOut);

    // Print control signals of EX, MEM, and WB stages
    // $display("\nEX: %b MEM: %b WB: %b", dut.alu_op_reg, dut.mem_enable_reg, dut.rf_enable_reg);
  
  // 4 displays total: keyword, Pc, Npc,  control signal(desglosado) from ID to EX, EX to MEM, MEM to WB

  end


endmodule
