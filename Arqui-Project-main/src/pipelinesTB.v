// `include "pipelines.v"
// module SimplePipeline_TB;
//     // Define parameters
//     reg clk, reset, S;
//     reg [31:0] test_instruction,instruction;
//     wire [31:0] test_result;
//     reg [31:0] pc_reg, npc_reg;  // Declare pc_reg and npc_reg
//     wire [14:0] control_output;  // Declare control_output

//   // Instantiate Pipeline
//   SimplePipeline dut (
//     .clk(clk),
//     .reset(reset),
//     .instruction_in(test_instruction),
//     .result_out(test_result),
//     .pc_reg(),
//     .npc_reg(),
//     .control_output(control_output)
//   );

    

// // Clock generation
// 	always begin
// 		#2 clk = ~clk; // Invert the clock every 2 time unit
// 	end

//   initial fork
//     clk = 1'b0; // Initialize the clock
//     reset = 1'b1; // Reset the circuit
//     #3 reset = 1'b0; // Remove the reset
//     S = 1'b0; 
//     #40 S = 1'b1; // Set the S signal
//     #48 $finish;
// join

//   // Initial block for setup
//   initial begin
//     $readmemb("instructions.txt", dut.imem.mem);
//   end
 	

//   // Display information at each clock cycle
//   always @(posedge clk) begin

//     // Apply S signal
//     #40 S = 1'b1;

//     // Simulate until time 48
    
//   end

//   // Display information at each clock cycle
//   always @(posedge clk) begin

//     //testing if clock and reset are working
//     $display("\nClock=%0d, Reset=%0d", clk, reset);

  
//     // Print keyword, PC, nPC, and control signals
//     $display("\nPC=%0d nPC=%0d Control Signals=%b",  pc_reg, npc_reg, control_output);

//     // Print control signals of EX, MEM, and WB stages
//     $display("\nEX: %b MEM: %b WB: %b", dut.alu_op_reg, dut.mem_enable_reg, dut.rf_enable_reg);
  
//   // 4 displays total: keyword, Pc, Npc,  control signal(desglosado) from ID to EX, EX to MEM, MEM to WB

//   end

// endmodule

`include "system-control.v"

module Pipeline_TB;

  // Define parameters
  reg clk, reset, S;
  wire [31:0] test_instruction;
  wire [31:0] test_result;
  wire [31:0] pc_wire_out;
  wire [31:0] pc_wire_in;
  wire [31:0] npc_wire_in;
  wire [31:0] npc_wire_out;
  wire [16:0] control_wire;
  wire [31:0] instructionMem_wire_in;
  wire [31:0] instructionMem_wire_out;
  wire [16:0] ControlSignal_wire;
  wire [16:0] mux_wire_in;
  wire [16:0] mux_wire_out;




  // Instantiate Pipeline
  system_control dut (
    .clk(clk),
    .reset(reset)
  );

  NPC_Register npc_instance(
    .clk(clk),
    .reset(reset),
    .npc_in(npc_wire_in),
    .npc_out(npc_wire_out)
);

// Adder_4 adder_instance(
//     .adder_in(npc_wire_out),
//     .adder_out(adder_wire_out)
// );

PC_Register pc_instance(
    .clk(clk),
    .reset(reset),
    .pc_in(pc_wire_in),
    .pc_out(pc_wire_out)
);

// PPU_Control_Unit control_unit(
//     .instruction(instructionMem_wire_in),
//     .control_signals(ControlSignal_wire)
//   );    


ID_Mux mux_instance(
    .input_0(mux_wire_in),
    .S(S), // por ahora
    .mux_control_signals(mux_wire_out)
);


IF_Stage if_instance(
    .clk(clk),
    .reset(reset),
    .instruction_in(instructionMem_wire_in),
    .instruction_out(instructionMem_wire_out)
);

ID_Stage id_instance(
    .clk(clk),
    .reset(reset),
    .control_signals(control_wire),
    .control_signals_out(control_wire)
);

EX_Stage ex_instance(
    .clk(clk),
    .reset(reset),
    .control_signals(control_wire),
    .control_signals_out(control_wire)
);
MEM_Stage mem_instance(
    .clk(clk),
    .reset(reset),
    .control_signals(control_wire),
    .control_signals_out(control_wire)
);
WB_Stage wb_instance(
    .clk(clk),
    .reset(reset),
    .control_signals(control_wire),
    .control_signals_out(control_wire)
);



  
 always begin
		#1 clk = ~clk; // Invert the clock every 2 time unit
     //$display("Time %0t: Toggling clk to %b", $time, clk);
	end

  
  initial begin
   
    clk = 1'b0;
    reset = 1'b1;
    S = 1'b0;
   

    
    #2 reset = 1'b0;


   
    #38 S = 1'b1;

    
    #8 $finish;
  end



  
  always @(posedge clk) begin
    //  $display("=========================================================================",
    //         "\n  KeyWord: ADDIU  ,Instruction = %b, PC = %d, nPC = %d, Control Signals = %b, Clk = %b, Reset = %b, S = %b",  instructionMem_wire_out, pc_wire_out, npc_wire_out, mux_wire_out, clk, reset, S);


   
    if (instructionMem_wire_out[31:26] == 6'b001001) begin
        $display("=========================================================================",
            "\n  KeyWord: ADDIU  ,Instruction = %b, PC = %d, nPC = %d, Control Signals = %b, Clk = %b, Reset = %b, S = %b",  instructionMem_wire_out, pc_wire_out, npc_wire_out, mux_wire_out, clk, reset, S);
    
        //str = "ADDIU";
    end else if (instructionMem_wire_out[31:26] == 6'b100100) begin
        //str = "LBU";
        $display("=========================================================================",
            "\n  KeyWord: LBU  ,Instruction = %b, PC = %d, nPC = %d, Control Signals = %b, Clk = %b, Reset = %b, S = %b",  instructionMem_wire_out, pc_wire_out, npc_wire_out, mux_wire_out, clk, reset, S);
    

    end else if (instructionMem_wire_out[31:26] == 6'b000111) begin
       // str = "BGTZ";
        $display("=========================================================================",
            "\n  KeyWord: BGTZ  ,Instruction = %b, PC = %d, nPC = %d, Control Signals = %b, Clk = %b, Reset = %b, S = %b", instructionMem_wire_out, pc_wire_out, npc_wire_out, mux_wire_out, clk, reset, S);
    

    end else if (instructionMem_wire_out[31:26] == 6'b101000) begin
       // str = "SB";
        $display("=========================================================================",
            "\n  KeyWord: SB  ,Instruction = %b, PC = %d, nPC = %d, Control Signals = %b, Clk = %b, Reset = %b, S = %b", instructionMem_wire_out, pc_wire_out, npc_wire_out, mux_wire_out, clk, reset, S);
    

    end else if (instructionMem_wire_out[31:26] == 6'b000011) begin
        //str = "JAL";
        $display("=========================================================================",
            "\n  KeyWord: JAL  ,Instruction = %b, PC = %d, nPC = %d, Control Signals = %b, Clk = %b, Reset = %b, S = %b", instructionMem_wire_out, pc_wire_out, npc_wire_out, mux_wire_out, clk, reset, S);
    

    end else if (instructionMem_wire_out[31:26] == 6'b001111) begin
        //str = "LUI";
        $display("=========================================================================",
            "\n  KeyWord: LUI  ,Instruction = %b, PC = %d, nPC = %d, Control Signals = %b, Clk = %b, Reset = %b, S = %b", instructionMem_wire_out, pc_wire_out, npc_wire_out, mux_wire_out, clk, reset, S);
    

    end else if (instructionMem_wire_out[31:26] == 6'b000000) begin
        //str = "R";
        // $display("=========================================================================",
        //     "\n  KeyWord = %s  ,Instruction = %b, PC = %d, nPC = %d, Control Signals = %b, Clk = %b, Reset = %b, S = %b", test_instruction, pc_wire, npc_wire, control_wire, clk, reset, S);
    

        
        if (instructionMem_wire_out[5:0] == 6'b100011) begin
            //str = "SUBU";
            $display("=========================================================================",
            "\n  KeyWord: SUBU  ,Instruction = %b, PC = %d, nPC = %d, Control Signals = %b, Clk = %b, Reset = %b, S = %b",  instructionMem_wire_out, pc_wire_out, npc_wire_out, mux_wire_out, clk, reset, S);
    

        end
       
    end




    // Print keyword, PC, nPC, and control signals
    // $display("=========================================================================",
    //         "\n  KeyWord = %s  ,Instruction = %b, PC = %d, nPC = %d, Control Unit = %b, Clk = %b, Reset = %b, S = %b", str, test_instruction, pc_wire, npc_wire, control_wire, clk, reset, S);
    
    // Print control signals of EX, MEM, and WB stages
    // $display("EX: %b MEM: %b WB: %b", dut.alu_op_reg, dut.mem_enable_reg, dut.rf_enable_reg);
end

endmodule