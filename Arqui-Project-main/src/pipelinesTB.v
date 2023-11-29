`include "pipelines.v"
module SimplePipeline_TB;
    // Define parameters
    reg clk, reset, S;
    reg [31:0] test_instruction,instruction;
    wire [31:0] test_result;
    reg [31:0] pc_reg, npc_reg;  // Declare pc_reg and npc_reg
    wire [14:0] control_output;  // Declare control_output

  // Instantiate Pipeline
  SimplePipeline dut (
    .clk(clk),
    .reset(reset),
    .instruction_in(test_instruction),
    .result_out(test_result),
    .pc_reg(),
    .npc_reg(),
    .control_output(control_output)
  );

    

// Clock generation
	always begin
		#2 clk = ~clk; // Invert the clock every 2 time unit
	end

  initial fork
    clk = 1'b0; // Initialize the clock
    reset = 1'b1; // Reset the circuit
    #3 reset = 1'b0; // Remove the reset
    S = 1'b0; 
    #40 S = 1'b1; // Set the S signal
    #48 $finish;
join

  // Initial block for setup
  initial begin
    $readmemb("instructions.txt", dut.imem.mem);
  end
 	

  // Display information at each clock cycle
  always @(posedge clk) begin

    // Apply S signal
    #40 S = 1'b1;

    // Simulate until time 48
    
  end

  // Display information at each clock cycle
  always @(posedge clk) begin

    //testing if clock and reset are working
    $display("\nClock=%0d, Reset=%0d", clk, reset);

  
    // Print keyword, PC, nPC, and control signals
    $display("\nPC=%0d nPC=%0d Control Signals=%b",  pc_reg, npc_reg, control_output);

    // Print control signals of EX, MEM, and WB stages
    $display("\nEX: %b MEM: %b WB: %b", dut.alu_op_reg, dut.mem_enable_reg, dut.rf_enable_reg);
  
  // 4 displays total: keyword, Pc, Npc,  control signal(desglosado) from ID to EX, EX to MEM, MEM to WB

  end

endmodule