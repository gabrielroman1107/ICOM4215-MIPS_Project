// `include "control-unit.v"
`include "instructionMemory.v"
`include "Adder+4.v"
`include "PC-Register.v"
`include "NPC-Register.v"
`include "IF_Stage.v"

module SimplePipeline(
    input wire clk,
    input wire reset,
    input wire [31:0] instruction_in,
    output wire [31:0] result_out, 
    output reg [31:0] pc_reg,
    output reg [31:0] npc_reg,
    output wire [14:0] control_output
);
    reg [31:0] instruction_reg;
    reg [2:0] alu_op_reg;
    reg [31:0] mem_result;
    reg [31:0] result_reg;
	reg load_instr_reg;
	reg shift_imm_reg;
	reg rf_enable_reg;
	reg branch_reg;
	reg ta_instr_reg;
	reg mem_enable_reg;
	reg mem_se_reg;
	reg mem_rw_reg;
	reg [1:0] mem_size_reg;
	reg hi_enable_reg;
	reg lo_enable_reg;
	reg [8:0] A; 
	wire [31:0] I;
    reg [7:0] data;
	
	// Instantiate Control Unit
// 	PPU_Control_Unit control_unit(
//     .instruction(instruction_reg),
//     .control_output(control_output)
//   );
 

    // Load enable for PC and nPC
	reg le_alu, le_mem, le_wb;
	reg le_pc = 1'b1;
	reg le_npc = 1'b1;

    // Outputs from ID stage
    wire [5:0] opcode;
    wire [4:0] rs;
    wire [4:0] rt;
    wire [4:0] rd;
    wire [15:0] immediate;
    wire [5:0] funct;
  
  // Instantiate Instruction Memory
  InstructionMemory imem (
            .I(I),
            .A(A)
        );

    initial begin
        $readmemb("instructions.txt", imem.mem);
    end

      // Etapa IF (Fetch)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_reg <= 32'b0;
            npc_reg <= 32'b0 + 4;
            instruction_reg <= 15'b0;
        end else begin
            if (le_pc && le_npc) pc_reg <= npc_reg;
			#1;
            npc_reg <= npc_reg + 4;
            A <= pc_reg;
            instruction_reg <= I;
            #1;
			$display("----------------------------------------------------------------------------------------",
            "\nInstruction = %b, PC = %d, nPC = %d, Control Unit = %b", instruction_reg, pc.pc_out , npc.npc_out, control_output);
        end
    end




    // Etapa ID (Decodificación de Instrucciones)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Initialize control signals
            // Add your initialization logic here if needed
			ta_instr_reg = 1'b0;
        end else begin
			ta_instr_reg = control_unit.control_output[7];
			
			if (ta_instr_reg) $display("Etapa ID: \nTA Reg = %b", ta_instr_reg);
        end
    end

    // Etapa EX (Ejecución)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Inicializar registros en caso de reset'
			alu_op_reg = 3'b000;
			branch_reg = 1'b0;
			load_instr_reg = 1'b0;
			rf_enable_reg = 1'b0;
			shift_imm_reg = 1'b0;
        end else begin
            // Lógica de la etapa EX, como operaciones aritméticas y lógicas
			alu_op_reg = control_unit.control_output[13:11];
			branch_reg = control_unit.control_output[8];
			load_instr_reg = control_unit.control_output[10];
			rf_enable_reg = control_unit.control_output[9];
			shift_imm_reg = control_unit.control_output[14];
			
			if (shift_imm_reg || alu_op_reg || branch_reg || load_instr_reg || rf_enable_reg) $display("Etapa EX: \nShift Imm = %b, ALU Reg = %b, Load Instruction = %b, RF Enable = %b, Branch Reg = %b,", shift_imm_reg, alu_op_reg, load_instr_reg, rf_enable_reg, branch_reg);
			
            if (le_alu) begin
                // Perform ALU operation
            end
        end
    end

    // Etapa MEM (Acceso a memoria)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Inicializar registros en caso de reset
			mem_size_reg = 2'b00;
			mem_se_reg = 1'b0;
			mem_rw_reg = 1'b0; 
			mem_enable_reg = 1'b0;
			load_instr_reg = 1'b0;		
			rf_enable_reg = 1'b0;			
        end else begin
            // Lógica de la etapa MEM, como acceso a memoria (load o store)
			mem_size_reg = control_unit.control_output[6:5];
			mem_se_reg = control_unit.control_output[3];
			mem_rw_reg = control_unit.control_output[4];
			mem_enable_reg = control_unit.control_output[0];
			load_instr_reg = control_unit.control_output[10];
			rf_enable_reg = control_unit.control_output[9];
			
			if (mem_size_reg || mem_se_reg || mem_rw_reg || mem_enable_reg || rf_enable_reg || load_instr_reg) $display("Etapa MEM: \nLoad Instruction = %b, RF Enable = %b, Mem Size = %b, Mem RW = %b, Mem SE = %b, Mem Enable = %b", load_instr_reg, rf_enable_reg, mem_size_reg, mem_rw_reg, mem_se_reg, mem_enable_reg);
			
            if (le_mem) begin
                // Perform memory operation
            end
        end
    end

    // Etapa WB (Write-Back)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Inicializar registros en caso de reset
			rf_enable_reg = 1'b0;
			hi_enable_reg = 1'b0;
			lo_enable_reg = 1'b0;
        end else begin
            result_reg <= mem_result; 
			rf_enable_reg = control_unit.control_output[9];
			hi_enable_reg = control_unit.control_output[2];
			lo_enable_reg = control_unit.control_output[1];
			// En una implementación real, puedes seleccionar entre alu_result y mem_result según la instrucción
			
			if (rf_enable_reg || hi_enable_reg || lo_enable_reg) $display("Etapa WB: \nRF Enable = %b, HI Enable = %b, LO Enable = %b", rf_enable_reg, hi_enable_reg, lo_enable_reg);
			
        end
    end

    assign result_out = result_reg;
    

endmodule