module ID_Mux(
    input wire [21:0] input_0,
    input S,
    output reg [21:0] mux_control_signals
    // output reg ID_branch_instr
);

always @ (*) begin
    if (S == 0) begin
        mux_control_signals = input_0; //check for later
    end else begin 
        mux_control_signals = 22'b0;
    end
end

endmodule