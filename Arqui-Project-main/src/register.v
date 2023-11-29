module Register (
    output reg [31:0] Q, 
    input [31:0] D, 
    input clk, Ld
);

    always @ (posedge clk)
        if (Ld) Q <= D;

endmodule
