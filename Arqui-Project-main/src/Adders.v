module Adder_4 (
    input [31:0] adder_in,
    output reg [31:0] adder_out
);

    always @* begin
       adder_out = adder_in + 4;
    end

endmodule

module AdderTA_signal(
    output reg [31:0] sum,
    input [31:0] operandBig,
    input [8:0] operandSmall
);

always @* begin
    sum = operandBig + operandSmall;
end

endmodule

module AdderPC_8(
    output reg [8:0] sum,
    input [8:0] PC
);

always @* begin
    sum = PC + 8'b00001000;
end

endmodule

module AdderPC_4signal( 
    output reg [8:0] result,
    input [8:0] input_value
);

always @* begin
    result = input_value + 8'b00000100;
end

endmodule