module Adder_4 (
    input [31:0] adder_in,
    output reg [31:0] adder_out
);

    always @* begin
       adder_out = adder_in + 4;
    end

endmodule