module SE_4addr26( 
    output reg [31:0] extended,
    input [25:0] extend
    );

    always @* begin
        extended = ({{26{extend[25]}}, extend} <<2);
    end
endmodule

module SE_4imm16( 
    output reg [31:0] extended,
    input [15:0] extend
    );

    always @* begin
        extended = ({{16{extend[15]}}, extend} <<2);
    end
endmodule
