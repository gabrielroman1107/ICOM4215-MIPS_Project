module Reset_OR(
    input a, 
    input b,

    output reg Out

);
    always@(*) Out= a||b;
    

endmodule


