module pipeline_registers (
    input wire clk,
    input wire rst,
    input wire [31:0] in_data,
    input wire load_en,
    output reg [31:0] out_data
);
    reg [31:0] reg_data;

    always @(posedge clk or posedge rst)
        if (rst)
            reg_data <= 32'b0000_0000;
        else if (load_en)
            reg_data <= in_data;

    assign out_data = reg_data;
endmodule

module ID_EX (
    input wire clk,
    input wire rst,
    input wire [31:0] in_data,
    input wire load_en,
    output reg [31:0] out_data
);
    reg [31:0] reg_data;

    always @(posedge clk or posedge rst)
        if (rst)
            reg_data <= 32'b0000_0000;
        else if (load_en)
            reg_data <= in_data;

    assign out_data = reg_data;
endmodule

module EX_MEM (
    input wire clk,
    input wire rst,
    input wire [31:0] in_data,
    input wire load_en,
    output reg [31:0] out_data
);
    reg [31:0] reg_data;

    always @(posedge clk or posedge rst)
        if (rst)
            reg_data <= 32'b0000_0000;
        else if (load_en)
            reg_data <= in_data;

    assign out_data = reg_data;
endmodule

module MEM_WB (
    input wire clk,
    input wire rst,
    input wire [31:0] in_data,
    input wire load_en,
    output reg [31:0] out_data
);
    reg [31:0] reg_data;

    always @(posedge clk or posedge rst)
        if (rst)
            reg_data <= 32'b0000_0000;
        else if (load_en)
            reg_data <= in_data;

    assign out_data = reg_data;
endmodule
