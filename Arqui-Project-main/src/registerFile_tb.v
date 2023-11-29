// Register File testing bench
`include "registerFile.v"
`timescale 1ns / 1ns

module RegisterFile_tb;

    //Inputs
    reg [31:0] PW;
    reg [4:0] RA, RB, RW;
    reg LE, Clk;

    //Outputs
    wire [31:0] PA, PB;

    // Register File instance
    RegisterFile regFile (PA, PB, PW, RA, RB, RW, LE, Clk);

    // Clock
    initial begin
        // Initial values
        RA = 5'b00000; // 0
        RB = 5'b11111; // 31
        PW = 32'b00000000000000000000000000010100; //20
        RW = 5'b00000; // 0

        #3 // Delay

        // Cycle that increments the values of RW, RA, RB and PW by 1 every 4 units of time.
        repeat (32) begin 
        //#4 Clk = ~Clk; 
        RW = RW + 1;
        RA = RA + 1;
        RB = RB + 1;
        PW = PW + 1;
        #4; // Delay
        end

    end

    // Initial conditions
    initial begin   
        Clk = 0; // Clock set to 0
        LE = 1; // Load/Enable set to 1
        forever #2 Clk = ~Clk; // Forever clock

    end

    // Prints
    initial begin
        $display ("        PW          PA          PB   |  RW RA RB  | LE Clk");
        $display ("       __________________");
        $monitor("%d  %d  %d   |  %d %d %d  |  %d\t%d", PW, PA, PB, RW, RA, RB, LE, Clk);

    end


    // End condition to avoid looping infinitely
    initial 
        #130 $finish;

endmodule