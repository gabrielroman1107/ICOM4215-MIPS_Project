`include "binary_decoder.v"

module BinaryDecoderTest;

  // Inputs
  reg [4:0] C;
  reg RF;

  // Outputs
  wire [31:0] E;

  // Instantiate the BinaryDecoder module
  BinaryDecoder decoder (
    .C(C),
    .RF(RF),
    .E(E)
  );

  // Initial block for testbench
  initial begin
    $display("4x16 BinaryDecoder Test");
    
    // Initialize inputs
    C = 5'b00000;
    RF = 1'b1;

    // Apply various test cases and display the process
    $display("Input: C = %b, RF = %b", C, RF);
    #10; // Delay for observation
    $display("Output E = %h", E);
    $display("");

    C = 5'b01010;
    $display("Input: C = %b, RF = %b", C, RF);
    #10;
    $display("Output E = %h", E);
    $display("");

    C = 5'b11111;
    $display("Input: C = %b, RF = %b", C, RF);
    #10;
    $display("Output E = %h", E);
    $display("");

    C = 5'b11001;
    $display("Input: C = %b, RF = %b", C, RF);
    #10;
    $display("Output E = %h", E);
    $display("");

    // Test with RF set to 0
    RF = 1'b0;
    $display("Input: C = %b, RF = %b", C, RF);
    #10;
    $display("Output E = %h", E);
    $display("");

    // End of simulation
    $finish;
  end

endmodule