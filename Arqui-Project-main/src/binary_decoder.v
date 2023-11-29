// module binary_decoder (output reg [15:0] D, input [3:0] S);

// //Input (S): 4 bit selector (16 options), 2^4=16
// //Output (D): 16 bit, Selected line 1, other lines 0

// always @ (D, S)

// 	casez (S)
// 		//Setting all output lines to 0, selected bit in D set to 1
// 		4'b0000: begin D = 16'b0; D[0] = 1; end
// 		4'b0001: begin D = 16'b0; D[1] = 1; end
// 		4'b0010: begin D = 16'b0; D[2] = 1; end
// 		4'b0011: begin D = 16'b0; D[3] = 1; end
// 		4'b0100: begin D = 16'b0; D[4] = 1; end
// 		4'b0101: begin D = 16'b0; D[5] = 1; end
// 		4'b0110: begin D = 16'b0; D[6] = 1; end
// 		4'b0111: begin D = 16'b0; D[7] = 1; end
// 		4'b1000: begin D = 16'b0; D[8] = 1; end
// 		4'b1001: begin D = 16'b0; D[9] = 1; end
// 		4'b1010: begin D = 16'b0; D[10] = 1; end
// 		4'b1011: begin D = 16'b0; D[11] = 1; end
// 		4'b1100: begin D = 16'b0; D[12] = 1; end
// 		4'b1101: begin D = 16'b0; D[13] = 1; end
// 		4'b1110: begin D = 16'b0; D[14] = 1; end
// 		4'b1111: begin D = 16'b0; D[15] = 1; end
// 	endcase

// endmodule

// module binary_decoder (output reg [31:0] D, input [4:0] S);

// //Input (S): 5 bit selector (32 options), 2^5=32
// //Output (D): 32 bit, Selected line 1, other lines 0

// always @ (D, S)

// 	casez (S)
// 		//Setting all output lines to 0, selected bit in D set to 1
// 		5'b00000: begin D = 32'b0; D[0] = 1; end
// 		5'b00001: begin D = 32'b0; D[1] = 1; end
// 		5'b00010: begin D = 32'b0; D[2] = 1; end
// 		5'b00011: begin D = 32'b0; D[3] = 1; end
// 		5'b00100: begin D = 32'b0; D[4] = 1; end
// 		5'b00101: begin D = 32'b0; D[5] = 1; end
// 		5'b00110: begin D = 32'b0; D[6] = 1; end
// 		5'b00111: begin D = 32'b0; D[7] = 1; end
// 		5'b01000: begin D = 32'b0; D[8] = 1; end
// 		5'b01001: begin D = 32'b0; D[9] = 1; end
// 		5'b01010: begin D = 32'b0; D[10] = 1; end
// 		5'b01011: begin D = 32'b0; D[11] = 1; end
// 		5'b01100: begin D = 32'b0; D[12] = 1; end
// 		5'b01101: begin D = 32'b0; D[13] = 1; end
// 		5'b01110: begin D = 32'b0; D[14] = 1; end
// 		5'b01111: begin D = 32'b0; D[15] = 1; end
// 		5'b10000: begin D = 32'b0; D[16] = 1; end
// 		5'b10001: begin D = 32'b0; D[17] = 1; end
// 		5'b10010: begin D = 32'b0; D[18] = 1; end
// 		5'b10011: begin D = 32'b0; D[19] = 1; end
// 		5'b10100: begin D = 32'b0; D[20] = 1; end
// 		5'b10101: begin D = 32'b0; D[21] = 1; end
// 		5'b10110: begin D = 32'b0; D[22] = 1; end
// 		5'b10111: begin D = 32'b0; D[23] = 1; end
// 		5'b11000: begin D = 32'b0; D[24] = 1; end
// 		5'b11001: begin D = 32'b0; D[25] = 1; end
// 		5'b11010: begin D = 32'b0; D[26] = 1; end
// 		5'b11011: begin D = 32'b0; D[27] = 1; end
// 		5'b11100: begin D = 32'b0; D[28] = 1; end
// 		5'b11101: begin D = 32'b0; D[29] = 1; end
// 		5'b11110: begin D = 32'b0; D[30] = 1; end
// 		5'b11111: begin D = 32'b0; D[31] = 1; end
// 	endcase

// endmodule

// module BinaryDecoder (
//     input [4:0] C, // 5-bit binary input
//     input RF,      // 1-bit binary input
//     output reg [31:0] E // 32-bit binary output
// );

//     always @ (*) begin
//         if (RF == 1'b1) begin
//             // Initialize E to all zeros
//             E = 32'b0;
            
//             // Set the selected bit based on the 5-bit input C
//             E[C] = 1'b1;
//         end
//         else begin
//             // When RF is 0, set E to all zeros
//             E = 32'b0;
//         end
//     end
// endmodule

module BinaryDecoder (
  output reg [31:0] E, // Binary output (32 bit)
  input [4:0] C, // Binary input (5 bit)
  input RF // Binary input (1 bit)
   
  
);
    always @ (*) begin
        if (RF == 1'b1) // Checks if it's 0 or 1
          begin
              case (C)
              5'b00000: E = 32'b00000000000000000000000000000001;
              5'b00001: E = 32'b00000000000000000000000000000010;
              5'b00010: E = 32'b00000000000000000000000000000100;
              5'b00011: E = 32'b00000000000000000000000000001000;
              5'b00100: E = 32'b00000000000000000000000000010000;
              5'b00101: E = 32'b00000000000000000000000000100000;
              5'b00110: E = 32'b00000000000000000000000001000000;
              5'b00111: E = 32'b00000000000000000000000010000000;
              5'b01000: E = 32'b00000000000000000000000100000000;
              5'b01001: E = 32'b00000000000000000000001000000000;
              5'b01010: E = 32'b00000000000000000000010000000000;
              5'b01011: E = 32'b00000000000000000000100000000000;
              5'b01100: E = 32'b00000000000000000001000000000000;
              5'b01101: E = 32'b00000000000000000010000000000000;
              5'b01110: E = 32'b00000000000000000100000000000000;
              5'b01111: E = 32'b00000000000000001000000000000000;
              5'b10000: E = 32'b00000000000000010000000000000000;
              5'b10001: E = 32'b00000000000000100000000000000000;
              5'b10010: E = 32'b00000000000001000000000000000000;
              5'b10011: E = 32'b00000000000010000000000000000000;
              5'b10100: E = 32'b00000000000100000000000000000000;
              5'b10101: E = 32'b00000000001000000000000000000000;
              5'b10110: E = 32'b00000000010000000000000000000000;
              5'b10111: E = 32'b00000000100000000000000000000000;
              5'b11000: E = 32'b00000001000000000000000000000000;
              5'b11001: E = 32'b00000010000000000000000000000000;
              5'b11010: E = 32'b00000100000000000000000000000000;
              5'b11011: E = 32'b00001000000000000000000000000000;
              5'b11100: E = 32'b00010000000000000000000000000000;
              5'b11101: E = 32'b00100000000000000000000000000000;
              5'b11110: E = 32'b01000000000000000000000000000000;
              5'b11111: E = 32'b10000000000000000000000000000000;
              endcase
          end
        else 
          E = 32'b00000000000000000000000000000000;
    end
endmodule
