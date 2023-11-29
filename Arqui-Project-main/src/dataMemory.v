module DataMemory(
    input [8:0] A, // Address input
    input [31:0] DI, // Data input
    input [1:0] Size, // Data size: 00 (byte), 01 (halfword), 10 (word)
    input R_W, // Read/Write signal: 0 (Read), 1 (Write)
    input E, // Enable signal
    input SE, // Sign extension signal for halfword and byte operations
    output reg [31:0] DO // Data output
);

reg [7:0] mem [0:511]; // RAM memory with 512 locations, each 8 bits wide


always @(*) begin
    if (E) begin
        if (R_W == 0) begin
        // Reading operation
            if (Size == 2'b00)begin 
                if(SE == 0)begin
                    DO = {24'b0, mem[A]}; // Byte read
                end
                else begin 
                    if (mem[A][7] == 1) begin
                        DO = {{24'b111111111111111111111111}, mem[A]};
                    end
                    else begin
                        DO = {{24'b0}, mem[A]};
                    end
                     
                end
            
        end
            else if (Size == 2'b01)begin 
                if(SE == 0)begin

                    DO = {16'b0, mem[A], mem[A+1]}; // Halfword read
                end
                else begin 
                    if (mem[A][7] == 1) begin
                        DO = {{16'b1111111111111111}, mem[A], mem[A+1]};
                    end
                    else begin
                        DO = {{16'b0}, mem[A], mem[A+1]};
                    end
                end
            end

            else begin
                DO = {mem[A], mem[A+1],  mem[A+2],  mem[A+3]};
            end
        end
     else  begin
        // Writing operation
        if (Size == 2'b00) begin
            mem[A] = DI[7:0]; // Byte write
            
        end 
        else if (Size == 2'b01) begin

            mem[A] = DI[15:8]; // Halfword write
            mem[A+1] = DI[7:0];

        end 
        else if (Size == 2'b10) begin

            mem[A] = DI[31:24]; // Word write
            mem[A+1] = DI[23:16];
            mem[A+2] = DI[15:8];
            mem[A+3] = DI[7:0];
        end    
    end
        
    end
end
endmodule

module DataMemory_tb;

    reg [8:0] A;
    reg [31:0] DI;
    reg [1:0] Size;
    reg R_W;
    reg E;
    reg SE;
    wire [31:0] DO;

    reg finished1, finished2, finished3, finished4, finished5;

    // Instantiate the DataMemory module
    DataMemory DataMemory(
        .A(A),
        .DI(DI),
        .Size(Size),
        .R_W(R_W),
        .E(E),
        .SE(SE),
        .DO(DO)
    );

    initial begin
        $readmemb("precargaTest.txt", DataMemory.mem);
        // Start the tests
        #1
        test_case_1();
    end

    // Test case 1: Read a word from locations 0, 4, 8, and 12
    task test_case_1;
    begin
        A = 9'b0; // Address  = 0
        Size = 2'b10; // Word read
        R_W = 0; // Read operation
        E = 1;
        SE = 1'b0;
         
        #1
        $display("Test Case 1: Read a word from location %d", A);
        $display("Address =%d DataOut= %04h\n", A, DO);
        #1
        A = A + 4; // Address  = 4
         
        #1
        $display("Test Case 1: Read a word from location %d", A);
        $display("Address =%d DataOut= %04h\n", A, DO);
        #1
        A = A + 4; // Address  = 8
         
        #1
        $display("Test Case 1: Read a word from location %d", A);
        $display("Address =%d DataOut= %04h\n", A, DO);
        #1
        A = A + 4; // Address  = 12
         
        #1
        $display("Test Case 1: Read a word from location %d", A);
        $display("Address =%d DataOut= %04h\n", A, DO);
        #1
        finished1 = 1;
         
        // Start the next test
        test_case_2();
    end
    endtask

    // Test case 2: Read unsigned byte from location 0, halfword from location 2, and halfword from location 4
    task test_case_2;
    begin
        #1
        wait(finished1);
        A = 9'b0; // Address  = 0
        Size = 2'b00; // Byte read
        R_W = 0; // Read operation
        E = 1;
        SE = 1'b0;
         
        #1
        $display("Test Case 2:Read unsigned byte from location %d", A);
        $display("Address =%d DataOut= %04h\n", A, DO);
        #1
        A = A + 2; // Address  = 2
        Size = 2'b01; // Halfword read
         
        #1
        $display("Test Case 2:Read unsigned halfword from location %d", A);
        $display("Address =%d DataOut= %04h\n", A, DO);
        #1
        A = A + 2; // Address  = 4
        Size = 2'b01; // Halfword read
         
        #1
        $display("Test Case 2:Read unsigned halfword from location %d", A);
        $display("Address =%d DataOut= %04h\n", A, DO);
        #1
        finished2 = 1;
        #1 
        // Start the next test
        test_case_3();
    end
    endtask

    // Test case 3: Read signed byte from location 0, halfword from location 2, and halfword from location 4
    task test_case_3;
    begin
        #1
        wait(finished2);
            A = 9'b0; // Address  = 0
            Size = 2'b00; // Byte read
            R_W = 0; // Read operation
            E = 1;
            SE = 1'b1;
            #1
            $display("Test Case 3:Read signed byte from location %d", A);
            $display("Address =%d DataOut= %04h\n", A, DO);
            #1
            
            A = A + 2; // Address  = 2
            Size = 2'b01; // Halfword read
            
            #1
            $display("Test Case 3:Read signed halfword from location %d", A);
            $display("Address =%d DataOut= %04h\n", A, DO);
            #1
            
            A = A + 2; // Address  = 4
            Size = 2'b01; // Halfword read
            #1
            $display("Test Case 3:Read signed halfword from location %d", A);
            $display("Address =%d DataOut= %04h\n", A, DO);
            finished3 = 1;
            #1
        // Start the next test
        test_case_4();
    end
    endtask

    // Test case 4: Write a byte to location 0, a halfword in location 2, a halfword in location 4 and a word in location 8
    task test_case_4;
    begin
        wait(finished3);
        R_W = 1; // Read operation
        E = 1;

        A = 9'b0; // Address  = 0
        Size = 2'b00; // Byte write
        DI = 32'b00000000000000000000000001010101;

        #1
        $display("Test Case 4: Write a byte from location %d", A);
        $display("Succesful write of a byte from location %d\n", A);

        #1
        DI = 32'b00000000000000000000100001010101;
        A = A + 2;// Address  = 2 
        Size = 2'b01; // Halfword write
        #1
        $display("Test Case 4: Write a halfword from location %d", A);
        $display("Succesful write of a halfword from location %d\n", A);
        
        #1
        DI = 32'b00000000000000000000010001010101;
        A = A + 2;// Address  = 4
        Size = 2'b01; // Halfword write
        #1
        $display("Test Case 4: Write a halfword from location %d", A);
        $display("Succesful write of a halfword from location %d\n", A);
       
        #1
        A = A + 4;// Address  = 8
        Size = 2'b10; // Word write
        DI = 32'b00000000000100000000000000010101;
        #1
        $display("Test Case 4: Write a word from location %d", A);
        $display("Succesful write of a word from location %d\n", A);
        
        #1
        finished4 = 1;
        #1
        // Start the next test
        test_case_5();
    end
    endtask

    // Test case 5: Read a word from location 0, 4 and 8
    task test_case_5;
    begin
        #1
        wait(finished4);
        A = 9'b0; // Address  = 0
        Size = 2'b10; // Word read
        R_W = 0; // Read operation
        E = 1;
        SE = 1'b0;
       
        #1
        $display("Test Case 5: Read a word from location %d", A);
        $display("Address =%d DataOut= %04h\n", A, DO);
        #1
        A = A + 4; // Address  = 4
       
        #1
        $display("Test Case 5: Read a word from location %d", A);
        $display("Address =%d DataOut= %04h\n", A, DO);
        #1
        A = A + 4; // Address  = 8
       
        #1
        $display("Test Case 5: Read a word from location %d", A);
        $display("Address =%d DataOut= %04h\n", A, DO);
        #1
        // Finish the simulation     
        finished5 = 1;  
        wait(finished5);
        $finish;
    end
    endtask
endmodule




