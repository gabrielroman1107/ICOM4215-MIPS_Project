module HAZARD_FORWARDING_UNIT (//UPDATE AND CHANGE 
    output reg[1:0] pa_selector, pb_selector,
    output reg load_enable, pc_enable, nop_signal,  //The pc enable must also be used for npc
    input wire[4:0] ex_destination, mem_destination, wb_destination,
    input wire[4:0] id_rs, id_rt,
    input wire ex_rf_enable, mem_rf_enable, wb_rf_enable, ex_load_instruction, mem_load_instruction
);
    /*Forwarding code for PA and PB MUX

    PA or PB = 2'b00
    Ex_Rd = 2'b01
    Mem_Rd = 2'b10
    Wb_Rd = 2'b11

    

    */

    //Default values
    reg[1:0] pa_selector_val;
    reg[1:0] pb_selector_val;
    reg load_enable_val;
    reg pc_enable_val;
    reg nop_signal_val;

    always @(*) begin

        pa_selector_val = 2'b00;
        pb_selector_val = 2'b00;
        load_enable_val = 1'b1;
        pc_enable_val = 1'b1;
        nop_signal_val = 1'b0;
        // if(mem_load_instruction && (id_rs == mem_destination || id_rt == mem_destination)) begin
        //     //Forward mem to pa
        //     if (id_rs == mem_destination) begin
        //         pa_selector_val = 2'b10;
        //     end
        //     //Forward mem to pb
        //     if (id_rt == mem_destination) begin
        //         pb_selector_val = 2'b10;
        //     end
        // end
        //Load Data hazard
        if (ex_load_instruction && (id_rs == ex_destination || id_rt == ex_destination)) begin
            load_enable_val = 1'b0;
            pc_enable_val = 1'b0;
            nop_signal_val = 1'b1;
        end
        else begin
             //PA selector designation
            if (ex_rf_enable && id_rs == ex_destination) begin
                pa_selector_val = 2'b01;
            end
            else if (mem_rf_enable && id_rs == mem_destination) begin
                pa_selector_val = 2'b10;
            end
            else if (wb_rf_enable && id_rs == wb_destination) begin
                pa_selector_val = 2'b11;
            end

            //PB selector designation
            if (ex_rf_enable && id_rt == ex_destination) begin
                pb_selector_val = 2'b01;
            end
            else if (mem_rf_enable && id_rt == mem_destination) begin
                pb_selector_val = 2'b10;
            end
            else if (wb_rf_enable && id_rt == wb_destination) begin
                pb_selector_val = 2'b11;
            end
        end

        pa_selector = pa_selector_val;
        pb_selector = pb_selector_val;
        load_enable = load_enable_val;
        pc_enable = pc_enable_val; 
        nop_signal = nop_signal_val;
    end
endmodule