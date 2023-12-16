module HAZARD_FORWARDING_UNIT ( 
    output reg[1:0] pa_selector, pb_selector, hazard_type,
    output reg load_enable, pc_enable, nop_signal,  
    input wire[4:0] ex_destination, mem_destination, wb_destination,
    input wire[4:0] id_rs, id_rt,
    input wire ex_rf_enable, mem_rf_enable, wb_rf_enable, ex_load_instruction, mem_load_instruction
);
  
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
        hazard_type = 2'b00; 
  
        if (ex_load_instruction && (id_rs == ex_destination || id_rt == ex_destination)) begin
            load_enable_val = 1'b0;
            pc_enable_val = 1'b0;
            nop_signal_val = 1'b1;
            hazard_type = 2'b01; 
        end
        else begin
             //PA selector designation
            if (ex_rf_enable && id_rs == ex_destination) begin
                pa_selector_val = 2'b01;
                hazard_type = 2'b10; 
            end
            else if (mem_rf_enable && id_rs == mem_destination) begin
                pa_selector_val = 2'b10;
                hazard_type = 2'b10; 
            end
            else if (wb_rf_enable && id_rs == wb_destination) begin
                pa_selector_val = 2'b11;
                hazard_type = 2'b10; 
            end

            //PB selector designation
            if (ex_rf_enable && id_rt == ex_destination) begin
                pb_selector_val = 2'b01;
                hazard_type = 2'b11; 
            end
            else if (mem_rf_enable && id_rt == mem_destination) begin
                pb_selector_val = 2'b10;
                hazard_type = 2'b11; 
            end
            else if (wb_rf_enable && id_rt == wb_destination) begin
                pb_selector_val = 2'b11;
                hazard_type = 2'b11; 
            end
        end

        pa_selector = pa_selector_val;
        pb_selector = pb_selector_val;
        load_enable = load_enable_val;
        pc_enable = pc_enable_val; 
        nop_signal = nop_signal_val;
    end
endmodule