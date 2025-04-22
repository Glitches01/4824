`timescale 1ns/1ps


module ROB (
    input                       clock,
    input                       reset,
    input                       squash_signal,//todo

    input       CDB_PACKET      CDB_packet_in,      //todo has to be one cycle???
    input       DP_ROB_PACKET   dp_rob_packet,//todo
    input                       enable,//enable for allocating the entry
    input       MT_ROB_PACKET   mt_rob_packet,   

    output      logic           available,    //todo
    output      CP_RT_PACKET    cp_rt_packet,   //todo
    output      ROB_RS_PACKET   rob_rs_packet,  
    output      ROB_MT_PACKET   rob_mt_packet
);

    ROB_ENTRY   ROB_content   [`ROB_SIZE-1:0];
    ROB_ENTRY   ROB_content_n [`ROB_SIZE-1:0];

    logic   [$clog2(`ROB_SIZE):0] head, head_n;
    logic   [$clog2(`ROB_SIZE):0] tail, tail_n;

    wire [$clog2(`ROB_SIZE)-1:0] head_idx = head[$clog2(`ROB_SIZE)-1:0];
    wire [$clog2(`ROB_SIZE)-1:0] tail_idx = tail[$clog2(`ROB_SIZE)-1:0];

    always_comb begin
        for (integer unsigned j = 0; j < `ROB_SIZE; j = j + 1)
            ROB_content_n[j] = ROB_content[j];
        //todo should i complete/cp parallel priority with enbale? or enable first
        if (ROB_content[head_idx].cp_bit) begin
            ROB_content_n[head_idx].cp_bit = 1'b0;
            head_n = head + 1;
        end else begin
            head_n = head;
        end

	if (ROB_content[head_idx].ep_bit) begin//todo roll-back
                ROB_content_n[head_idx].ep_bit = 1'b0;
                tail_n = head_idx + 1;//?ep_idx
		head = head;
        end

        if (CDB_packet_in.valid) begin
            ROB_content_n[CDB_packet_in.Tag].value   = CDB_packet_in.Value;
            ROB_content_n[CDB_packet_in.Tag].cp_bit  = 1'b1;
            ROB_content_n[CDB_packet_in.Tag].ep_bit  = CDB_packet_in.take_branch;
            ROB_content_n[CDB_packet_in.Tag].NPC     = CDB_packet_in.NPC;
            ROB_content_n[CDB_packet_in.Tag].valid   = CDB_packet_in.valid;
        end




        //allocate entry
        if (enable) begin
            rob_rs_packet.Tag = tail_idx;
            rob_mt_packet.Tag = tail_idx;
            tail_n = tail + 1;
        end else begin
            rob_rs_packet.Tag = tail_idx;//todo
            rob_mt_packet.Tag = tail_idx;//todo have changed
            tail_n = tail;
        end

        if (enable) begin
            for (integer unsigned k = 0; k < `ROB_SIZE; k = k + 1) begin
                if (k == tail_idx) begin
                    ROB_content_n[k].reg_idx = dp_rob_packet.dest_reg_idx;//destination
                    ROB_content_n[k].PC      = dp_rob_packet.PC;
                    ROB_content_n[k].NPC     = dp_rob_packet.NPC;
                    ROB_content_n[k].cp_bit  = 1'b0;
                    ROB_content_n[k].valid   = 1'b0;
                end
            end
        end

        // if (enable) begin
        //     for (integer unsigned k = 0; k < `ROB_SIZE; k = k + 1) begin
        //         if (k == tail_idx) begin
        //             ROB_content_n[k].reg_idx = dp_rob_packet.dest_reg_idx;//destination
        //             ROB_content_n[k].PC      = dp_rob_packet.PC;
        //             ROB_content_n[k].NPC      = dp_rob_packet.NPC;
        //             ROB_content_n[k].valid   = 1'b0;
        //         end else begin
        //             ROB_content_n[k].reg_idx = ROB_content[k].reg_idx;
        //             ROB_content_n[k].PC      = ROB_content[k].PC;
        //             ROB_content_n[k].NPC      = ROB_content[k].NPC;
        //             ROB_content_n[k].valid   = ROB_content_n[k].valid;
        //         end
        //     end
        // end else begin
        //     for (integer unsigned k = 0; k < `ROB_SIZE; k = k + 1) begin
        //         ROB_content_n[k].reg_idx = ROB_content[k].reg_idx;
        //         ROB_content_n[k].PC      = ROB_content[k].PC;
        //         ROB_content_n[k].valid   = ROB_content_n[k].valid;
        //     end
        // end
    end

    //for RS
    always_comb begin
        rob_rs_packet.rs1_value = mt_rob_packet.valid_vector[0] ? ROB_content[mt_rob_packet.RegS1_Tag].value : 0;//source
        rob_rs_packet.rs2_value = mt_rob_packet.valid_vector[1] ? ROB_content[mt_rob_packet.RegS2_Tag].value : 0;

        rob_rs_packet.RegS1_Tag = mt_rob_packet.RegS1_Tag;
        rob_rs_packet.RegS2_Tag = mt_rob_packet.RegS2_Tag;

        rob_rs_packet.valid_vector = mt_rob_packet.valid_vector;

        rob_rs_packet.complete[0] = ROB_content[mt_rob_packet.RegS1_Tag].valid;
        rob_rs_packet.complete[1] = ROB_content[mt_rob_packet.RegS2_Tag].valid;

        if (CDB_packet_in.valid && CDB_packet_in.Tag == mt_rob_packet.RegS1_Tag) begin
            rob_rs_packet.rs1_value = CDB_packet_in.Value;
            rob_rs_packet.RegS1_Tag = mt_rob_packet.RegS1_Tag;
            rob_rs_packet.valid_vector[0] = 1'b1; 
            rob_rs_packet.complete[0] = 1'b1;
        end

        if (CDB_packet_in.valid && CDB_packet_in.Tag == mt_rob_packet.RegS2_Tag) begin
            rob_rs_packet.rs2_value = CDB_packet_in.Value;
            rob_rs_packet.RegS2_Tag = mt_rob_packet.RegS2_Tag;
            rob_rs_packet.valid_vector[1] = 1'b1; 
            rob_rs_packet.complete[1] = 1'b1;
        end   
    end


    // assign rob_rs_packet.rs1_value = mt_rob_packet.valid_vector[0] ? ROB_content[mt_rob_packet.RegS1_Tag].value : 0;//source
    // assign rob_rs_packet.rs2_value = mt_rob_packet.valid_vector[1] ? ROB_content[mt_rob_packet.RegS2_Tag].value : 0;
    // assign rob_rs_packet.RegS1_Tag = mt_rob_packet.RegS1_Tag;
    // assign rob_rs_packet.RegS2_Tag = mt_rob_packet.RegS2_Tag;
    // assign rob_rs_packet.valid_vector = mt_rob_packet.valid_vector;
    // assign rob_rs_packet.complete[0] = ROB_content[mt_rob_packet.RegS1_Tag].valid;
    // assign rob_rs_packet.complete[1] = ROB_content[mt_rob_packet.RegS2_Tag].valid;



    //for retire
    assign cp_rt_packet.rob_entry = ROB_content[head_idx];
    assign cp_rt_packet.Tag       = head_idx;

    //for full? rob_entry_available
    logic [$clog2(`ROB_SIZE)-1:0] next_tail;
    assign next_tail = (tail_idx == `ROB_SIZE-1) ? 0 : tail_idx + 1;
    always_comb begin
        if (next_tail == head_idx)
            available = 1'b0;
        else
            available = 1'b1;
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            ROB_content <= '{`ROB_SIZE{0}};
            head        <= 0;
            tail        <= 0;
        end else if (squash_signal) begin
            ROB_content <= '{`ROB_SIZE{0}};
            head        <= 0;
            tail        <= 0;
        end else begin
            ROB_content <= ROB_content_n;
            head        <= head_n;
            tail        <= tail_n;
        end
    end

endmodule
