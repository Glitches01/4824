`timescale 1ns/1ps


module ROB (
    input                       clock,
    input                       reset,
    input                       squash_signal,//todo

    input       CDB_PACKET      CDB_packet_in,      //todo has to be one cycle???
    input       DP_ROB_PACKET   dp_rob_packet,//todo
    input                       enable,//enable for allocating the entry
    input       MT_ROB_PACKET   mt_rob_packet,   

    input       CDB_PACKET      lsq_input,
    input logic [$clog2(`LSQ_SIZE)-1:0]   lsq_idx,

    output      logic           available,    //todo
    output      CP_RT_PACKET    cp_rt_packet,   //todo
    output      ROB_RS_PACKET   rob_rs_packet,  
    output      ROB_MT_PACKET   rob_mt_packet,


    output logic [`XLEN-1:0] Branch_PC,
    output logic [`XLEN-1:0] Branch_Target,
    output logic take_branch,
    output logic IsBranch,
    output logic Branch_Miss//todo
);//todotodotodo every tag by default is 0, have to change

    ROB_ENTRY   ROB_content   [`ROB_SIZE-1:0];
    ROB_ENTRY   ROB_content_n [`ROB_SIZE-1:0];

    logic   [$clog2(`ROB_SIZE):0] head, head_n;
    logic   [$clog2(`ROB_SIZE):0] tail, tail_n;

    wire [$clog2(`ROB_SIZE)-1:0] head_idx = head[$clog2(`ROB_SIZE)-1:0];
    wire [$clog2(`ROB_SIZE)-1:0] tail_idx = tail[$clog2(`ROB_SIZE)-1:0];


    always_comb begin
        for (integer unsigned j = 0; j < `ROB_SIZE; j = j + 1)
            ROB_content_n[j] = ROB_content[j];

            tail_n = tail;
            head_n = head;

        if (CDB_packet_in.valid && (ROB_content[CDB_packet_in.Tag].PC == CDB_packet_in.PC) && !Branch_Miss) begin
            ROB_content_n[CDB_packet_in.Tag].inst           = CDB_packet_in.inst;
            ROB_content_n[CDB_packet_in.Tag].value          = CDB_packet_in.Value;
            ROB_content_n[CDB_packet_in.Tag].alu_result     = CDB_packet_in.alu_result;//todo
            ROB_content_n[CDB_packet_in.Tag].cp_bit         = 1'b1;
            ROB_content_n[CDB_packet_in.Tag].ep_bit         = CDB_packet_in.take_branch;
            ROB_content_n[CDB_packet_in.Tag].NPC            = CDB_packet_in.NPC;
            ROB_content_n[CDB_packet_in.Tag].PC             = CDB_packet_in.PC;
            ROB_content_n[CDB_packet_in.Tag].valid          = CDB_packet_in.valid;
            ROB_content_n[CDB_packet_in.Tag].halt           = CDB_packet_in.halt;
            ROB_content_n[CDB_packet_in.Tag].illegal        = CDB_packet_in.illegal;
        end

        if (lsq_input.valid && (ROB_content[lsq_input.Tag].PC == lsq_input.PC) && !Branch_Miss) begin
            ROB_content_n[lsq_input.Tag].inst           = lsq_input.inst;
            ROB_content_n[lsq_input.Tag].value          = lsq_input.Value;
            ROB_content_n[lsq_input.Tag].alu_result     = 0;//todo
            ROB_content_n[lsq_input.Tag].cp_bit         = 1'b1;
            ROB_content_n[lsq_input.Tag].ep_bit         = 0;
            ROB_content_n[lsq_input.Tag].NPC            = lsq_input.NPC;
            ROB_content_n[lsq_input.Tag].PC             = lsq_input.PC;
            ROB_content_n[lsq_input.Tag].valid          = lsq_input.valid;
            ROB_content_n[lsq_input.Tag].halt           = 0;
            ROB_content_n[lsq_input.Tag].illegal        = 0;
        end




        //allocate entry
        if (enable) begin
            rob_rs_packet.Tag = tail_idx;
            rob_mt_packet.Tag = tail_idx;
            tail_n = tail + 1;
        end else begin
            rob_rs_packet.Tag = tail_idx;//todo
            rob_mt_packet.Tag = tail_idx;//todo have changed
        end

        if (enable) begin
            for (integer unsigned k = 0; k < `ROB_SIZE; k = k + 1) begin
                if (k == tail_idx) begin
                    ROB_content_n[k].reg_idx = dp_rob_packet.dest_reg_idx;//destination
                    ROB_content_n[k].PC      = dp_rob_packet.PC;
                    ROB_content_n[k].NPC     = dp_rob_packet.NPC;
                    ROB_content_n[k].cp_bit  = 1'b0;
                    ROB_content_n[k].valid   = 1'b0;
                    ROB_content_n[k].IsBranch= dp_rob_packet.IsBranch;
                    if (dp_rob_packet.mem) begin
                        ROB_content_n[k].lsq_idx = lsq_idx;
                    end
                    ROB_content_n[k].wr_mem  = dp_rob_packet.wr_mem;
                    //ROB_content_n[k].inst= dp_rob_packet.IsBranch;
                    // ROB_content_n[k].CantComplete = 1'b0;
                end
            end
        end 
        
        if (ROB_content[head_idx].cp_bit) begin  //todo should i complete/cp parallel priority with enbale? or enable first
            ROB_content_n[head_idx].cp_bit = 1'b0;
            if (ROB_content[head_idx].IsBranch) begin//todo roll-back
                ROB_content_n[head_idx].ep_bit = 1'b0;
                if (Branch_Miss) begin
                    tail_n = head + 1;
                    //ROB_content_n[head_idx] = ROB_content[head_idx];
                    for (integer i = 0; i < `ROB_SIZE; i++) begin
                        if (i != head_idx) begin
                            ROB_content_n[i] = 0;
                        end
                    end
                    //ROB_content_n[head_idx] = ROB_content[head_idx];
                end
            end
            head_n = head + 1;
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

        if (CDB_packet_in.valid && (CDB_packet_in.Tag == mt_rob_packet.RegS1_Tag) && mt_rob_packet.valid_vector[0]) begin
            rob_rs_packet.rs1_value = CDB_packet_in.Value;
            rob_rs_packet.RegS1_Tag = mt_rob_packet.RegS1_Tag;
            rob_rs_packet.valid_vector[0] = 1'b1; 
            rob_rs_packet.complete[0] = 1'b1;
        end

        if (CDB_packet_in.valid && (CDB_packet_in.Tag == mt_rob_packet.RegS2_Tag) && mt_rob_packet.valid_vector[1]) begin
            rob_rs_packet.rs2_value = CDB_packet_in.Value;
            rob_rs_packet.RegS2_Tag = mt_rob_packet.RegS2_Tag;
            rob_rs_packet.valid_vector[1] = 1'b1; 
            rob_rs_packet.complete[1] = 1'b1;
        end

        if (lsq_input.valid && (lsq_input.Tag == mt_rob_packet.RegS1_Tag) && mt_rob_packet.valid_vector[0]) begin
            rob_rs_packet.rs1_value = lsq_input.Value;
            rob_rs_packet.RegS1_Tag = mt_rob_packet.RegS1_Tag;
            rob_rs_packet.valid_vector[0] = 1'b1; 
            rob_rs_packet.complete[0] = 1'b1;
        end

        if (lsq_input.valid && (lsq_input.Tag == mt_rob_packet.RegS2_Tag) && mt_rob_packet.valid_vector[1]) begin
            rob_rs_packet.rs2_value = lsq_input.Value;
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

    assign Branch_PC     = ROB_content[head_idx].PC;
    assign Branch_Target = take_branch? ROB_content[head_idx].alu_result : Branch_PC +4;
    assign take_branch   = ROB_content[head_idx].ep_bit;
    assign IsBranch      = ROB_content[head_idx].IsBranch;

    assign Branch_Miss   = (ROB_content[head_idx].ep_bit && (Branch_Target != ROB_content[head_idx].NPC)) 
                                    || (ROB_content[head_idx].cp_bit && ROB_content[head_idx].IsBranch && !ROB_content[head_idx].ep_bit && (ROB_content[head_idx].NPC != (ROB_content[head_idx].PC + 4)));

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
