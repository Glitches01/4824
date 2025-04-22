`timescale 1ns/100ps

// `define DEBUG
module ROB (
    input               clock,
    input               reset,
    input               enable,
    input               squash_signal,
    input               retire_disable,//store buffer full...
    input       CDB_PACKET    CDB_packet_in,      
    input       DP_PACKET     DP_packet_in,       
    input       MT_ROB_PACKET MT_ROB_packet_in,   

    output      logic   dp_available,       
    output      CP_RT_PACKET  CP_RT_packet_out,   
    output      ROB_RS_PACKET ROB_RS_packet_out,  
    output      ROB_MT_PACKET ROB_MT_packet_out    


    `ifdef DEBUG
    ,
    output logic [$clog2(`ROB_SIZE)-1:0] head_out,
    output logic [$clog2(`ROB_SIZE)-1:0] tail_out,
    output logic [$clog2(`ROB_SIZE):0]   space_out,
    output ROB_ENTRY [`ROB_SIZE-1:0]      content_out
    `endif
);

    logic   [$clog2(`ROB_SIZE)-1:0] space_available;
    ROB_ENTRY   ROB_content   [`ROB_SIZE-1:0];
    ROB_ENTRY   ROB_content_n [`ROB_SIZE-1:0];

    logic   [$clog2(`ROB_SIZE):0] head, head_n;
    logic   [$clog2(`ROB_SIZE):0] tail, tail_n;

    wire [$clog2(`ROB_SIZE)-1:0] head_idx = head[$clog2(`ROB_SIZE)-1:0];
    wire [$clog2(`ROB_SIZE)-1:0] tail_idx = tail[$clog2(`ROB_SIZE)-1:0];


    `ifdef DEBUG
    assign head_out  = head_idx;
    assign tail_out  = tail_idx;
    assign space_out = space_available;
    always_comb begin
        for (integer unsigned i = 0; i < `ROB_SIZE; i = i + 1)
            content_out[i] = ROB_content[i];
    end
    `endif


    always_comb begin
        for (integer unsigned j = 0; j < `ROB_SIZE; j = j + 1)
            ROB_content_n[j] = ROB_content[j];
        if (ROB_content[head_idx].cp_bit && (retire_disable == 1'b0)) begin
            ROB_content_n[head_idx].cp_bit = 1'b0;
            head_n = head + 1;
        end else begin
            head_n = head;
        end

        if (CDB_packet_in.valid) begin
            ROB_content_n[CDB_packet_in.Tag].value   = CDB_packet_in.Value;
            ROB_content_n[CDB_packet_in.Tag].cp_bit  = 1'b1;
            ROB_content_n[CDB_packet_in.Tag].ep_bit  = CDB_packet_in.take_branch;
            //ROB_content_n[CDB_packet_in.Tag].illegal = CDB_packet_in.illegal;
            //ROB_content_n[CDB_packet_in.Tag].halt    = CDB_packet_in.halt;
            ROB_content_n[CDB_packet_in.Tag].NPC     = CDB_packet_in.NPC;
            ROB_content_n[CDB_packet_in.Tag].valid   = CDB_packet_in.valid;
        end

        if (DP_packet_in.dp_en) begin
            ROB_RS_packet_out.Tag = tail_idx;
            ROB_MT_packet_out.Tag = tail_idx;
            tail_n = tail + 1;
        end else begin
            ROB_RS_packet_out.Tag = 0;
            ROB_MT_packet_out.Tag = 0;
            tail_n = tail;
        end

        if (DP_packet_in.dp_en) begin
            for (integer unsigned k = 0; k < `ROB_SIZE; k = k + 1) begin
                if (k == tail_idx) begin
                    ROB_content_n[k].reg_idx = DP_packet_in.dest_reg_idx;//destination
                    ROB_content_n[k].PC      = DP_packet_in.PC;
                end else begin
                    ROB_content_n[k].reg_idx = ROB_content[k].reg_idx;
                    ROB_content_n[k].PC      = ROB_content[k].PC;
                end
            end
        end else begin
            for (integer unsigned k = 0; k < `ROB_SIZE; k = k + 1) begin
                ROB_content_n[k].reg_idx = ROB_content[k].reg_idx;
                ROB_content_n[k].PC      = ROB_content[k].PC;
            end
        end
    end

    assign ROB_RS_packet_out.rs1_value = MT_ROB_packet_in.valid_vector[0] ? ROB_content[MT_ROB_packet_in.RegS1_Tag].value : 0;//source
    assign ROB_RS_packet_out.rs2_value = MT_ROB_packet_in.valid_vector[1] ? ROB_content[MT_ROB_packet_in.RegS2_Tag].value : 0;

    assign CP_RT_packet_out.rob_entry = ROB_content[head_idx];
    assign CP_RT_packet_out.Tag       = head_idx;

    logic [$clog2(`ROB_SIZE)-1:0] next_tail;
    assign next_tail = (tail_idx == `ROB_SIZE-1) ? 0 : tail_idx + 1;
    always_comb begin
        if (next_tail == head_idx)
            dp_available = 1'b0;
        else
            dp_available = 1'b1;
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            ROB_content <= `SD '{`ROB_SIZE{0}};
            head        <= `SD 0;
            tail        <= `SD 0;
        end else if (squash_signal) begin
            ROB_content <= `SD '{`ROB_SIZE{0}};
            head        <= `SD 0;
            tail        <= `SD 0;
        end else if (enable) begin
            ROB_content <= `SD ROB_content_n;
            head        <= `SD head_n;
            tail        <= `SD tail_n;
        end
    end

endmodule

