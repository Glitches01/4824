module ReservationStation (
    input                   clock,
    input                   reset,

    input   DP_RS_PACKET    dp_rs_packet,
    input                   enable,
    input   ROB_RS_PACKET   rob_rs_packet,
    input   CDB_PACKET      cdb_packet,
    // output rs_dp_packet rs_dp_packet,

    output RS_EX_PACKET     rs_ex_packet,
    output logic            read_enable,
    output [2:0]            busy
);

    ////////////////////////////////////////////////
    //Get the Correct ready and rs_value from ROB MT DP
    //
    ///////////////////////////////////////////////
    logic [`XLEN:0] rs1_value, rs2_value;
    logic [$clog2(`ROB_SIZE)-1:0]  Tag;
    logic [$clog2(`ROB_SIZE)-1:0]  RS1_Tag, RS2_Tag;
    logic ready[0:1];
    logic [1:0] needTag;

    always_comb begin
        Tag = rob_rs_packet.Tag;

        rs1_value = dp_rs_packet.rs1_value;
        RS1_Tag = 0;
        needTag = 2'b0;
        ready[0] = 1;
        if (dp_rs_packet.inst.r.rs1 == 5'h0) begin
            rs1_value = 0;
        end else if (rob_rs_packet.valid_vector[0]) begin
            if(rob_rs_packet.complete[0]) begin
                rs1_value = rob_rs_packet.rs1_value;
                ready[0] = 1;
            end else begin
                rs1_value = 0;
                RS1_Tag = rob_rs_packet.RegS1_Tag;
                ready[0] = 0;
                needTag[0] = 1'b1;
            end
        end

        rs2_value = dp_rs_packet.rs2_value;
        RS2_Tag = 0;
        ready[1] = 1;
        if (dp_rs_packet.inst.r.rs2 == 5'h0) begin
            rs2_value = 0;
        end else if (rob_rs_packet.valid_vector[1]) begin
            if(rob_rs_packet.complete[1]) begin
                rs2_value = rob_rs_packet.rs2_value;
                ready[1] = 1;
            end else begin
                rs2_value = 0;
                RS2_Tag = rob_rs_packet.RegS2_Tag;
                ready[1] = 0;
                needTag[1] = 1'b1;
            end
        end
    end

    logic [1:0] gnt;
    priority_encoder #(
        .WIDTH(2)
    )u_priority_encoder(
        .req({!RS_Alu[1].busy,!RS_Alu[0].busy}),
        .gnt(gnt)
    );

    ///////////////////////////////////////////////////
    // Control for reading from Instruction Buffer
    //
    ///////////////////////////////////////////////////
    logic prev_read_enable;
    wire rs_available = (!busy[0] || !busy[1] || 0) && !prev_read_enable;//todo to fast maybe gg

    // 时序逻辑：生成 read_enable
    always @(posedge clock) begin
        if (reset) begin
            read_enable      <= 1'b0;
            prev_read_enable <= 1'b0;
        end else begin
            // 生成单周期脉冲
            read_enable <= rs_available;
            // 记录上一周期的读操作状态
            prev_read_enable <= read_enable;
        end
    end

    RS RS_Alu[0:1];
    logic [1:0] enable_alu;
    assign enable_alu[0] = 1 && (!RS_Alu[0].busy) && dp_rs_packet.valid && !{dp_rs_packet.rd_mem, dp_rs_packet.wr_mem} && enable && (gnt[0] == 1'b1);
    assign busy[0] = RS_Alu[0].busy;
    assign busy[1] = RS_Alu[1].busy;
    assign busy[2] = RS_Mem.busy;

    logic [2:0] gnt_rs;
    //assign rs_available = |{!busy[0], !busy[1], 0};
    ResSta u_RS0(
        .clock(clock),
        .reset(reset),

        .flush(cdb_packet.take_branch),
        .enable(enable_alu[0]),
        .dp_rs_packet(dp_rs_packet),
        .RS1_Tag(RS1_Tag),.RS2_Tag(RS2_Tag),.Tag(Tag),.needTag(needTag),
        .rs1_value(rs1_value),.rs2_value(rs2_value),

        .ready(ready),
        .cdb_packet(cdb_packet),

        .issue(gnt_rs[2]),




        //output for issue
        .rs(RS_Alu[0])
    );

    assign enable_alu[1] = 1 && (!RS_Alu[1].busy) && dp_rs_packet.valid && !{dp_rs_packet.rd_mem, dp_rs_packet.wr_mem} && enable && (gnt[1] == 1'b1);
    ResSta u_RS1(
        .clock(clock),
        .reset(reset),

        .flush(cdb_packet.take_branch),
        .enable(enable_alu[1]),
        .dp_rs_packet(dp_rs_packet),
        .RS1_Tag(RS1_Tag),.RS2_Tag(RS2_Tag),.Tag(Tag),
        .rs1_value(rs1_value),.rs2_value(rs2_value),

        .ready(ready),
        .cdb_packet(cdb_packet),

        .issue(gnt_rs[1]),



        //output for issue
        .rs(RS_Alu[1])
    );

    RS RS_Mem;
    logic enable_mem;
    assign enable_mem = 1 && (!RS_Mem.busy) && dp_rs_packet.valid && |{dp_rs_packet.rd_mem, dp_rs_packet.wr_mem} && enable;
    ResSta u_RS_MEM(
        .clock(clock),
        .reset(reset),

        .flush(cdb_packet.take_branch),
        .enable(enable_mem),
        .dp_rs_packet(dp_rs_packet),
        .RS1_Tag(RS1_Tag),.RS2_Tag(RS2_Tag),.Tag(Tag),
        .rs1_value(rs1_value),.rs2_value(rs2_value),

        .ready(ready),
        .cdb_packet(cdb_packet),

        .issue(gnt_rs[0]),




        //output for issue
        .rs(RS_Mem)
    );


    /////////////////////////////////////////////////////
    // Issue when RS get ready
    //
    /////////////////////////////////////////////////////
    RS RS_issue;
    logic issue, issue_alu1, issue_alu2, issue_mem;
    assign issue_alu1 = RS_Alu[0].ready[0] && RS_Alu[0].ready[1];
    assign issue_alu2 = RS_Alu[1].ready[0] && RS_Alu[1].ready[1];
    assign issue_mem  = RS_Mem.ready[0] && RS_Mem.ready[1];
    // assign issue = |{issue_alu1, issue_alu2, issue_mem};

    priority_encoder #(
        .WIDTH(3)
    )u_priority_encoder_rs(
        .req({issue_alu1,issue_alu2,issue_mem}),
        .gnt(gnt_rs),
        .valid(issue)
    );

    always_comb begin
        case (gnt_rs)
            3'b100: begin
                RS_issue = RS_Alu[0];
            end
            3'b010: begin
                RS_issue = RS_Alu[1];
            end
            3'b001: begin
                RS_issue = RS_Mem;
            end
            default: RS_issue = 0;
        endcase
    end

    // always_comb begin
    //     case (1'b1)
    //         issue_alu1: begin
    //             RS_issue = RS_Alu[0];
    //         end
    //         issue_alu2: begin
    //             RS_issue = RS_Alu[1];
    //         end
    //         issue_mem: begin
    //             RS_issue = RS_Mem;
    //         end
    //         default: RS_issue = 0;
    //     endcase
    // end

    always_ff @( posedge clock ) begin
        if(reset || cdb_packet.take_branch) begin
            rs_ex_packet <= 0;
        end else if (issue) begin
            rs_ex_packet.inst           <= RS_issue.inst;
            rs_ex_packet.PC             <= RS_issue.PC;
            rs_ex_packet.NPC            <= RS_issue.NPC; 

            rs_ex_packet.rs1_value      <= RS_issue.rs1_value; 
            rs_ex_packet.rs2_value      <= RS_issue.rs2_value; 

            rs_ex_packet.Tag            <= RS_issue.Tag;

            rs_ex_packet.opa_select     <= RS_issue.opa_select; 
            rs_ex_packet.opb_select     <= RS_issue.opb_select;
            rs_ex_packet.dest_reg_idx   <= RS_issue.dest_reg_idx; 
            rs_ex_packet.alu_func       <= RS_issue.alu_func;     
            rs_ex_packet.rd_mem         <= RS_issue.rd_mem;       
            rs_ex_packet.wr_mem         <= RS_issue.wr_mem;        
            rs_ex_packet.cond_branch    <= RS_issue.cond_branch;  
            rs_ex_packet.uncond_branch  <= RS_issue.uncond_branch; 
            rs_ex_packet.halt           <= RS_issue.halt;          
            rs_ex_packet.illegal        <= RS_issue.illegal;      
            rs_ex_packet.csr_op         <= RS_issue.csr_op;       
            rs_ex_packet.valid          <= RS_issue.valid;
        end else begin
            rs_ex_packet.valid          <= 0;
        end
    end


endmodule

module priority_encoder #(
    parameter WIDTH = 8
)(
    input  wire [WIDTH-1:0] req,
    output reg  [WIDTH-1:0] gnt,
    output reg              valid
);

    integer i;

    always @(*) begin
        gnt = 0;
        valid = 0;
        for (i = WIDTH-1; i >= 0; i = i - 1) begin
            if (!valid && req[i]) begin
                gnt[i] = 1'b1;
                valid = 1;
            end
        end
    end

endmodule


module ResSta #(
    
) (
    input clock, reset,

    input flush,
    //dispatch
    input enable,
    input DP_RS_PACKET              dp_rs_packet,
    //rob and mt and dp  value
    input [$clog2(`ROB_SIZE)-1:0]   RS1_Tag, RS2_Tag, Tag,
    input [`XLEN:0]                 rs1_value, rs2_value,
    input [1:0] needTag,
    //deassert rs
    input                           ready[0:1],
    input CDB_PACKET                cdb_packet,
    //for deassert issued rs
    input                           issue,

    output RS rs
);
    always_ff @( posedge clock ) begin
        if (reset) begin
            rs <= 0;
        end
        else if (flush) begin
            rs <= 0;
        end
        else if (enable) begin
            rs.inst             <= dp_rs_packet.inst;
            rs.NPC              <= dp_rs_packet.NPC;
            rs.PC               <= dp_rs_packet.PC;
            rs.busy             <= 1;

            rs.Tag              <= Tag;

            rs.RegS1_Tag        <= RS1_Tag;
            rs.RegS2_Tag        <= RS2_Tag;

            rs.ready[0]         <= ready[0];
            rs.ready[1]         <= ready[1];

            rs.rs1_value        <= rs1_value;
            rs.rs2_value        <= rs2_value;
            rs.opa_select       <= dp_rs_packet.opa_select;
            rs.opb_select       <= dp_rs_packet.opb_select;
            rs.dest_reg_idx     <= dp_rs_packet.dest_reg_idx;
            rs.alu_func         <= dp_rs_packet.alu_func;
            rs.rd_mem           <= dp_rs_packet.rd_mem;
            rs.wr_mem           <= dp_rs_packet.wr_mem;
            rs.cond_branch      <= dp_rs_packet.cond_branch;
            rs.uncond_branch    <= dp_rs_packet.uncond_branch;
            rs.halt             <= dp_rs_packet.halt;
            rs.illegal          <= dp_rs_packet.illegal;
            rs.csr_op           <= dp_rs_packet.csr_op;
            rs.valid            <= dp_rs_packet.valid;
            rs.func_unit        <= 1;
            rs.received         <= 2'b00;
            rs.needTag          <= needTag;
        end

        if(cdb_packet.valid) begin
            if ((cdb_packet.Tag == rs.RegS1_Tag) && (cdb_packet.dest_reg_idx != `ZERO_REG) && !rs.received[0] && !rs.ready[0] && rs.needTag[0]) begin
                rs.rs1_value <= cdb_packet.Value;
                rs.ready[0]  <= 1;
                rs.received[0] <= 1;
            end
            if ((cdb_packet.Tag == rs.RegS2_Tag) && (cdb_packet.dest_reg_idx != `ZERO_REG) && !rs.received[1] && !rs.ready[1] && rs.needTag[1]) begin
                rs.rs2_value <= cdb_packet.Value;
                rs.ready[1]  <= 1;
                rs.received[1] <= 1;
            end
            if(cdb_packet.valid && (cdb_packet.Tag == rs.Tag)) begin
                rs.busy <= 0;
            end
        end
        
        if ((issue)) begin//todo
            rs.ready[0]         <= 1'b0;
            rs.ready[1]         <= 1'b0;
        end
    end
endmodule