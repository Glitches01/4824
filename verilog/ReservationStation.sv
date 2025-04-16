module ReservationStation (
    input   clock,
    input   reset,

    input   DP_RS_PACKET dp_rs_packet,
    input   enable,
    input   ROB_RS_PACKET rob_rs_packet,
    input   CDB_PACKET cdb_packet,
    // output rs_dp_packet rs_dp_packet,

    output RS_EX_PACKET rs_ex_packet,
    output logic read_enable,
    output [2:0] busy
);

    logic [1:0] gnt;
    priority_encoder(
        .req({!rs_alu2.busy,!rs_alu.busy}),
        .gnt(gnt)
    );

    RS rs_alu;
    logic enable_alu;
    assign enable_alu = 1 && (!rs_alu.busy) && dp_rs_packet.valid && !{dp_rs_packet.rd_mem, dp_rs_packet.wr_mem} && enable && (gnt[0] == 1'b1);
    assign busy[0] = rs_alu.busy;
    assign busy[1] = rs_alu2.busy;
    assign busy[2] = rs_mem.busy;

    //assign rs_available = |{!busy[0], !busy[1], 0};

    logic [`XLEN:0] rs1_value, rs2_value;
    logic [$clog2(`ROB_SIZE)-1:0]  Tag;
    logic [$clog2(`ROB_SIZE)-1:0]  RS1_Tag, RS2_Tag;
    logic ready[0:1];

    
    logic prev_read_enable;

    // 组合逻辑：判断是否有 RS 空闲，且上一周期未触发读操作
    wire rs_available = (!busy[0] || !busy[1]) && !prev_read_enable;

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

    //rs1_value RS1_Tag rs2_value RS2_Tag Tag ready
    always_comb begin
        Tag = rob_rs_packet.Tag;

        rs1_value = dp_rs_packet.rs1_value;
        RS1_Tag = 0;
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
            end
        end
    end

    always_ff @( posedge clock ) begin
        if (reset) begin
            rs_alu <= 0;
        end
        else if (cdb_packet.take_branch) begin
            rs_alu <= 0;
        end
        else if (enable_alu) begin
            rs_alu.inst             <= dp_rs_packet.inst;
            rs_alu.NPC              <= dp_rs_packet.NPC;
            rs_alu.PC               <= dp_rs_packet.PC;
            rs_alu.busy             <= 1;

            rs_alu.Tag              <= Tag;

            rs_alu.RegS1_Tag        <= RS1_Tag;
            rs_alu.RegS2_Tag        <= RS2_Tag;

            rs_alu.ready[0]         <= ready[0];
            rs_alu.ready[1]         <= ready[1];

            rs_alu.rs1_value        <= rs1_value;
            rs_alu.rs2_value        <= rs2_value;
            rs_alu.opa_select       <= dp_rs_packet.opa_select;
            rs_alu.opb_select       <= dp_rs_packet.opb_select;
            rs_alu.dest_reg_idx     <= dp_rs_packet.dest_reg_idx;
            rs_alu.alu_func         <= dp_rs_packet.alu_func;
            rs_alu.rd_mem           <= dp_rs_packet.rd_mem;
            rs_alu.wr_mem           <= dp_rs_packet.wr_mem;
            rs_alu.cond_branch      <= dp_rs_packet.cond_branch;
            rs_alu.uncond_branch    <= dp_rs_packet.uncond_branch;
            rs_alu.halt             <= dp_rs_packet.halt;
            rs_alu.illegal          <= dp_rs_packet.illegal;
            rs_alu.csr_op           <= dp_rs_packet.csr_op;
            rs_alu.valid            <= dp_rs_packet.valid;
            rs_alu.func_unit        <= 1;
        end else if(cdb_packet.valid) begin
            if ((rs_alu.RegS1_Tag != 5'h0) && (cdb_packet.Tag == rs_alu.RegS1_Tag)) begin
                rs_alu.rs1_value <= cdb_packet.Value;
                rs_alu.ready[0]  <= 1;
            end
            if ((rs_alu.RegS2_Tag != 5'h0) && (cdb_packet.Tag == rs_alu.RegS2_Tag)) begin
                rs_alu.rs2_value <= cdb_packet.Value;
                rs_alu.ready[1]  <= 1;
            end
            if(cdb_packet.valid && (cdb_packet.Tag == rs_alu.Tag)) begin
                rs_alu.busy <= 0;
            end
        end else if (rs_ex_packet.PC == rs_alu.PC) begin
            rs_alu.ready[0]         <= 1'b0;
            rs_alu.ready[1]         <= 1'b0;
        end
    end

    logic enable_alu2;
    assign enable_alu2 = 1 && (!rs_alu2.busy) && dp_rs_packet.valid && !{dp_rs_packet.rd_mem, dp_rs_packet.wr_mem} && enable && (gnt[1] == 1'b1);
    RS rs_alu2;
    always_ff @( posedge clock ) begin
        if (reset) begin
            rs_alu2 <= 0;
        end 
        else if (cdb_packet.take_branch) begin
            rs_alu2 <= 0;
        end
        else if (enable_alu2) begin
            rs_alu2.inst             <= dp_rs_packet.inst;
            rs_alu2.NPC              <= dp_rs_packet.NPC;
            rs_alu2.PC               <= dp_rs_packet.PC;
            rs_alu2.busy             <= 1;

            rs_alu2.Tag              <= Tag;

            rs_alu2.RegS1_Tag        <= RS1_Tag;
            rs_alu2.RegS2_Tag        <= RS2_Tag;

            rs_alu2.ready[0]         <= ready[0];
            rs_alu2.ready[1]         <= ready[1];

            rs_alu2.rs1_value        <= rs1_value;
            rs_alu2.rs2_value        <= rs2_value;
            rs_alu2.opa_select       <= dp_rs_packet.opa_select;
            rs_alu2.opb_select       <= dp_rs_packet.opb_select;
            rs_alu2.dest_reg_idx     <= dp_rs_packet.dest_reg_idx;
            rs_alu2.alu_func         <= dp_rs_packet.alu_func;
            rs_alu2.rd_mem           <= dp_rs_packet.rd_mem;
            rs_alu2.wr_mem           <= dp_rs_packet.wr_mem;
            rs_alu2.cond_branch      <= dp_rs_packet.cond_branch;
            rs_alu2.uncond_branch    <= dp_rs_packet.uncond_branch;
            rs_alu2.halt             <= dp_rs_packet.halt;
            rs_alu2.illegal          <= dp_rs_packet.illegal;
            rs_alu2.csr_op           <= dp_rs_packet.csr_op;
            rs_alu2.valid            <= dp_rs_packet.valid;
            rs_alu2.func_unit        <= 1;
        end else if(cdb_packet.valid) begin
            if ((rs_alu2.RegS1_Tag != 5'h0) && (cdb_packet.Tag == rs_alu2.RegS1_Tag)) begin
                rs_alu2.rs1_value <= cdb_packet.Value;
                rs_alu2.ready[0]  <= 1;
            end
            if ((rs_alu2.RegS2_Tag != 5'h0) && (cdb_packet.Tag == rs_alu2.RegS2_Tag)) begin
                rs_alu2.rs2_value <= cdb_packet.Value;
                rs_alu2.ready[1]  <= 1;
            end
            if(cdb_packet.valid && (cdb_packet.Tag == rs_alu2.Tag)) begin
                rs_alu2.busy <= 0;
            end
        end else if (rs_ex_packet.PC == rs_alu2.PC) begin
            rs_alu2.ready[0]         <= 1'b0;
            rs_alu2.ready[1]         <= 1'b0;
        end
    end


    
    RS rs_mem;
    logic enable_mem;
    assign enable_mem = 1 && (!rs_mem.busy) && dp_rs_packet.valid && |{dp_rs_packet.rd_mem, dp_rs_packet.wr_mem} && enable;
    always_ff @( posedge clock ) begin
        if (reset) begin
            rs_mem <= 0;
        end
        else if (cdb_packet.take_branch) begin
            rs_mem <= 0;
        end  
        else if (enable_mem) begin
            rs_mem.inst             <= dp_rs_packet.inst;
            rs_mem.NPC              <= dp_rs_packet.NPC;
            rs_mem.PC               <= dp_rs_packet.PC;

            rs_mem.busy             <= 1;

            rs_mem.Tag              <= Tag;

            rs_mem.RegS1_Tag        <= RS1_Tag;
            rs_mem.RegS2_Tag        <= RS2_Tag;

            rs_mem.ready[0]         <= ready[0];
            rs_mem.ready[1]         <= ready[1];

            rs_mem.rs1_value        <= rs1_value;
            rs_mem.rs2_value        <= rs2_value;
            rs_mem.opa_select       <= dp_rs_packet.opa_select;
            rs_mem.opb_select       <= dp_rs_packet.opb_select;
            rs_mem.dest_reg_idx     <= dp_rs_packet.dest_reg_idx;
            rs_mem.alu_func         <= dp_rs_packet.alu_func;
            rs_mem.rd_mem           <= dp_rs_packet.rd_mem;
            rs_mem.wr_mem           <= dp_rs_packet.wr_mem;
            rs_mem.cond_branch      <= dp_rs_packet.cond_branch;
            rs_mem.uncond_branch    <= dp_rs_packet.uncond_branch;
            rs_mem.halt             <= dp_rs_packet.halt;
            rs_mem.illegal          <= dp_rs_packet.illegal;
            rs_mem.csr_op           <= dp_rs_packet.csr_op;
            rs_mem.valid            <= dp_rs_packet.valid;
            rs_mem.func_unit        <= 3;
        end else if(cdb_packet.valid) begin
            if ((rs_mem.RegS1_Tag != 5'h0) && (cdb_packet.Tag == rs_mem.RegS1_Tag)) begin
                rs_mem.rs1_value <= cdb_packet.Value;
                rs_mem.ready[0]  <= 1;
            end
            if ((rs_mem.RegS2_Tag != 5'h0) && (cdb_packet.Tag == rs_mem.RegS2_Tag)) begin
                rs_mem.rs2_value <= cdb_packet.Value;
                rs_mem.ready[1]  <= 1;
            end
            if(cdb_packet.valid && (cdb_packet.Tag == rs_mem.Tag)) begin
                rs_mem.busy <= 0;
            end
        end else if (rs_ex_packet.PC == rs_alu2.PC) begin
            rs_mem.ready[0]         <= 1'b0;
            rs_mem.ready[1]         <= 1'b0;
        end
    end


    //issue
    RS RS_issue;
    logic issue, issue_alu1, issue_alu2, issue_mem;
    assign issue_alu1 = rs_alu.ready[0] && rs_alu.ready[1];
    assign issue_alu2 = rs_alu2.ready[0] && rs_alu2.ready[1];
    assign issue_mem  = rs_mem.ready[0] && rs_mem.ready[1];
    assign issue = |{issue_alu1, issue_alu2, issue_mem};

    always_comb begin
        case (1'b1)
            issue_alu1: begin
                RS_issue = rs_alu;
            end
            issue_alu2: begin
                RS_issue = rs_alu2;
            end
            issue_mem: begin
                RS_issue = rs_mem;
            end
            default: RS_issue = 0;
        endcase
    end

    always_ff @( posedge clock ) begin
        if(reset) begin
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
    parameter WIDTH = 2        // 默认 8-bit 输入
) (
    input  logic  [WIDTH-1:0] req,
    output logic  [WIDTH-1:0] gnt // 自动计算编码位数
);

always_comb begin
    casex (req)
        2'b00: gnt = 00;
        2'b1x: gnt = 10;
        2'b01: gnt = 01;
        default: gnt = 00;
    endcase
end

endmodule