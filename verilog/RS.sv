module ReservationStation (
    input                   clock,
    input                   reset,

    input   DP_RS_PACKET    dp_rs_packet,
    input                   enable,
    input   ROB_RS_PACKET   rob_rs_packet,
    input logic [$clog2(`LSQ_SIZE)-1:0]   lsq_idx,
    input   CDB_PACKET      cdb_packet,
    // output rs_dp_packet rs_dp_packet,
    input                   take_branch,
    input   CDB_PACKET      lsq_input,

    output RS_EX_PACKET     rs_ex_packet,
    output logic            read_enable,
    output [11:0]            busy
);

    ////////////////////////////////////////////////
    //Get the Correct ready and rs_value from ROB MT DP
    //
    ///////////////////////////////////////////////
    logic [`XLEN-1:0] rs1_value, rs2_value;
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


    ///////////////////////////////////////////////////
    // Control for reading from Instruction Buffer
    //
    ///////////////////////////////////////////////////
    logic req_valid, req_valid_mem;
    assign read_enable = req_valid && req_valid_mem;
    // logic prev_read_enable;
    // wire rs_available = (req_valid || 0) && !prev_read_enable;//todo to fast maybe gg
    // // 时序逻辑：生成 read_enable
    // always @(posedge clock) begin
    //     if (reset) begin
    //         read_enable      <= 1'b0;
    //         prev_read_enable <= 1'b0;
    //     end else begin
    //         // 生成单周期脉冲
    //         read_enable <= rs_available;
    //         // 记录上一周期的读操作状态
    //         prev_read_enable <= read_enable;
    //     end
    // end

    // ================================================================
    // gnt
    // ================================================================
    logic [7:0] gnt_rs;
    wire  [7:0] req = {!RS_Alu[7].busy,!RS_Alu[6].busy,!RS_Alu[5].busy,!RS_Alu[4].busy,!RS_Alu[3].busy,!RS_Alu[2].busy,!RS_Alu[1].busy,!RS_Alu[0].busy};
    priority_encoder #(
        .WIDTH(8)
    )u_priority_encoder(
        .req(req),
        .gnt(gnt_rs),
        .valid(req_valid)
    );


    ////////////////////////////////////////////////////////////////////
    //  RS
    //
    ////////////////////////////////////////////////////////////////////
    //assign busy[2] = RS_Mem.busy;todo

    RS RS_Alu[0:7];
    logic [7:0] enable_alu;
    logic [11:0] gnt;
    logic issue;
    logic [7:0] issue_alu;
    // assign issue_mem  = RS_Mem.ready[0] && RS_Mem.ready[1];todo
    // ================================================================
    // 生成 8 个保留站实例
    // ================================================================
    genvar i;
    generate
      for (i = 0; i < 8; i = i + 1) begin : gen_reservation_stations
        assign enable_alu[i] = 1 && (!RS_Alu[i].busy) && dp_rs_packet.valid && !{dp_rs_packet.rd_mem, dp_rs_packet.wr_mem} && enable && (gnt_rs[i] == 1'b1);
        assign busy[i] = RS_Alu[i].busy;
        assign issue_alu[i] = RS_Alu[i].ready[0] && RS_Alu[i].ready[1];
        ResSta u_RS (
            .clock       (clock),
            .reset       (reset),

            // 冲刷信号（共享）
            .flush       (take_branch),
            //.flush       (1'b0),
            // 使能信号按索引分配（例如 enable_alu[0] ~ enable_alu[7]）
            .enable      (enable_alu[i]),

            // 输入数据包（假设广播共享）
            .dp_rs_packet(dp_rs_packet),

            // Tag 信号按索引分配（假设 Tag 是向量）
            .RS1_Tag     (RS1_Tag),
            .RS2_Tag     (RS2_Tag),
            .Tag         (Tag),
            .needTag     (needTag),

            // 操作数值（按索引分配）
            .rs1_value   (rs1_value),
            .rs2_value   (rs2_value),

            // 就绪信号（按索引输出）
            .ready       (ready),

            // CDB 广播（共享）
            .cdb_packet  (cdb_packet),
            .lsq_input   (lsq_input),

            // 仲裁授权信号（按索引分配）
            .issue       (gnt[i]),  // 例如 gnt_rs[0] ~ gnt_rs[7]

            // 输出到执行单元（按索引分配）
            .rs          (RS_Alu[i])   // 例如 RS_Alu[0] ~ RS_Alu[7]
        );
      end
    endgenerate

    logic [3:0] gnt_mem;
    wire  [3:0] req_mem = {!RS_Mem[3].busy,!RS_Mem[2].busy,!RS_Mem[1].busy,!RS_Mem[0].busy};
    priority_encoder #(
        .WIDTH(4)
    )u_priority_encoder_mem(
        .req(req_mem),
        .gnt(gnt_mem),
        .valid(req_valid_mem)
    );

    RS RS_Mem[0:3];
    logic [3:0] enable_mem;
    logic [3:0] issue_mem;
    genvar f;
    generate
      for (f = 0; f < 4; f = f + 1) begin
        assign enable_mem[f]    = 1 && (!RS_Mem[f].busy) && dp_rs_packet.valid && |{dp_rs_packet.rd_mem, dp_rs_packet.wr_mem} && enable && (gnt_mem[f] == 1'b1);
        assign busy[f+8]        = RS_Mem[f].busy;
        assign issue_mem[f]     = RS_Mem[f].ready[0] && RS_Mem[f].ready[1];
        ResSta_MEM u_RS_MEM (
            .clock       (clock),
            .reset       (reset),

            // 冲刷信号（共享）
            .flush       (take_branch),
            //.flush       (1'b0),
            // 使能信号按索引分配（例如 enable_alu[0] ~ enable_alu[7]）
            .enable      (enable_mem[f]),

            // 输入数据包（假设广播共享）
            .dp_rs_packet(dp_rs_packet),

            // Tag 信号按索引分配（假设 Tag 是向量）
            .RS1_Tag     (RS1_Tag),
            .RS2_Tag     (RS2_Tag),
            .Tag         (Tag),
            .needTag     (needTag),

            .lsq_idx     (lsq_idx),

            // 操作数值（按索引分配）
            .rs1_value   (rs1_value),
            .rs2_value   (rs2_value),

            // 就绪信号（按索引输出）
            .ready       (ready),

            // CDB 广播（共享）
            .cdb_packet  (cdb_packet),
            .lsq_input   (lsq_input),

            // 仲裁授权信号（按索引分配）
            .issue       (gnt[8 + f]),  // 例如 gnt_rs[0] ~ gnt_rs[7]

            // 输出到执行单元（按索引分配）
            .rs          (RS_Mem[f])   // 例如 RS_Alu[0] ~ RS_Alu[7]
        );
      end
    endgenerate

    /////////////////////////////////////////////////////
    // Issue when RS get ready
    //
    /////////////////////////////////////////////////////
    RS RS_issue;
    // assign issue = |{issue_alu1, issue_alu2, issue_mem};

    priority_encoder #(
        .WIDTH(12)
    )u_priority_encoder_rs(
        .req({issue_mem,issue_alu}),
        .gnt(gnt),
        .valid(issue)
    );

    always_comb begin
        RS_issue = 0;
        for (int m = 0;m < 8 ; m++) begin
            if (gnt[m]) begin
                RS_issue = RS_Alu[m];
            end
        end
        for (int k = 0;k < 4 ; k++) begin
            if (gnt[k+8]) begin
                RS_issue = RS_Mem[k];
            end
        end
        // case (gnt)
        //     9'b100000000: begin
        //         RS_issue = RS_Mem;
        //     end
        //     9'b010000000: begin
        //         RS_issue = RS_Alu[7];
        //     end
        //     9'b001000000: begin
        //         RS_issue = RS_Alu[6];
        //     end
        //     9'b000100000: begin
        //         RS_issue = RS_Alu[5];
        //     end
        //     9'b000010000: begin
        //         RS_issue = RS_Alu[4];
        //     end
        //     9'b000001000: begin
        //         RS_issue = RS_Alu[3];
        //     end
        //     9'b000000100: begin
        //         RS_issue = RS_Alu[2];
        //     end
        //     9'b000000010: begin
        //         RS_issue = RS_Alu[1];
        //     end
        //     9'b000000001: begin
        //         RS_issue = RS_Alu[0];
        //     end
        //     default: RS_issue = 0;
        // endcase
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
            rs_ex_packet.lsq_idx        <= RS_issue.lsq_idx;
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
    input CDB_PACKET      lsq_input,
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
            if(cdb_packet.valid && (cdb_packet.Tag == rs.Tag) && (rs.PC == cdb_packet.PC)) begin
                rs.busy <= 0;
            end
        end

        if(lsq_input.valid) begin
            if ((lsq_input.Tag == rs.RegS1_Tag) && (lsq_input.dest_reg_idx != `ZERO_REG) && !rs.received[0] && !rs.ready[0] && rs.needTag[0]) begin
                rs.rs1_value <= lsq_input.Value;
                rs.ready[0]  <= 1;
                rs.received[0] <= 1;
            end
            if ((lsq_input.Tag == rs.RegS2_Tag) && (lsq_input.dest_reg_idx != `ZERO_REG) && !rs.received[1] && !rs.ready[1] && rs.needTag[1]) begin
                rs.rs2_value <= lsq_input.Value;
                rs.ready[1]  <= 1;
                rs.received[1] <= 1;
            end
            if(lsq_input.valid && (lsq_input.Tag == rs.Tag) && (rs.PC == lsq_input.PC)) begin
                rs.busy <= 0;
            end
        end
        
        if ((issue)) begin//todo
            rs.ready[0]         <= 1'b0;
            rs.ready[1]         <= 1'b0;
        end
    end
endmodule



module ResSta_MEM #(
    
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
    input logic [$clog2(`LSQ_SIZE)-1:0]   lsq_idx,
    //deassert rs
    input                           ready[0:1],
    input CDB_PACKET                cdb_packet,
    input CDB_PACKET      lsq_input,
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
            rs.lsq_idx          <= lsq_idx;

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
            if(cdb_packet.valid && (cdb_packet.Tag == rs.Tag) && (rs.PC == cdb_packet.PC)) begin
                rs.busy <= 0;
            end
        end

        if(lsq_input.valid) begin
            if ((lsq_input.Tag == rs.RegS1_Tag) && (lsq_input.dest_reg_idx != `ZERO_REG) && !rs.received[0] && !rs.ready[0] && rs.needTag[0]) begin
                rs.rs1_value <= lsq_input.Value;
                rs.ready[0]  <= 1;
                rs.received[0] <= 1;
            end
            if ((lsq_input.Tag == rs.RegS2_Tag) && (lsq_input.dest_reg_idx != `ZERO_REG) && !rs.received[1] && !rs.ready[1] && rs.needTag[1]) begin
                rs.rs2_value <= lsq_input.Value;
                rs.ready[1]  <= 1;
                rs.received[1] <= 1;
            end
            if(lsq_input.valid && (lsq_input.Tag == rs.Tag) && (rs.PC == lsq_input.PC)) begin
                rs.busy <= 0;
            end
        end
        
        if ((issue)) begin//todo
            rs.ready[0]         <= 1'b0;
            rs.ready[1]         <= 1'b0;
            //rs.busy             <= rs.rd_mem;
        end
    end
endmodule