module ReservationStation (
    input   clock,
    input   reset,

    input   DP_RS_PACKET dp_rs_packet,
    input   enable,
    input   ROB_RS_PACKET rob_rs_packet,
    input   CDB_PACKET cdb_packet,
    // output rs_dp_packet rs_dp_packet,

    output RS_EX_PACKET rs_ex_packet,
    output logic busy[0:1]
);
    RS rs_alu;
    logic enable_alu;
    assign enable_alu = 1 && (!rs_alu.busy) && dp_rs_packet.valid && !{dp_rs_packet.rd_mem, dp_rs_packet.wr_mem} && enable;
    assign busy[0] = rs_alu.busy;
    assign busy[1] = 1;

    logic [`XLEN:0] rs1_value, rs2_value;
    logic [$clog2(`ROB_SIZE)-1:0]  Tag;
    logic [$clog2(`ROB_SIZE)-1:0]  RS1_Tag, RS2_Tag;
    logic ready[0:1];


    //rs1_value RS1_Tag rs2_value RS2_Tag Tag ready
    always_comb begin
        Tag = rob_rs_packet.Tag;

        if (rob_rs_packet.valid_vector[0]) begin
            if(rob_rs_packet.complete[0]) begin
                rs1_value = rob_rs_packet.rs1_value;
                ready[0] = 1;
            end else begin
                rs1_value = 0;
                RS1_Tag = rob_rs_packet.RegS1_Tag;
                ready[0] = 0;
            end
        end else begin
            rs1_value = dp_rs_packet.rs1_value;
            RS1_Tag = 0;
            ready[0] = 1;
        end

        if (rob_rs_packet.valid_vector[1]) begin
            if(rob_rs_packet.complete[1]) begin
                rs2_value = rob_rs_packet.rs2_value;
                ready[1] = 1;
            end else begin
                rs2_value = 0;
                RS2_Tag = rob_rs_packet.RegS2_Tag;
                ready[1] = 0;
            end
        end else begin
            rs2_value = dp_rs_packet.rs2_value;
            RS2_Tag = 0;
            ready[1] = 1;
        end
    end

    always_ff @( posedge clock ) begin
        if (reset) begin
            rs_alu <= 0;
        end else if (enable_alu) begin
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
        end
    end

    //issue
    logic issue;
    assign issue = rs_alu.ready[0] && rs_alu.ready[1];
    always_ff @( posedge clock ) begin
        if(reset) begin
            rs_ex_packet <= 0;
        end else if (issue) begin
            rs_ex_packet.inst           <= rs_alu.inst;
            rs_ex_packet.PC             <= rs_alu.PC;
            rs_ex_packet.NPC            <= rs_alu.NPC; 

            rs_ex_packet.rs1_value      <= rs_alu.rs1_value; 
            rs_ex_packet.rs2_value      <= rs_alu.rs2_value; 

            rs_ex_packet.Tag            <= rs_alu.Tag;

            rs_ex_packet.opa_select     <= rs_alu.opa_select; 
            rs_ex_packet.opb_select     <= rs_alu.opb_select;
            rs_ex_packet.dest_reg_idx   <= rs_alu.dest_reg_idx; 
            rs_ex_packet.alu_func       <= rs_alu.alu_func;     
            rs_ex_packet.rd_mem         <= rs_alu.rd_mem;       
            rs_ex_packet.wr_mem         <= rs_alu.wr_mem;        
            rs_ex_packet.cond_branch    <= rs_alu.cond_branch;  
            rs_ex_packet.uncond_branch  <= rs_alu.uncond_branch; 
            rs_ex_packet.halt           <= rs_alu.halt;          
            rs_ex_packet.illegal        <= rs_alu.illegal;      
            rs_ex_packet.csr_op         <= rs_alu.csr_op;       
            rs_ex_packet.valid          <= rs_alu.valid;
        end else begin
            rs_ex_packet.valid          <= 0;
        end
    end

    RS rs_mem;
    logic enable_mem;
    assign enable_mem = 1 && (!rs_mem.busy) && dp_rs_packet.valid && |{dp_rs_packet.rd_mem, dp_rs_packet.wr_mem};
    always_ff @( posedge clock ) begin
        if (reset) begin
            rs_mem <= 0;
        end else if (0) begin
            rs_mem.inst             <= dp_rs_packet.inst;
            rs_mem.NPC              <= dp_rs_packet.NPC;
            rs_mem.PC               <= dp_rs_packet.PC;

            rs_mem.busy             <= 1;


            rs_mem.rs1_value        <= dp_rs_packet.rs1_value;
            rs_mem.rs2_value        <= dp_rs_packet.rs2_value;
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
        end
    end


    //issue


endmodule

/*
typedef struct packed {
    INST inst;                 // instruction
	logic [`XLEN-1:0] NPC;     // PC + 4
	logic [`XLEN-1:0] PC;      // PC                                 

    logic busy;
    logic [$clog2(`ROB_SIZE)-1:0] Tag; 
    logic [$clog2(`ROB_SIZE)-1:0] rs1_tag;
    logic [$clog2(`ROB_SIZE)-1:0] rs2_tag;
	logic [`XLEN-1:0] rs1_value;    // reg A value                                  
	logic [`XLEN-1:0] rs2_value;    // reg B value   

	ALU_OPA_SELECT opa_select; // ALU opa mux select (ALU_OPA_xxx *)
	ALU_OPB_SELECT opb_select; // ALU opb mux select (ALU_OPB_xxx *)
	
	logic [4:0] dest_reg_idx;  // destination (writeback) register index      
	ALU_FUNC    alu_func;      // ALU function select (ALU_xxx *)
	logic       rd_mem;        // does inst read memory?
	logic       wr_mem;        // does inst write memory?
	logic       cond_branch;   // is inst a conditional branch?
	logic       uncond_branch; // is inst an unconditional branch?
	logic       halt;          // is this a halt?
	logic       illegal;       // is this instruction illegal?
	logic       csr_op;        // is this a CSR operation? (we only used this as a cheap way to get return code)
	logic       valid;         // is inst a valid instruction to be counted for CPI calculations?

	FUNC_UNIT   func_unit;     // function unit
	// logic [$clog2(`SQ_SIZE)-1:0] tail_pos;
} RS;

typedef enum logic [1:0] {
	FUNC_NOP    = 2'h0,    // no instruction free, DO NOT USE THIS AS DEFAULT CASE!
	FUNC_ALU    = 2'h1,    // all of the instruction  except mult and load and store
	FUNC_MULT   = 2'h2,    // mult 
	FUNC_MEM    = 2'h3     // load and store
}FUNC_UNIT;
*/