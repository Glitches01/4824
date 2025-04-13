module ReservationStation (
    input   clock,
    input   reset,

    input  DP_RS_PACKET dp_rs_packet
    // output rs_dp_packet rs_dp_packet,

    // output rs_ex_packet rs_ex_packet
);
    RS rs_alu;
    logic enable_alu;
    assign enable_alu = 1 && (!rs_alu.busy) && dp_rs_packet.valid;
    always_ff @( posedge clock ) begin : blockName
        if (reset) begin
            rs_alu <= 0;
        end else if (enable_alu) begin
            rs_alu.inst             <= dp_rs_packet.inst;
            rs_alu.NPC              <= dp_rs_packet.NPC;
            rs_alu.PC               <= dp_rs_packet.PC;
            rs_alu.busy             <= 1;
            rs_alu.Tag              <= 2;
            rs_alu.rs1_tag          <= 3;
            rs_alu.rs2_tag          <= 4;
            rs_alu.rs1_value        <= dp_rs_packet.rs1_value;
            rs_alu.rs2_value        <= dp_rs_packet.rs2_value;
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
        end
    end



    
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