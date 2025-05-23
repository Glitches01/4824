// The decoder, copied from p3/stage_id.sv without changes

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"


module Dispatch (
    input clock,
    input reset,
    input enable,

    // From Inst Buffer
    input  IB_ID_PACKET ib_id_packet,

    input ROB_MT_PACKET rob_mt_packet,

    input logic take_branch,
    //output ID_IB_PACKET id_ib_packet,

    input CP_RT_PACKET    cp_rt_packet,   
    
    // To RS
    output DP_RS_PACKET  dp_rs_packet,
    output DP_LSQ_PACKET dp_lsq_packet,

    output DP_ROB_PACKET dp_rob_packet,

    output MT_ROB_PACKET mt_rob_packet,

    input logic             wb_regfile_en,  // register write enable
    input logic [4:0]       wb_regfile_idx, // register write index
    input logic [`XLEN-1:0] wb_regfile_data // register write data

);

    always_comb begin
        dp_lsq_packet.inst          = dp_rs_packet.inst;
        dp_lsq_packet.PC            = dp_rs_packet.PC;
        dp_lsq_packet.NPC           = dp_rs_packet.NPC;
        dp_lsq_packet.dest_reg_idx  = dp_rs_packet.dest_reg_idx;
        dp_lsq_packet.rd_mem        = dp_rs_packet.rd_mem;
        dp_lsq_packet.wr_mem        = dp_rs_packet.wr_mem;
        dp_lsq_packet.Tag           = rob_mt_packet.Tag;
        dp_lsq_packet.rd_unsigned   = ib_id_packet.inst.r.funct3[2];
    end
/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Instance Name :  u_regfile                                         //
//                                                                     //
//  Description :  This module is the Architectural Regfiles, receive  //
//                  Input from rt_packet: retirement units and store   //
//                  precise state regfiles and read it into id_packet  //
//                                                                     //
/////////////////////////////////////////////////////////////////////////
    regfile u_regfile(
        .clock          (clock)     ,
        .reset          (reset),
        .read_idx_1     (ib_id_packet.inst.r.rs1),
        .read_idx_2     (ib_id_packet.inst.r.rs2),
        .write_idx      (wb_regfile_idx) ,
        .write_en       (wb_regfile_en)  ,
        .write_data     (wb_regfile_data),
        .read_out_1     (dp_rs_packet.rs1_value),
        .read_out_2     (dp_rs_packet.rs2_value)
    );

    logic has_dest_reg;
    decoder u_decoder0 (
        // Inputs
        .inst  (ib_id_packet.inst),
        .valid (ib_id_packet.valid),

        // Outputs
        .opa_select    (dp_rs_packet.opa_select),
        .opb_select    (dp_rs_packet.opb_select),
        .alu_func      (dp_rs_packet.alu_func),
        .has_dest      (has_dest_reg),
        .rd_mem        (dp_rs_packet.rd_mem),
        .wr_mem        (dp_rs_packet.wr_mem),
        .cond_branch   (dp_rs_packet.cond_branch),
        .uncond_branch (dp_rs_packet.uncond_branch),
        .csr_op        (dp_rs_packet.csr_op),
        .halt          (dp_rs_packet.halt),
        .illegal       (dp_rs_packet.illegal)
    );


/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Instance Name :  Other Logic                                       //
//                                                                     //
//  Description :  Other Logic Bypassing                               //
//                                                                     //
/////////////////////////////////////////////////////////////////////////
    assign dp_rs_packet.inst = ib_id_packet.inst;
    assign dp_rs_packet.PC   = ib_id_packet.PC;
    assign dp_rs_packet.NPC  = ib_id_packet.NPC;

    // For counting valid instructions executed
    // and making the fetch stage die on halts/keeping track of when
    // to allow the next instruction out of fetch
    // 0 for HALT and illegal instructions (end processor on halt)
    assign dp_rs_packet.valid = ib_id_packet.valid & ~dp_rs_packet.illegal;
    assign dp_rs_packet.dest_reg_idx = (has_dest_reg) ? ib_id_packet.inst.r.rd : `ZERO_REG;
    assign dp_rs_packet.mem = |{dp_rs_packet.rd_mem, dp_rs_packet.wr_mem};



/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Instance Name :  For ROB                                           //
//                                                                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////
    assign dp_rob_packet.dest_reg_idx   = dp_rs_packet.dest_reg_idx;
    assign dp_rob_packet.PC             = dp_rs_packet.PC;
    assign dp_rob_packet.NPC            = dp_rs_packet.NPC;
    assign dp_rob_packet.IsBranch       = (dp_rs_packet.cond_branch || dp_rs_packet.uncond_branch);
    assign dp_rob_packet.mem            = dp_rs_packet.mem;
    assign dp_rob_packet.wr_mem         = dp_rs_packet.wr_mem;

    MAPTABLE rob_entry1;
    MAPTABLE rob_entry2;
    assign mt_rob_packet.RegS1_Tag = rob_entry1.rob_entry;
    assign mt_rob_packet.RegS2_Tag = rob_entry2.rob_entry;
    assign mt_rob_packet.valid_vector[0] = rob_entry1.valid;
    assign mt_rob_packet.valid_vector[1] = rob_entry2.valid;

    logic [4:0] mt_read_idx_1, mt_read_idx_2;

    assign mt_read_idx_1 = ib_id_packet.inst.r.rs1;
    assign mt_read_idx_2 = ib_id_packet.inst.r.rs2;
    MapTable u_MapTable(
        .clock              (clock),
        .reset              (reset),

        .enable             (enable),
        .squash             (take_branch),

        .read_idx_1         (mt_read_idx_1),
        .read_idx_2         (mt_read_idx_2),

        .rob_entry1         (rob_entry1),
        .rob_entry2         (rob_entry2),

        .has_dest_reg       (has_dest_reg),
        .dest_reg_idx       (dp_rs_packet.dest_reg_idx),
        .PC                 (dp_rs_packet.PC),

        .rob_tail           (rob_mt_packet.Tag),

        .cp_rt_packet       (cp_rt_packet)
    );
    
endmodule


module MapTable #(
    parameter ROB_SIZE = 32
) (
    input clock,
    input reset,

    input enable,
    input squash,

    input [4:0] read_idx_1,
    input [4:0] read_idx_2,

    output MAPTABLE rob_entry1,
    output MAPTABLE rob_entry2,

    input       has_dest_reg,
    input [4:0] dest_reg_idx,
    input logic [`XLEN-1:0]           PC,

    input  [$clog2(`ROB_SIZE)-1:0] rob_tail,

    input CP_RT_PACKET    cp_rt_packet  
);
    MAPTABLE maptable[0:31];
    assign rob_entry1 = maptable[read_idx_1];
    assign rob_entry2 = maptable[read_idx_2];

    always_ff @(posedge clock) begin : blockName
        if(reset) begin
            for (integer i = 0; i < 32; i = i + 1) begin
                maptable[i] <= 0;
            end
        end else if (squash) begin
            for (integer i = 0; i < 32; i = i + 1) begin
                maptable[i] <= 0;
            end
        end else begin
            
            if(has_dest_reg && enable) begin
                maptable[dest_reg_idx].rob_entry <= rob_tail;
                maptable[dest_reg_idx].valid <= 1;
            end

            if (cp_rt_packet.rob_entry.cp_bit && (cp_rt_packet.Tag == maptable[cp_rt_packet.rob_entry.reg_idx].rob_entry)) begin
                if (has_dest_reg && enable && (dest_reg_idx == cp_rt_packet.rob_entry.reg_idx)) begin
                    maptable[dest_reg_idx].rob_entry <= rob_tail;
                    maptable[dest_reg_idx].valid     <= 1;
                end else begin
                    maptable[cp_rt_packet.rob_entry.reg_idx].valid <= 1'b0;
                end
            end
        end
    end
endmodule


// Decode an instruction: generate useful datapath control signals by matching the RISC-V ISA
// This module is purely combinational
module decoder (
    input INST  inst,
    input logic valid, // when low, ignore inst. Output will look like a NOP

    output ALU_OPA_SELECT opa_select,
    output ALU_OPB_SELECT opb_select,
    output logic          has_dest, // if there is a destination register
    output ALU_FUNC       alu_func,
    output logic          rd_mem, wr_mem, cond_branch, uncond_branch,
    output logic          csr_op, // used for CSR operations, we only use this as a cheap way to get the return code out
    output logic          halt,   // non-zero on a halt
    output logic          illegal // non-zero on an illegal instruction
);

    // Note: I recommend using an IDE's code folding feature on this block
    always_comb begin
    // Default control values (looks like a NOP)
    // See sys_defs.svh for the constants used here
    /////////////////////////////////////////////////////////////////////////////////////
    // Notes:
    // 1. Ref has functor_out = FUNC_ALU; logic FUNC_UNIT functor_out;
    //          This is basically distinguish btw FUNCTIONS, Necessary???
    // 2. rs1_exist  = `TRUE; rs2_exist = `FALSE; Necessray???
    /////////////////////////////////////////////////////////////////////////////////////

        opa_select    = OPA_IS_RS1;
        opb_select    = OPB_IS_RS2;
        alu_func      = ALU_ADD;
        has_dest      = `FALSE;
        csr_op        = `FALSE;
        rd_mem        = `FALSE;
        wr_mem        = `FALSE;
        cond_branch   = `FALSE;
        uncond_branch = `FALSE;
        halt          = `FALSE;
        illegal       = `FALSE;

        if (valid) begin
            casez (inst)
                `RV32_LUI: begin
                    has_dest   = `TRUE;
                    opa_select = OPA_IS_ZERO;
                    opb_select = OPB_IS_U_IMM;
                end
                `RV32_AUIPC: begin
                    has_dest   = `TRUE;
                    opa_select = OPA_IS_PC;
                    opb_select = OPB_IS_U_IMM;
                end
                `RV32_JAL: begin
                    has_dest      = `TRUE;
                    opa_select    = OPA_IS_PC;
                    opb_select    = OPB_IS_J_IMM;
                    uncond_branch = `TRUE;
                end
                `RV32_JALR: begin
                    has_dest      = `TRUE;
                    opa_select    = OPA_IS_RS1;
                    opb_select    = OPB_IS_I_IMM;
                    uncond_branch = `TRUE;
                end
                `RV32_BEQ, `RV32_BNE, `RV32_BLT, `RV32_BGE,
                `RV32_BLTU, `RV32_BGEU: begin
                    opa_select  = OPA_IS_PC;
                    opb_select  = OPB_IS_B_IMM;
                    cond_branch = `TRUE;
                end
                `RV32_LB, `RV32_LH, `RV32_LW,
                `RV32_LBU, `RV32_LHU: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    rd_mem     = `TRUE;
                end
                `RV32_SB, `RV32_SH, `RV32_SW: begin
                    opb_select = OPB_IS_S_IMM;
                    wr_mem     = `TRUE;
                end
                `RV32_ADDI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                end
                `RV32_SLTI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_SLT;
                end
                `RV32_SLTIU: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_SLTU;
                end
                `RV32_ANDI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_AND;
                end
                `RV32_ORI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_OR;
                end
                `RV32_XORI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_XOR;
                end
                `RV32_SLLI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_SLL;
                end
                `RV32_SRLI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_SRL;
                end
                `RV32_SRAI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_SRA;
                end
                `RV32_ADD: begin
                    has_dest   = `TRUE;
                end
                `RV32_SUB: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SUB;
                end
                `RV32_SLT: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SLT;
                end
                `RV32_SLTU: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SLTU;
                end
                `RV32_AND: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_AND;
                end
                `RV32_OR: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_OR;
                end
                `RV32_XOR: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_XOR;
                end
                `RV32_SLL: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SLL;
                end
                `RV32_SRL: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SRL;
                end
                `RV32_SRA: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SRA;
                end
                `RV32_MUL: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_MUL;
                end
                `RV32_MULH: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_MULH;
                end
                `RV32_MULHSU: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_MULHSU;
                end
                `RV32_MULHU: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_MULHU;
                end
                `RV32_CSRRW, `RV32_CSRRS, `RV32_CSRRC: begin
                    csr_op = `TRUE;
                end
                `WFI: begin
                    halt = `TRUE;
                end
                default: begin
                    illegal = `TRUE;
                end
        endcase // casez (inst)
        end // if (valid)
    end // always

endmodule // decoder

/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  regfile.sv                                          //
//                                                                     //
//  Description :  This module creates the Regfile used by the ID and  //
//                 WB Stages of the Pipeline.                          //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

module regfile (
    input             clock, // system clock
    input             reset,
    // note: no system reset, register values must be written before they can be read
    input [4:0]       read_idx_1, read_idx_2, write_idx,
    input             write_en,
    input [`XLEN-1:0] write_data,

    output logic [`XLEN-1:0] read_out_1, read_out_2
);

    logic [31:1] [`XLEN-1:0] registers; // 31 XLEN-length Registers (0 is known)

    // Read port 1
    always_comb begin
        if (read_idx_1 == `ZERO_REG) begin
            read_out_1 = 0;
        end else if (write_en && (write_idx == read_idx_1)) begin
            read_out_1 = write_data; // internal forwarding
        end else begin
            read_out_1 = registers[read_idx_1];
        end
    end

    // Read port 2
    always_comb begin
        if (read_idx_2 == `ZERO_REG) begin
            read_out_2 = 0;
        end else if (write_en && (write_idx == read_idx_2)) begin
            read_out_2 = write_data; // internal forwarding
        end else begin
            read_out_2 = registers[read_idx_2];
        end
    end

    // Write port
    always_ff @(posedge clock) begin
        if(reset) begin
            for (integer i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'h0;
            end
        end else if (write_en && write_idx != `ZERO_REG) begin
            registers[write_idx] <= write_data;
        end
    end

endmodule // regfile

