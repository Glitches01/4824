
// The decoder, copied from p3/stage_id.sv without changes

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"


module Stage_ID (
    input clock,
    input reset,

    //From Retirement Units
    input RT_PACKET rt_packet,

    //From Fetch
    input IF_ID_PACKET if_id_reg,
    
    ///////////////////////////////////////////////////////////////////////////////////////
    //      From LSA, RS, RoB to determine the dp_packets that could be sent in this cycle
    //      Send to Instruciton Buffer
    //
    // input LSQ, RS, RoB,
	// output logic [1:0] dp_packet_count_out,
    ///////////////////////////////////////////////////////////////////////////////////////

    //To RAT
    output ID_RAT_PACKET id_rat_packet
);


/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Instance Name :  u_regfile                                         //
//                                                                     //
//  Description :  This module is the Architectural Regfiles, receive  //
//                  Input from rt_packet: retirement units and store   //
//                  precise state regfiles and read it into id_packet  //
//                                                                     //
/////////////////////////////////////////////////////////////////////////
    // regfile u_regfile(
    //     .clock          (clock)     ,
    //     .read_idx_1     (read_idx_1),
    //     .read_idx_2     (read_idx_2),
    //     .write_idx      (write_idx) ,
    //     .write_en       (write_en)  ,
    //     .write_data     (write_data),
    //     .read_out_1     (read_out_1),
    //     .read_out_2     (read_out_2)
    // );
    regfile u_regfile(
        .clock          (clock)     ,
        .read_idx_1     (if_id_reg.inst.r.rs1),
        .read_idx_2     (if_id_reg.inst.r.rs2),

        .write_idx      (rt_packet.write_idx) ,
        .write_en       (rt_packet.write_en)  ,
        .write_data     (rt_packet.write_data),

        .read_out_1     (id_rat_packet.rs1_value),
        .read_out_2     (id_rat_packet.rs2_value)
    );

/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Instance Name :  u_regfile                                         //
//                                                                     //
//  Description :  This module is the Architectural Regfiles, receive  //
//                  Input from rt_packet: retirement units and store   //
//                  precise state regfiles and read it into id_packet  //
//                                                                     //
/////////////////////////////////////////////////////////////////////////
    decoder u_decoder (
        // Inputs
        .inst  (if_id_reg.inst),
        .valid (if_id_reg.valid),

        // Outputs
        .opa_select    (id_rat_packet.opa_select),
        .opb_select    (id_rat_packet.opb_select),
        .alu_func      (id_rat_packet.alu_func),
        .has_dest      (has_dest_reg),
        .rd_mem        (id_rat_packet.rd_mem),
        .wr_mem        (id_rat_packet.wr_mem),
        .cond_branch   (id_rat_packet.cond_branch),
        .uncond_branch (id_rat_packet.uncond_branch),
        .csr_op        (id_rat_packet.csr_op),
        .halt          (id_rat_packet.halt),
        .illegal       (id_rat_packet.illegal)
    );

/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Instance Name :  Other Logic                                       //
//                                                                     //
//  Description :  Other Logic Bypassing                               //
//                                                                     //
/////////////////////////////////////////////////////////////////////////
    assign id_rat_packet.inst = if_id_reg.inst;
    assign id_rat_packet.PC   = if_id_reg.PC;
    assign id_rat_packet.NPC  = if_id_reg.NPC;

    // For counting valid instructions executed
    // and making the fetch stage die on halts/keeping track of when
    // to allow the next instruction out of fetch
    // 0 for HALT and illegal instructions (end processor on halt)
    assign id_rat_packet.valid = if_id_reg.valid & ~id_rat_packet.illegal;

    logic has_dest_reg;
    assign id_rat_packet.dest_reg_idx = (has_dest_reg) ? if_id_reg.inst.r.rd : `ZERO_REG;
    
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
    //
    //
    //
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

`include "verilog/sys_defs.svh"

module regfile (
    input             clock, // system clock
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
        if (write_en && write_idx != `ZERO_REG) begin
            registers[write_idx] <= write_data;
        end
    end

endmodule // regfile

