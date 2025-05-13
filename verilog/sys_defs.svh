/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  sys_defs.svh                                        //
//                                                                     //
//  Description :  This file has the macro-defines for macros used in  //
//                 the pipeline design.                                //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`ifndef __SYS_DEFS_SVH__
`define __SYS_DEFS_SVH__

// all files should `include "sys_defs.svh" to at least define the timescale
`timescale 1ns/100ps

///////////////////////////////////
// ---- Starting Parameters ---- //
///////////////////////////////////

// some starting parameters that you should set
// this is *your* processor, you decide these values (try analyzing which is best!)

// superscalar width
`define N 1
`define MULT_STAGES 2
// sizes
`define ROB_SZ xx
`define RS_SZ xx
`define PHYS_REG_SZ (32 + `ROB_SZ)

// worry about these later
`define BRANCH_PRED_SZ xx
`define LSQ_SZ xx

// functional units (you should decide if you want more or fewer types of FUs)
`define NUM_FU_ALU xx
`define NUM_FU_MULT xx
`define NUM_FU_LOAD xx
`define NUM_FU_STORE xx

///////////////////////////////
// ---- Basic Constants ---- //
///////////////////////////////

// NOTE: the global CLOCK_PERIOD is defined in the Makefile

// useful boolean single-bit definitions
`define FALSE 1'h0
`define TRUE  1'h1

// data length
`define XLEN 32

// the zero register
// In RISC-V, any read of this register returns zero and any writes are thrown away
`define ZERO_REG 5'd0

// Basic NOP instruction. Allows pipline registers to clearly be reset with
// an instruction that does nothing instead of Zero which is really an ADDI x0, x0, 0
`define NOP 32'h00000013

//////////////////////////////////
// ---- Memory Definitions ---- //
//////////////////////////////////

// Cache mode removes the byte-level interface from memory, so it always returns
// a double word. The original processor won't work with this defined. Your new
// processor will have to account for this effect on mem.
// Notably, you can no longer write data without first reading.
`define CACHE_MODE

// you are not allowed to change this definition for your final processor
// the project 3 processor has a massive boost in performance just from having no mem latency
// see if you can beat it's CPI in project 4 even with a 100ns latency!
//`define MEM_LATENCY_IN_CYCLES  0
`define SYNTH_CLOCK_PERIOD     10
 `define MEM_LATENCY_IN_CYCLES (100.0/`SYNTH_CLOCK_PERIOD+0.49999)//Imp: Set to Zero to Make P3 Run
// the 0.49999 is to force ceiling(100/period). The default behavior for
// float to integer conversion is rounding to nearest

// How many memory requests can be waiting at once
`define NUM_MEM_TAGS 15

`define MEM_SIZE_IN_BYTES (64*1024)
// `define MEM_SIZE_IN_BYTES (64*1)
`define MEM_64BIT_LINES   (`MEM_SIZE_IN_BYTES/8)

typedef union packed {
    logic [7:0][7:0]  byte_level;
    logic [3:0][15:0] half_level;
    logic [1:0][31:0] word_level;
} EXAMPLE_CACHE_BLOCK;

typedef enum logic [1:0] {
    BYTE   = 2'h0,
    HALF   = 2'h1,
    WORD   = 2'h2,
    DOUBLE = 2'h3
} MEM_SIZE;

// Memory bus commands
typedef enum logic [1:0] {
    BUS_NONE   = 2'h0,
    BUS_LOAD   = 2'h1,
    BUS_STORE  = 2'h2
} BUS_COMMAND;

///////////////////////////////
// ---- Exception Codes ---- //
///////////////////////////////

/**
 * Exception codes for when something goes wrong in the processor.
 * Note that we use HALTED_ON_WFI to signify the end of computation.
 * It's original meaning is to 'Wait For an Interrupt', but we generally
 * ignore interrupts in 470
 *
 * This mostly follows the RISC-V Privileged spec
 * except a few add-ons for our infrastructure
 * The majority of them won't be used, but it's good to know what they are
 */

typedef enum logic [3:0] {
    INST_ADDR_MISALIGN  = 4'h0,
    INST_ACCESS_FAULT   = 4'h1,
    ILLEGAL_INST        = 4'h2,
    BREAKPOINT          = 4'h3,
    LOAD_ADDR_MISALIGN  = 4'h4,
    LOAD_ACCESS_FAULT   = 4'h5,
    STORE_ADDR_MISALIGN = 4'h6,
    STORE_ACCESS_FAULT  = 4'h7,
    ECALL_U_MODE        = 4'h8,
    ECALL_S_MODE        = 4'h9,
    NO_ERROR            = 4'ha, // a reserved code that we use to signal no errors
    ECALL_M_MODE        = 4'hb,
    INST_PAGE_FAULT     = 4'hc,
    LOAD_PAGE_FAULT     = 4'hd,
    HALTED_ON_WFI       = 4'he, // 'Wait For Interrupt'. In 470, signifies the end of computation
    STORE_PAGE_FAULT    = 4'hf
} EXCEPTION_CODE;

///////////////////////////////////
// ---- Instruction Typedef ---- //
///////////////////////////////////

// from the RISC-V ISA spec
typedef union packed {
    logic [31:0] inst;
    struct packed {
        logic [6:0] funct7;
        logic [4:0] rs2; // source register 2
        logic [4:0] rs1; // source register 1
        logic [2:0] funct3;
        logic [4:0] rd; // destination register
        logic [6:0] opcode;
    } r; // register-to-register instructions
    struct packed {
        logic [11:0] imm; // immediate value for calculating address
        logic [4:0]  rs1; // source register 1 (used as address base)
        logic [2:0]  funct3;
        logic [4:0]  rd;  // destination register
        logic [6:0]  opcode;
    } i; // immediate or load instructions
    struct packed {
        logic [6:0] off; // offset[11:5] for calculating address
        logic [4:0] rs2; // source register 2
        logic [4:0] rs1; // source register 1 (used as address base)
        logic [2:0] funct3;
        logic [4:0] set; // offset[4:0] for calculating address
        logic [6:0] opcode;
    } s; // store instructions
    struct packed {
        logic       of;  // offset[12]
        logic [5:0] s;   // offset[10:5]
        logic [4:0] rs2; // source register 2
        logic [4:0] rs1; // source register 1
        logic [2:0] funct3;
        logic [3:0] et;  // offset[4:1]
        logic       f;   // offset[11]
        logic [6:0] opcode;
    } b; // branch instructions
    struct packed {
        logic [19:0] imm; // immediate value
        logic [4:0]  rd; // destination register
        logic [6:0]  opcode;
    } u; // upper-immediate instructions
    struct packed {
        logic       of; // offset[20]
        logic [9:0] et; // offset[10:1]
        logic       s;  // offset[11]
        logic [7:0] f;  // offset[19:12]
        logic [4:0] rd; // destination register
        logic [6:0] opcode;
    } j;  // jump instructions

// extensions for other instruction types
`ifdef ATOMIC_EXT
    struct packed {
        logic [4:0] funct5;
        logic       aq;
        logic       rl;
        logic [4:0] rs2;
        logic [4:0] rs1;
        logic [2:0] funct3;
        logic [4:0] rd;
        logic [6:0] opcode;
    } a; // atomic instructions
`endif
`ifdef SYSTEM_EXT
    struct packed {
        logic [11:0] csr;
        logic [4:0]  rs1;
        logic [2:0]  funct3;
        logic [4:0]  rd;
        logic [6:0]  opcode;
    } sys; // system call instructions
`endif

} INST; // instruction typedef, this should cover all types of instructions

////////////////////////////////////////
// ---- Datapath Control Signals ---- //
////////////////////////////////////////

// ALU opA input mux selects
typedef enum logic [1:0] {
    OPA_IS_RS1  = 2'h0,
    OPA_IS_NPC  = 2'h1,
    OPA_IS_PC   = 2'h2,
    OPA_IS_ZERO = 2'h3
} ALU_OPA_SELECT;

// ALU opB input mux selects
typedef enum logic [3:0] {
    OPB_IS_RS2    = 4'h0,
    OPB_IS_I_IMM  = 4'h1,
    OPB_IS_S_IMM  = 4'h2,
    OPB_IS_B_IMM  = 4'h3,
    OPB_IS_U_IMM  = 4'h4,
    OPB_IS_J_IMM  = 4'h5
} ALU_OPB_SELECT;

// ALU function code input
// probably want to leave these alone
typedef enum logic [4:0] {
    ALU_ADD     = 5'h00,
    ALU_SUB     = 5'h01,
    ALU_SLT     = 5'h02,
    ALU_SLTU    = 5'h03,
    ALU_AND     = 5'h04,
    ALU_OR      = 5'h05,
    ALU_XOR     = 5'h06,
    ALU_SLL     = 5'h07,
    ALU_SRL     = 5'h08,
    ALU_SRA     = 5'h09,
    ALU_MUL     = 5'h0a, // Mult FU
    ALU_MULH    = 5'h0b, // Mult FU
    ALU_MULHSU  = 5'h0c, // Mult FU
    ALU_MULHU   = 5'h0d, // Mult FU
    ALU_DIV     = 5'h0e, // unused
    ALU_DIVU    = 5'h0f, // unused
    ALU_REM     = 5'h10, // unused
    ALU_REMU    = 5'h11  // unused
} ALU_FUNC;

////////////////////////////////
// ---- Datapath Packets ---- //
////////////////////////////////
`define SD #1
/**
 * Packets are used to move many variables between modules with
 * just one datatype, but can be cumbersome in some circumstances.
 *
 * Define new ones in project 4 at your own discretion
 */

//Cache
`define CACHE_LINE 32
`define CACHE_LINE_BITS $clog2(`CACHE_LINE)
`define ICACHE_WAY 2
`define ICACHE_LINE_NUM  (`CACHE_LINE/`ICACHE_WAY)
`define ICACHE_TAG_WIDTH (13-$clog2(`ICACHE_LINE_NUM))

`define DCACHE_WAY 1
`define DCACHE_LINE_NUM  (`CACHE_LINE/`DCACHE_WAY)
`define DCACHE_TAG_WIDTH (13-$clog2(`DCACHE_LINE_NUM))

typedef struct packed {
    logic [63:0]                  data;
    // (13 bits) since only need 16 bits to access all memory and 3 are the offset
    logic [12-`CACHE_LINE_BITS:0] tags;
    logic                         valid;
} ICACHE_ENTRY;

typedef enum logic [1:0] {
	IDLE = 2'b00,
	LOAD = 2'b01,
	PREF = 2'b10
} PREFETCH_STATE;

typedef struct packed {
	logic [63:0] data;
	logic [8:0] tag;
	logic		 valid;
	logic		 dirty;
} CACHE_LINE;

//ICache ------ IF
/**
 * ICACHE2IF_PACKET:
 * Data from icache to if
 */


 typedef struct packed {
	logic [31:0] Icache_data_out;
	// logic             Icache_hit;
	logic             Icache_valid_out;
} ICACHE_IF_PACKET;

/**
 * IF2ICACHE_PACKET:
 * Data from icache to if
 */
typedef struct packed {
	logic [`XLEN-1:0] Icache_addr_in;
	logic             Icache_request;
} IF_ICACHE_PACKET;


//////////////////////////////////////////////
//
// PHT section
//
//////////////////////////////////////////////

`define H_SIZE 8
`define PHT_SIZE 32

//////////////////////////////////////////////
//
// BHT section
//
//////////////////////////////////////////////
   
`define BHT_SIZE 32
`define BHT_WIDTH $clog2(`H_SIZE)//3

//////////////////////////////////////////////
//
// BTB section
//
//////////////////////////////////////////////

`define BTB_SIZE 32
`define TAG_SIZE 10
`define VAL_SIZE 12



/**
 * IF_IB_PACKET:
 * Data from if to dp
 */
typedef struct packed {
	logic             valid; // If low, the data in this struct is garbage
    INST              inst;  // fetched instruction out
	logic [`XLEN-1:0] NPC; // PC + 4
	logic [`XLEN-1:0] PC;  // PC 
} IF_IB_PACKET;

/**
 * IB_ID Packet:
 * Data exchanged from the IF to the ID stage
 */

 typedef struct packed {
	logic             valid; // If low, the data in this struct is garbage
    INST              inst;  // fetched instruction out
	logic [`XLEN-1:0] NPC; // PC + 4
	logic [`XLEN-1:0] PC;  // PC 
} IB_ID_PACKET;



/**
 * DP_RS Packet:
 * Data exchanged from the ID to the EX stage
 */
typedef struct packed {
    INST              inst;
    logic [`XLEN-1:0] PC;
    logic [`XLEN-1:0] NPC; // PC + 4

    logic [`XLEN-1:0] rs1_value; // reg A value
    logic [`XLEN-1:0] rs2_value; // reg B value

    ALU_OPA_SELECT opa_select; // ALU opa mux select (ALU_OPA_xxx *)
    ALU_OPB_SELECT opb_select; // ALU opb mux select (ALU_OPB_xxx *)

    logic [4:0] dest_reg_idx;  // destination (writeback) register index
    ALU_FUNC    alu_func;      // ALU function select (ALU_xxx *)
    logic       rd_mem;        // Does inst read memory?
    logic       wr_mem;        // Does inst write memory?
    logic       cond_branch;   // Is inst a conditional branch?
    logic       uncond_branch; // Is inst an unconditional branch?
    logic       halt;          // Is this a halt?
    logic       illegal;       // Is this instruction illegal?
    logic       csr_op;        // Is this a CSR operation? (we use this to get return code)

    logic       valid;
    logic       mem;
} DP_RS_PACKET;

`define ROB_SIZE 32
`define ROB_ADDR_BITS $clog2(`ROB_SIZE)
typedef struct packed {
    INST                        inst;
    logic [`XLEN-1:0]           PC;
    logic [`XLEN-1:0]           NPC; // PC + 4
    logic [4:0]                 dest_reg_idx;  // destination (writeback) register index
    logic                       rd_mem;        // Does inst read memory?
    logic                       wr_mem;        // Does inst write memory?
	logic [`ROB_ADDR_BITS-1:0]  Tag;  // #ROB
    logic                       rd_unsigned;
} DP_LSQ_PACKET;


 typedef struct packed {
	logic                            valid; // If low, the data in this struct is garbage
    logic [$clog2(`ROB_SIZE)-1:0]    rob_entry;//4:0
    logic [`XLEN-1:0]           PC;
} MAPTABLE;
typedef struct packed {
    logic [4:0]         dest_reg_idx;  // destination (writeback) register index
    logic [`XLEN-1:0]   PC;
    logic [`XLEN-1:0]   NPC;
    logic IsBranch;
    logic mem;
    logic wr_mem;
} DP_ROB_PACKET;



`define LSQ_SIZE 8
typedef struct packed {
    INST                inst;//todo
	logic [4:0] 		reg_idx; //enabletodo
    logic [`XLEN-1:0]	value; // cdbtodo
	logic 				cp_bit;    // cbdtodo
	logic				ep_bit;    // ??todo
	logic [`XLEN-1:0]	NPC;    //enabletodo
	logic [`XLEN-1:0]	PC;     //enabletodo
	logic             	halt, illegal; // ??
	logic             	valid;    	// enable=0 cdb=1todo
    logic               wr_mem; 		// ??
    logic               CantComplete;
    logic [`XLEN-1:0]   alu_result;  // alu_result
    logic               IsBranch;
    logic [$clog2(`LSQ_SIZE)-1:0]   lsq_idx;
} ROB_ENTRY;


typedef struct packed {
	ROB_ENTRY		  			rob_entry;
	logic [`ROB_ADDR_BITS-1:0]  Tag;  // #ROB
} CP_RT_PACKET;

typedef struct packed {
	logic [$clog2(`ROB_SIZE)-1:0]  Tag; 
    logic [`XLEN-1:0]              rs1_value, rs2_value;
	logic [$clog2(`ROB_SIZE)-1:0]  RegS1_Tag;
	logic [$clog2(`ROB_SIZE)-1:0]  RegS2_Tag; 
	logic [1:0]					   valid_vector; // not valid means no #ROB
    logic [1:0]                    complete;
} ROB_RS_PACKET;

typedef struct packed {
	logic [$clog2(`ROB_SIZE)-1:0] Tag;  
} ROB_MT_PACKET;

typedef struct packed {
	logic [$clog2(`ROB_SIZE)-1:0]  RegS1_Tag;
	logic [$clog2(`ROB_SIZE)-1:0]  RegS2_Tag; 
	logic [1:0]					   valid_vector; // not valid means no #ROB
} MT_ROB_PACKET; //to ROB

//LSQ
  typedef struct packed {
    logic               valid;
    logic [`XLEN-1:0]   addr;
    logic [31:0]        data;
    MEM_SIZE            mem_size;
    logic               is_store;
    logic [$clog2(`ROB_SIZE)-1:0] rob_idx;
    INST                inst;
    logic [`XLEN-1:0]   PC;
    logic [`XLEN-1:0]   NPC;
    logic [4:0]         dest_reg_idx;
    logic               rd_unsigned;
  } lsq_entry_t;

//SQ
  typedef struct packed {
    logic                           valid;
    logic                           h_valid;
    logic [`XLEN-1:0]               addr;
    logic [31:0]                    data;
    MEM_SIZE                        mem_size;
    logic                           is_store;
    logic [$clog2(`ROB_SIZE)-1:0]   Tag;
    INST                            inst;
    logic [`XLEN-1:0]               PC;
    logic [`XLEN-1:0]               NPC;
    logic [4:0]                     dest_reg_idx;
    logic               rd_unsigned;
  } LSQ_ENTRY;

typedef struct packed {
    logic [`XLEN-1:0]               PC;
    logic                           valid;
    logic [`XLEN-1:0]               addr;
    logic [`XLEN-1:0]               data;
    logic [$clog2(`LSQ_SIZE)-1:0]   lsq_idx;
    logic                           is_store;
    MEM_SIZE                        mem_size;
} EX_LSQ_PACKET;


typedef struct packed {
    logic lsq_is_requesting; 

    logic [`XLEN-1:0] address; // the address of the memory instruction
    logic is_store; // 1 if the insn is a "store", otherwise "load"
    logic [`XLEN-1:0] value; // only useful if the insn is a "store".

    MEM_SIZE    mem_size; 
} DCACHE_IN_PACKET;

typedef struct packed {
    // In the same cycle:
    // If the insn given by IN_PACKET is a "load": 1 if the OUT_PACKET.value is valid, otherwise 0.
    // If the insn is a "store": 1 if dcache completed this transaction (so insn can retire), otheriwse 0. 
    // If lsq_is_requesting == 0, then completed is always 0.
    logic completed;
    // Dcache is processing a miss. Don't let LSQ process any other instructions until = 0!
	// this is equivalent to (~completed | ~lsq_is_requesting).
	// It might cause logic loops
    // logic dcache_stall;
    logic [`XLEN-1:0] value; // only useful if the insn is a "load"
} DCACHE_OUT_PACKET;

typedef struct packed {
    logic valid;
    // So the real address of this block is:
    // {tag, index, 3'b0}
    logic [24:0] tag;
    logic [63:0] data;
} DCACHE_LINE;

typedef struct packed {
    logic last_accessed;
    DCACHE_LINE [1:0] line;
} DCACHE_DATASET;

typedef enum logic [2:0] { 
    DCACHE_IDLE_HIT = 3'h0,
    DCACHE_ST_EVICT = 3'h1,
    DCACHE_LD_EVICT = 3'h2,
    DCACHE_ST_WAIT = 3'h3,
    DCACHE_LD_WAIT = 3'h4
} DCACHE_STATE;

typedef struct packed {
    INST              inst;
	logic [`XLEN-1:0] Value;  // alu_result
    logic [`XLEN-1:0] PC; 
	logic [`XLEN-1:0] NPC;         // pc + 4, forwarded
	logic             take_branch; // is this a taken branch?, forwarded
    //INST              inst; 		// forwarded
	logic [4:0]       dest_reg_idx; // forwarded
	logic             halt, illegal; // forwarded
	logic             done;
	logic             valid;
	logic [`ROB_ADDR_BITS-1:0]  Tag;  // #ROB
    logic [`XLEN-1:0] alu_result;  // alu_result
} CDB_PACKET;

typedef enum logic [1:0] {
	FUNC_NOP    = 2'h0,    // no instruction free, DO NOT USE THIS AS DEFAULT CASE!
	FUNC_ALU    = 2'h1,    // all of the instruction  except mult and load and store
	FUNC_MULT   = 2'h2,    // mult 
	FUNC_MEM    = 2'h3    // load and store
} FUNC_UNIT;

typedef struct packed {
    INST inst;                 // instruction
	logic [`XLEN-1:0] NPC;     // PC + 4
	logic [`XLEN-1:0] PC;      // PC                                 

    logic busy;
    logic [$clog2(`ROB_SIZE)-1:0] Tag; 
	logic [$clog2(`ROB_SIZE)-1:0] RegS1_Tag;
	logic [$clog2(`ROB_SIZE)-1:0] RegS2_Tag; 
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

    logic [1:0] ready;
    logic [1:0] received;
    logic [1:0] needTag;
    logic [$clog2(`LSQ_SIZE)-1:0]   lsq_idx;
} RS;


typedef struct packed {
    INST                            inst;
    logic [`XLEN-1:0]               PC;
    logic [`XLEN-1:0]               NPC; // PC + 4

    logic [`XLEN-1:0]               rs1_value; // reg A value
    logic [`XLEN-1:0]               rs2_value; // reg B value

    ALU_OPA_SELECT                  opa_select; // ALU opa mux select (ALU_OPA_xxx *)
    ALU_OPB_SELECT                  opb_select; // ALU opb mux select (ALU_OPB_xxx *)

    logic [4:0]                     dest_reg_idx;  // destination (writeback) register index
    ALU_FUNC                        alu_func;      // ALU function select (ALU_xxx *)
    logic                           rd_mem;        // Does inst read memory?
    logic                           wr_mem;        // Does inst write memory?
    logic                           cond_branch;   // Is inst a conditional branch?
    logic                           uncond_branch; // Is inst an unconditional branch?
    logic                           halt;          // Is this a halt?
    logic                           illegal;       // Is this instruction illegal?
    logic                           csr_op;        // Is this a CSR operation? (we use this to get return code)

    logic                           valid;

    logic [$clog2(`ROB_SIZE)-1:0]   Tag;//dest rob entry
    logic [$clog2(`LSQ_SIZE)-1:0]   lsq_idx;
} RS_EX_PACKET;

/**
 * ID_EX Packet:
 * Data exchanged from the ID to the EX stage
 */
typedef struct packed {
    INST              inst;
    logic [`XLEN-1:0] PC;
    logic [`XLEN-1:0] NPC; // PC + 4

    logic [`XLEN-1:0] rs1_value; // reg A value
    logic [`XLEN-1:0] rs2_value; // reg B value

    ALU_OPA_SELECT opa_select; // ALU opa mux select (ALU_OPA_xxx *)
    ALU_OPB_SELECT opb_select; // ALU opb mux select (ALU_OPB_xxx *)

    logic [4:0] dest_reg_idx;  // destination (writeback) register index
    ALU_FUNC    alu_func;      // ALU function select (ALU_xxx *)
    logic       rd_mem;        // Does inst read memory?
    logic       wr_mem;        // Does inst write memory?
    logic       cond_branch;   // Is inst a conditional branch?
    logic       uncond_branch; // Is inst an unconditional branch?
    logic       halt;          // Is this a halt?
    logic       illegal;       // Is this instruction illegal?
    logic       csr_op;        // Is this a CSR operation? (we use this to get return code)

    logic       valid;
    logic [$clog2(`ROB_SIZE)-1:0] Tag;//dest rob entry
} ID_EX_PACKET;

/**
 * EX_MEM Packet:
 * Data exchanged from the EX to the MEM stage
 */
typedef struct packed {
    logic [`XLEN-1:0] alu_result;
    logic [`XLEN-1:0] NPC;

    logic             take_branch; // Is this a taken branch?
    // Pass-through from decode stage
    logic [`XLEN-1:0] rs2_value;
    logic             rd_mem;
    logic             wr_mem;
    logic [4:0]       dest_reg_idx;
    logic             halt;
    logic             illegal;
    logic             csr_op;
    logic             rd_unsigned; // Whether proc2Dmem_data is signed or unsigned
    MEM_SIZE          mem_size;
    logic             valid;
} EX_MEM_PACKET;


typedef struct packed {
    INST              inst;
    logic [`XLEN-1:0] alu_result;
    logic [`XLEN-1:0] NPC;
    logic [`XLEN-1:0] PC;

    logic             take_branch; // Is this a taken branch?
    // Pass-through from decode stage
    logic [`XLEN-1:0] rs2_value;
    logic             rd_mem;
    logic             wr_mem;
    logic [4:0]       dest_reg_idx;
    logic             halt;
    logic             illegal;
    logic             csr_op;
    logic             rd_unsigned; // Whether proc2Dmem_data is signed or unsigned
    MEM_SIZE          mem_size;
    logic             valid;
    logic [$clog2(`ROB_SIZE)-1:0] Tag;//dest rob entry
} EX_PACKET;

/**
 * MEM_WB Packet:
 * Data exchanged from the MEM to the WB stage
 *
 * Does not include data sent from the MEM stage to memory
 */
typedef struct packed {
    logic [`XLEN-1:0] result;
    logic [`XLEN-1:0] NPC;
    logic [4:0]       dest_reg_idx; // writeback destination (ZERO_REG if no writeback)
    logic             take_branch;
    logic             halt;    // not used by wb stage
    logic             illegal; // not used by wb stage
    logic             valid;
} MEM_WB_PACKET;

/**
 * No WB output packet as it would be more cumbersome than useful
 */

`endif // __SYS_DEFS_SVH__
