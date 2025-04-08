/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  pipeline.sv                                         //
//                                                                     //
//  Description :  Top-level module of the verisimple pipeline;        //
//                 This instantiates and connects the 5 stages of the  //
//                 Verisimple pipeline together.                       //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module pipeline (
    input        clock,             // System clock
    input        reset,             // System reset
    input [3:0]  mem2proc_response, // Tag from memory about current request
    input [63:0] mem2proc_data,     // Data coming back from memory
    input [3:0]  mem2proc_tag,      // Tag from memory about current reply

    output logic [1:0]       proc2mem_command, // Command sent to memory
    output logic [`XLEN-1:0] proc2mem_addr,    // Address sent to memory
    output logic [63:0]      proc2mem_data,    // Data sent to memory
`ifndef CACHE_MODE // no longer sending size to memory
    output MEM_SIZE          proc2mem_size,    // Data size sent to memory
`endif

    // Note: these are assigned at the very bottom of the module
    output logic [3:0]       pipeline_completed_insts,
    output EXCEPTION_CODE    pipeline_error_status,
    output logic [4:0]       pipeline_commit_wr_idx,
    output logic [`XLEN-1:0] pipeline_commit_wr_data,
    output logic             pipeline_commit_wr_en,
    output logic [`XLEN-1:0] pipeline_commit_NPC

    // Debug outputs: these signals are solely used for debugging in testbenches
    // Do not change for project 3
    // You should definitely change these for project 4
    // output logic [`XLEN-1:0] if_NPC_dbg,
    // output logic [31:0]      if_inst_dbg,
    // output logic             if_valid_dbg,
    // output logic [`XLEN-1:0] if_id_NPC_dbg,
    // output logic [31:0]      if_id_inst_dbg,
    // output logic             if_id_valid_dbg,
    // output logic [`XLEN-1:0] id_ex_NPC_dbg,
    // output logic [31:0]      id_ex_inst_dbg,
    // output logic             id_ex_valid_dbg,
    // output logic [`XLEN-1:0] ex_mem_NPC_dbg,
    // output logic [31:0]      ex_mem_inst_dbg,
    // output logic             ex_mem_valid_dbg,
    // output logic [`XLEN-1:0] mem_wb_NPC_dbg,
    // output logic [31:0]      mem_wb_inst_dbg,
    // output logic             mem_wb_valid_dbg
);

    //////////////////////////////////////////////////
    //                                              //
    //                Pipeline Wires                //
    //                                              //
    //////////////////////////////////////////////////


    // Outputs from MEM-Stage to memory
    logic [`XLEN-1:0] proc2Dmem_addr;
    logic [`XLEN-1:0] proc2Dmem_data;
    logic [1:0]       proc2Dmem_command;
    MEM_SIZE          proc2Dmem_size;

    // Outputs from WB-Stage (These loop back to the register file in ID)
    logic             wb_regfile_en;
    logic [4:0]       wb_regfile_idx;
    logic [`XLEN-1:0] wb_regfile_data;

//////////////////////////////////////////////////
//                                              //
//                Memory Outputs                //
//                                              //
//////////////////////////////////////////////////

    // these signals go to and from the processor and memory
    // we give precedence to the mem stage over instruction fetch
    // note that there is no latency in project 3
    // but there will be a 100ns latency in project 4

//     always_comb begin
//         if (proc2Dmem_command != BUS_NONE) begin // read or write DATA from memory
//             proc2mem_command = proc2Dmem_command;
//             proc2mem_addr    = proc2Dmem_addr;
// `ifndef CACHE_MODE
//             proc2mem_size    = proc2Dmem_size;  // size is never DOUBLE in project 3
// `endif
//         end else begin                          // read an INSTRUCTION from memory
//             proc2mem_command = proc2Imem_command;
//             proc2mem_addr    = proc2Imem_addr;
// `ifndef CACHE_MODE
//             proc2mem_size    = DOUBLE;          // instructions load a full memory line (64 bits)
// `endif
//         end
//         proc2mem_data = {32'b0, proc2Dmem_data};
//     end
    logic [`XLEN-1:0]       Icache2mem_addr; // goes to mem module, imem part
    BUS_COMMAND             Icache2mem_command;
    // assign proc2mem_command = (proc2Dmem_command == BUS_NONE) ? Icache2mem_command : proc2Dmem_command;//todo
    // assign proc2mem_addr    = (proc2Dmem_command == BUS_NONE) ? Icache2mem_addr : proc2Dmem_addr;
    // assign proc2mem_data    =  proc2Dmem_data;
    assign proc2mem_command = Icache2mem_command;
    assign proc2mem_addr    = Icache2mem_addr;
    assign proc2mem_data    =  proc2Dmem_data;

    /////////////////////////////////////////////////////////////////////////
    //                                                                     //
    //  Instance Name :  u_icache                                          //
    //                                                                     //
    //  Description :  Normal icache provided by 4824                      //
    //                                                                     //
    /////////////////////////////////////////////////////////////////////////
    logic mem2Icache_ack;
    //assign mem2Icache_ack = (|mem2proc_response) && (|Icache2mem_command) && (!proc2Dmem_command);//todo
    assign mem2Icache_ack = (|mem2proc_response) && (|Icache2mem_command);
    // && (!proc2Dmem_command);
    IF_ICACHE_PACKET IF_Icache_packet;
    ICACHE_IF_PACKET Icache_IF_packet;

    icache u_icache(
        //system signal
        .clock                  (clock),
        .reset                  (reset),

        // //From Retire
        // .squash_en              (1'b0), //todo

        //From MEM
        .Imem2proc_response     (mem2proc_response),  // from mem, note the "I"
        .Imem2proc_data         (mem2proc_data),      // from mem
        .Imem2proc_tag          (mem2proc_tag),       // from mem

        //From FETCH
        .proc2Icache_addr       (IF_Icache_packet.Icache_addr_in),   //addr, request
        //To Fetch
        .Icache_data_out        (Icache_IF_packet.Icache_data_out),   //data, hit, valid
        .Icache_valid_out       (Icache_IF_packet.Icache_valid_out),   //data, hit, valid
        //To MEM
        .proc2Imem_command      (Icache2mem_command),  // output to mem
        .proc2Imem_addr         (Icache2mem_addr)      // output to mem
    );

    // icache u_icache(
    //     //system signal
    //     .clock                  (clock),
    //     .reset                  (reset),

    //     //From Retire
    //     .squash_en              (1'b0), //todo

    //     //From MEM
    //     .mem2Icache_response_in (mem2proc_response),  // from mem, note the "I"
    //     .mem2Icache_data_in     (mem2proc_data),      // from mem
    //     .mem2Icache_tag_in      (mem2proc_tag),       // from mem


    //     .mem2Icache_ack_in      (mem2Icache_ack),

    //     //From FETCH
    //     .IF_Icache_packet_in    (IF_Icache_packet),   //addr, request
    //     //To Fetch
    //     .Icache_IF_packet_out   (Icache_IF_packet),   //data, hit, valid

    //     //To MEM
    //     .Icache2mem_command_out (Icache2mem_command),  // output to mem
    //     .Icache2mem_addr_out    (Icache2mem_addr)      // output to mem
    // );

    //////////////////////////////////////////////////
    //                                              //
    //                  IF-Stage                    //
    //                                              //
    //////////////////////////////////////////////////
    IF_IB_PACKET if_ib_packet_out;
    stage_if u_stage_if (
        // Inputs
        .clock                  (clock),
        .reset                  (reset),

        .if_valid               (1'b1),

        .take_branch            (1'b0),
        .branch_target          (0),

        //To Icache
        .IF_Icache_packet       (IF_Icache_packet),
        //From Icache
        .Icache_IF_packet       (Icache_IF_packet),


        .if_ib_packet_out       (if_ib_packet_out)
    );





    /////////////////////////////////////////////////////////////////////////
    //                                                                     //
    //  Instance Name :  u_dcache                                          //
    //                                                                     //
    //  Description :  Normal dcache provided by 4824                      //
    //                                                                     //
    /////////////////////////////////////////////////////////////////////////
    // dcache u_dcache(
    //     .clock                  (clock), 
    //     .reset                  (reset),

    //     //from LSQ
    //     .lsq2dcache_packet      (lsq2dcache_packet),

    //     //from MEMORY
    //     .Dmem2proc_response     (mem2proc_response),
    //     .Dmem2proc_data         (mem2proc_data),
    //     .Dmem2proc_tag          (mem2proc_tag),

    //     //to LSQ (and ROB)
    //     .dcache2lsq_packet      (dcache2lsq_packet),

    //     //to testbench and cache set
    //     .cache_data             (dcache_data),

    //     //to MEMORY
    //     .proc2Dmem_addr         (proc2Dmem_addr),
    //     .proc2Dmem_data         (proc2Dmem_data),
    //     .proc2Dmem_command      (proc2Dmem_command)
    // );


    //////////////////////////////////////////////////
    //                                              //
    //               Pipeline Outputs               //
    //                                              //
    //////////////////////////////////////////////////

    // assign pipeline_completed_insts = {3'b0, mem_wb_reg.valid}; // commit one valid instruction
    // assign pipeline_error_status = mem_wb_reg.illegal        ? ILLEGAL_INST :
    //                                mem_wb_reg.halt           ? HALTED_ON_WFI :
    //                                (mem2proc_response==4'h0) ? LOAD_ACCESS_FAULT : NO_ERROR;

    // assign pipeline_commit_wr_en   = wb_regfile_en;
    // assign pipeline_commit_wr_idx  = wb_regfile_idx;
    // assign pipeline_commit_wr_data = wb_regfile_data;
    // assign pipeline_commit_NPC     = mem_wb_reg.NPC;

endmodule // pipeline
