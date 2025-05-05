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
    input        clock,                        // System clock
    input        reset,                        // System reset
    input [3:0]  mem2proc_response,            // Tag from memory about current request
    input [63:0] mem2proc_data,                // Data coming back from memory
    input [3:0]  mem2proc_tag,                 // Tag from memory about current reply

    output logic [1:0]       proc2mem_command, // Command sent to memory
    output logic [`XLEN-1:0] proc2mem_addr,    // Address sent to memory
    output logic [63:0]      proc2mem_data,    // Data sent to memory
`ifndef CACHE_MODE                             // no longer sending size to memory
    output MEM_SIZE          proc2mem_size,    // Data size sent to memory
`endif

    // Note: these are assigned at the very bottom of the module
    output logic [3:0]       pipeline_completed_insts,
    output EXCEPTION_CODE    pipeline_error_status,
    output logic [4:0]       pipeline_commit_wr_idx,
    output logic [`XLEN-1:0] pipeline_commit_wr_data,
    output logic             pipeline_commit_wr_en,
    output logic [`XLEN-1:0] pipeline_commit_PC

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
    logic [63:0] proc2Dmem_data;
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
    logic [`XLEN-1:0]       Icache2mem_addr; // goes to mem module, imem part
    BUS_COMMAND             Icache2mem_command;
    assign proc2mem_command = (proc2Dmem_command == BUS_NONE) ? Icache2mem_command : proc2Dmem_command;
    assign proc2mem_addr    = (proc2Dmem_command == BUS_NONE) ? Icache2mem_addr : proc2Dmem_addr;
    assign proc2mem_data    =  proc2Dmem_data;

    /////////////////////////////////////////////////////////////////////////
    //                                                                     //
    //  Instance Name :  u_icache                                          //
    //                                                                     //
    //  Description :  Normal icache provided by 4824                      //
    //                                                                     //
    /////////////////////////////////////////////////////////////////////////
    logic mem2Icache_ack;
    assign mem2Icache_ack = (|mem2proc_response) && (|Icache2mem_command) && (!proc2Dmem_command);
    //assign mem2Icache_ack = (|mem2proc_response) && (|Icache2mem_command);
    // && (!proc2Dmem_command);
    IF_ICACHE_PACKET IF_Icache_packet;
    ICACHE_IF_PACKET Icache_IF_packet;
    logic Branch_Miss;
    icache u_icache(
        //system signal
        .clock                  (clock),
        .reset                  (reset),

        //From MEM
        .Imem2proc_response     (mem2proc_response),  // from mem, note the "I"
        .Imem2proc_data         (mem2proc_data),      // from mem
        .Imem2proc_tag          (mem2proc_tag),       // from mem

        .proc2Dmem_command      (proc2Dmem_command),
        .Branch_Miss            (Branch_Miss),

        //From FETCH
        .proc2Icache_addr       (IF_Icache_packet.Icache_addr_in),    //addr, request
        //To Fetch
        .Icache_data_out        (Icache_IF_packet.Icache_data_out),   //data, hit, valid
        .Icache_valid_out       (Icache_IF_packet.Icache_valid_out),  //data, hit, valid
        //To MEM
        .proc2Imem_command      (Icache2mem_command),  // output to mem
        .proc2Imem_addr         (Icache2mem_addr)      // output to mem
    );



    //////////////////////////////////////////////////
    //                                              //
    //                  IF-Stage                    //
    //                                              //
    //////////////////////////////////////////////////
    EX_PACKET ex_packet;
    CDB_PACKET              CDB_packet;
    EX_PACKET ex_reg;
    IF_IB_PACKET if_ib_packet;

    logic [`XLEN-1:0] Branch_PC, PredictionPC;
    logic [`XLEN-1:0] Branch_Target;
    logic take_branch, IsBranch, predict_taken, IsFull;
    stage_if u_stage_if (
        // Inputs
        .clock                  (clock),
        .reset                  (reset),

        .if_valid               (1'b1),
        .IsFull                 (IsFull),

        .branch_pc              (Branch_PC),
        .IsBranch               (IsBranch),
        .take_branch            (take_branch),
        .branch_target          (Branch_Target),
        .Branch_Miss            (Branch_Miss),

        //To Icache
        .IF_Icache_packet       (IF_Icache_packet),
        //From Icache
        .Icache_IF_packet       (Icache_IF_packet),


        .if_ib_packet           (if_ib_packet)
        // .PredictionPC           (PredictionPC),
        // .predict_taken          (predict_taken)
    );

    //////////////////////////////////////////////////
    //                                              //
    //                  IB                          //
    //                                              //
    //////////////////////////////////////////////////
    IB_ID_PACKET ib_id_packet;
    logic rs_available;
    logic [11:0] busy;
    logic squash = 1'b0;
    logic read_enable, enable;
    logic available;//rob available

    assign read_enable = available && 1 && rs_available;
    logic rs_mt_rob_enable;
    inst_buffer u_inst_buffer(
        .clock                  (clock),
        .reset                  (reset),

        .squash                 (Branch_Miss),
        .branch_target          (Branch_Target),
        .read_enable            (read_enable),
        .ack                    (rs_mt_rob_enable),

        .if_ib_packet           (if_ib_packet),

        .enable                 (enable),
        .ib_id_packet           (ib_id_packet),
        .IsFull                 (IsFull)
    );

    //////////////////////////////////////////////////
    //                                              //
    //                  Dispatch                    //
    //                                              //
    //////////////////////////////////////////////////
    // logic             wb_regfile_en;  // register write enable
    // logic [4:0]       wb_regfile_idx; // register write index
    // logic [`XLEN-1:0] wb_regfile_data; // register write data

    ROB_MT_PACKET rob_mt_packet;
    DP_LSQ_PACKET dp_lsq_packet;
    DP_RS_PACKET  dp_rs_packet;
    DP_ROB_PACKET dp_rob_packet;
    MT_ROB_PACKET mt_rob_packet;
    CP_RT_PACKET    cp_rt_packet;   
    assign rs_mt_rob_enable = dp_rs_packet.valid && available && ((!dp_rs_packet.mem && ((!busy[7]) || (!busy[6]) || (!busy[5]) || (!busy[4]) ||(!busy[3]) || (!busy[2]) || (!busy[1]) || (!busy[0])))
                         || (dp_rs_packet.mem && ((!busy[8]) || (!busy[9])  || (!busy[10]) || (!busy[11]) ))) && enable;
    Dispatch u_Dispatch(
        .clock              (clock),
        .reset              (reset),

        .enable             (rs_mt_rob_enable),
        //input
        .ib_id_packet       (ib_id_packet),
        .rob_mt_packet      (rob_mt_packet),
        .take_branch        (Branch_Miss),
        //output
        .dp_rs_packet       (dp_rs_packet),
        .dp_rob_packet      (dp_rob_packet),
        .dp_lsq_packet      (dp_lsq_packet),
        .mt_rob_packet      (mt_rob_packet),
        .cp_rt_packet       (cp_rt_packet),


        .wb_regfile_en      (cp_rt_packet.rob_entry.cp_bit),  // register write enable
        .wb_regfile_idx     (cp_rt_packet.rob_entry.reg_idx), // register write index
        .wb_regfile_data    (cp_rt_packet.rob_entry.value)    // register write data
    );

    CDB_PACKET             lsq2cdb_packet; 

    ROB_RS_PACKET   rob_rs_packet;
    logic [$clog2(`LSQ_SIZE)-1:0]   lsq_idx;
    ROB rob(
        //Inputs
        .reset              (reset),
        .clock              (clock),

        .squash_signal      (1'b0),          // squash signal in

        .CDB_packet_in      (CDB_packet),    // from CDB
        .lsq_input          (lsq2cdb_packet),
        .lsq_idx            (lsq_idx),

        .dp_rob_packet      (dp_rob_packet),    // From dispatch stage, decoded get 1. destreg 2. pc
        .enable             (rs_mt_rob_enable),
        .mt_rob_packet      (mt_rob_packet),    // From Maptable



        //Outputs
        .available          (available),       // going to DP_stage
        .cp_rt_packet       (cp_rt_packet),    // going to retire stage
        .rob_rs_packet      (rob_rs_packet),   // going to RS
        .rob_mt_packet      (rob_mt_packet),   // going to maptable

        .Branch_PC          (Branch_PC),
        .Branch_Target      (Branch_Target),
        .take_branch        (take_branch),
        .IsBranch           (IsBranch),
        .Branch_Miss        (Branch_Miss)
    );

    //////////////////////////////////////////////////
    //                                              //
    //             Reservation Station              //
    //                                              //
    ////////////////////////////////////////////////// 
    RS_EX_PACKET rs_ex_packet;
    logic lsq_available;
    ReservationStation u_ReservationStation(
        .clock              (clock),
        .reset              (reset),

        .enable             (rs_mt_rob_enable),
        .dp_rs_packet       (dp_rs_packet),
        .rob_rs_packet      (rob_rs_packet),
        .lsq_idx            (lsq_idx),
        .cdb_packet         (CDB_packet),
        .lsq_input          (lsq2cdb_packet),

        .take_branch        (Branch_Miss),

        .rs_ex_packet       (rs_ex_packet),
        .read_enable        (rs_available),
        .busy               (busy)
    );

    EX_LSQ_PACKET ex_lsq_packet;
    execute u_execute(
        .rs_ex_packet       (rs_ex_packet),
        .ex_packet          (ex_packet),
        .ex_lsq_packet      (ex_lsq_packet)
    );
    DCACHE_IN_PACKET             lsq2dcache_packet;
    DCACHE_OUT_PACKET            dcache2lsq_packet;
    lsq u_lsq(
        .clock                  (clock),
        .reset                  (reset),

        .Branch_Miss            (Branch_Miss),
        .cp_rt_packet           (cp_rt_packet),

        .dp_lsq_packet          (dp_lsq_packet),
        .enable                 ((rs_mt_rob_enable && dp_rs_packet.mem)),
        .lsq_available          (lsq_available),
        .lsq_idx                (lsq_idx),

        .ex_lsq_packet          (ex_lsq_packet),
        .lsq2cdb                (lsq2cdb_packet),

        .lsq2dcache_packet      (lsq2dcache_packet),
        .dcache2lsq_packet      (dcache2lsq_packet)
    );

    always_ff @( posedge clock ) begin
        if (reset || Branch_Miss) begin
            ex_reg <= 0;
        end else begin
            ex_reg <= ex_packet;
        end
    end

    /////////////////////////////////////////////////////////////////////////
    //                                                                     //
    //  Instance Name :  u_dcache                                          //
    //                                                                     //
    //  Description :  Normal dcache provided by 4824                      //
    //                                                                     //
    /////////////////////////////////////////////////////////////////////////
    DCACHE_DATASET [15:0]        dcache;
    dcache u_dcache (
       .clock                  (clock),
       .reset                  (reset),

       .dcache_in              (lsq2dcache_packet),
       .Dmem2proc_resp         (mem2proc_response),
       .Dmem2proc_data         (mem2proc_data),
       .Dmem2proc_tag          (mem2proc_tag),
       .dcache2lsq_packet      (dcache2lsq_packet),

       .proc2Dmem_addr         (proc2Dmem_addr),
       .proc2Dmem_data         (proc2Dmem_data),
       .proc2Dmem_cmd          (proc2Dmem_command),

       .dcache                 (dcache)
   );


    complete u_complete(
        .clock(clock),
        .reset(reset),

        .ex_reg             (ex_reg),
        .cdb_packet         (CDB_packet),
        .wb_regfile_en      (wb_regfile_en),  // register write enable
        .wb_regfile_idx     (wb_regfile_idx), // register write index
        .wb_regfile_data    (wb_regfile_data) // register write data
    );



    //////////////////////////////////////////////////
    //                                              //
    //               Pipeline Outputs               //
    //                                              //
    //////////////////////////////////////////////////

    assign pipeline_completed_insts = {3'b0, cp_rt_packet.rob_entry.cp_bit}; // commit one valid instruction
    assign pipeline_error_status = cp_rt_packet.rob_entry.illegal        ? ILLEGAL_INST :
                                   cp_rt_packet.rob_entry.halt           ? HALTED_ON_WFI :
                                    (mem2proc_response==4'h0) ? NO_ERROR : NO_ERROR;

    assign pipeline_commit_wr_en   = cp_rt_packet.rob_entry.cp_bit && (cp_rt_packet.rob_entry.reg_idx != 5'h0);
    assign pipeline_commit_wr_idx  = cp_rt_packet.rob_entry.reg_idx;
    assign pipeline_commit_wr_data = cp_rt_packet.rob_entry.value;
    assign pipeline_commit_PC     = cp_rt_packet.rob_entry.PC;

endmodule // pipeline
