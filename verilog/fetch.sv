/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_if.sv                                         //
//                                                                     //
//  Description :  instruction fetch (IF) stage of the pipeline;       //
//                 fetch instruction, compute next PC location, and    //
//                 send them down the pipeline.                        //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module stage_if (
    input                           clock,          // system clock
    input                           reset,          // system reset

    input                           if_valid,       // only go to next PC when true
    input                           IsFull,

    input [`XLEN-1:0]               branch_pc,
    input                           take_branch,    // taken-branch signal
    input [`XLEN-1:0]               branch_target,  // target pc: use if take_branch is TRUE
    input                           IsBranch,
    input                           Branch_Miss,

	//From ICache
	input  ICACHE_IF_PACKET         Icache_IF_packet,

	//To ICache
	output IF_ICACHE_PACKET         IF_Icache_packet,

	//To Instruction Buffer
	output IF_IB_PACKET             if_ib_packet
);


    //////////////////////////////////////////////////////
	//	Fetch Module
	//////////////////////////////////////////////////////
    logic [`XLEN-1:0] PC_reg, NPC_reg; // PC we are currently fetching
    logic enable;

    logic [`XLEN-1:0] if_pc, bp2if_pc;
    always_comb begin
        if_pc = PC_reg;
    end

    always_comb begin//todo
        NPC_reg = bp2if_pc;
    end
    assign enable = (Icache_IF_packet.Icache_valid_out && if_valid) && !IsFull;//todo could controls (Icache_IF_packet.Icache_data_out != 32'h0)) &&

    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            PC_reg <= 0;             // initial PC value is 0 (the memory address where our program starts)
        end else if (Branch_Miss) begin
            PC_reg <= branch_target;
        end else if (enable) begin
            PC_reg <= NPC_reg;
        end
    end

    Branch_Predictor u_Branch_Predictor(
        .clock          (clock),
        .reset          (reset),

        .if_pc          (if_pc),
        // .inst(),
        // .valid          (valid),

        .IsBranch       (IsBranch),
        .take_branch    (take_branch),
        .Branch_PC      (branch_pc),
        .Branch_Target  (branch_target),

        .bp2if_pc       (bp2if_pc)
        // .predict_taken  (predict_taken)
        //.bp2if_npc      (),
        //.bp_taken()
    );

    //////////////////////////////////////////////////////
	//	To ICache
	//////////////////////////////////////////////////////
	always_comb begin
		IF_Icache_packet.Icache_addr_in = {PC_reg[`XLEN-1:2], 2'b0};
		IF_Icache_packet.Icache_request = 1;//todo could be control
	end

    //////////////////////////////////////////////////////
	//	To Inst Buffer
	//////////////////////////////////////////////////////
    always_comb begin
        if_ib_packet.valid      = enable;
        if_ib_packet.inst       = Icache_IF_packet.Icache_data_out;
        if_ib_packet.NPC        = NPC_reg;
        if_ib_packet.PC         = PC_reg;
    end

endmodule // stage_if

module Branch_Predictor(
    //system signal
    input                      clock, reset,

    //From IF Stage
    input [`XLEN-1:0]          if_pc,
    // input  INST [1:0]          inst,
    // input                      valid,

    //From EX Stage
    input                      IsBranch,
    input                      take_branch,
    input [`XLEN-1:0]          Branch_PC,
    input [`XLEN-1:0]          Branch_Target,

    //To IF Stage
    output logic  [`XLEN-1:0]  bp2if_pc
    //output logic  [`XLEN-1:0]  bp2if_npc,
    //output logic               bp_taken,
);

    // pht output
    logic predict_taken;
    
    // BTB output
    logic  hit;    // 1 if pc hit buffer 
    logic [`XLEN-1:0] predict_pc_out;


/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Modulename :  Branch_History_Table.v                               //
//                                                                     //
//  Description :  Branch History Table/Register.                      //
//      This Moudle is used for indicating the Branch History.         //
//      Get the Value of Branch History Register for given PC from IF  //
//      Stage, and Update the Branch History Register for given PC     //
//      from Excute Stage                                              //
//                                                                     //
/////////////////////////////////////////////////////////////////////////
    //bht to pht
    logic [`BHT_WIDTH-1:0] bht2pht_if;
    logic [`BHT_WIDTH-1:0] bht2pht_ex;
    Branch_History_Table u_Branch_History_Table(
    .clock             (clock), 
    .reset             (reset), 

    //From Excute
    .br_en             (IsBranch),
    .ex_pc             (Branch_PC),
    .take_branch       (take_branch),

    //From IF
    .if_pc             (if_pc),

    //To PHT
    .bht2pht_if        (bht2pht_if),
    .bht2pht_ex        (bht2pht_ex)
    );



/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Modulename :  Prediction_History_Table.v                           //
//                                                                     //
//  Description :  Prediction History Table that stores the            //
//      Prediction History, according to the bht get by branch         //
//      history table, update/read the Prediction History Table        //
//                                                                     //
/////////////////////////////////////////////////////////////////////////
    Prediction_History_Table u_Prediction_History_Table(
    .clock              (clock), 
    .reset              (reset),

    //From Excute
    .br_en              (IsBranch),
    .ex_pc              (Branch_PC),
    .take_branch        (take_branch),

    //From IF
    .if_pc              (if_pc),

    //From BHT
    .bht2pht_if         (bht2pht_if),
    .bht2pht_ex         (bht2pht_ex),

    //output
    .predict_taken      (predict_taken)
    );

//////////////////////////////////////////////////////
//  Branch Target Buffer
//  Description: Record the Branch Targets From Excute Stage
//      And judge if pc from IF Stage hit or not
//////////////////////////////////////////////////////
    Branch_Target_Buffer u_Branch_Target_Buffer (
    .clock              (clock), 
    .reset              (reset),

    //From Execute, record branch target history
    .br_en              (IsBranch),    // 1 if insn is branch (con/uncon)
    .take_branch        (take_branch),
    .ex_pc              (Branch_PC),  // pc from ex stage 
    .ex_tg_pc           (Branch_Target),    // target pc from ex stage in 

    //From IF Stage
    .if_pc              (if_pc),

    //output   
    .hit                (hit),
    .predict_pc_out     (predict_pc_out)
    );

    always_comb begin
        bp2if_pc = if_pc + 4;
        if (predict_taken && hit) begin
            bp2if_pc = predict_pc_out;
        end
    end


    // output to if, next pc multiplexer
    // always_comb begin
    //     if (link[0]) begin
    //         bp2if_npc[0] = return_addr;
    //         bp2if_npc[1] = return_addr + 4;
    //         bp2if_pc[0] = return_addr;
    //         bp2if_pc[1] = return_addr + 4;
    //     end else if (jump[0] && hit[0]) begin
    //         bp2if_npc[0] = predict_pc_out[0];
    //         bp2if_npc[1] = predict_pc_out[0] + 4;
    //         bp2if_pc[0] =  predict_pc_out[0];
    //         bp2if_pc[1] =  predict_pc_out[0] + 4;
    //     end else if (cond_branch[0] && predict_taken[0] && hit[0]) begin
    //         bp2if_npc[0] = predict_pc_out[0];
    //         bp2if_npc[1] = predict_pc_out[0] + 4;
    //         bp2if_pc[0] =  predict_pc_out[0];
    //         bp2if_pc[1] =  predict_pc_out[0] + 4;
    //     end else if (link[1]) begin
    //         bp2if_npc[0] = if_pc[0] + 4;
    //         bp2if_npc[1] = return_addr;
    //         bp2if_pc[0] =  return_addr;
    //         bp2if_pc[1] =  return_addr+4;
    //     end else if (jump[1] && hit[1]) begin
    //         bp2if_npc[0] = if_pc[0] + 4;
    //         bp2if_npc[1] = predict_pc_out[1];
    //         bp2if_pc[0] =  predict_pc_out[1];
    //         bp2if_pc[1] =  predict_pc_out[1]+4;
    //     end else if (cond_branch[1] && predict_taken[1] && hit[1]) begin
    //         bp2if_npc[0] = if_pc[0] + 4;
    //         bp2if_npc[1] = predict_pc_out[1];
    //         bp2if_pc[0] =  predict_pc_out[1];
    //         bp2if_pc[1] =  predict_pc_out[1]+4;
    //     end else begin
    //         bp2if_npc[0] = if_pc[0] + 4;
    //         bp2if_npc[1] = if_pc[1] + 4;
    //         bp2if_pc[0] =  if_pc[0] + 8;
    //         bp2if_pc[1] =  if_pc[1] + 8;
    //     end
    // end

    
endmodule


//BHT_SIZE 256 BHT_INDEX 8 BHT_WIDTH 3 SD #1
/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Modulename :  Branch_History_Table.v                               //
//                                                                     //
//  Description :  Branch History Table/Register.                      //
//      This Moudle is used for indicating the Branch History.         //
//      Get the Value of Branch History Register for given PC from IF  //
//      Stage, and Update the Branch History Register for given PC     //
//      from Excute Stage                                              //
//                                                                     //
/////////////////////////////////////////////////////////////////////////




module Branch_History_Table#(
    parameter BHT_INDEX = $clog2(`BHT_SIZE)//5
)(
    //system signal
    input  clock, reset,

    //From Excute
    //////////////////////////////////////////////////////
	//  From Excute
	//	br_en: branch enable: the instruction is a branch from excute
	//	ex_pc: Program Counter: bypassing program counter, it is the original one from dump file
    //  take_branch: Whether or not take branch
	//////////////////////////////////////////////////////
    input               br_en,
    input  [`XLEN-1:0]  ex_pc,
    input               take_branch,

    //From IF
    //////////////////////////////////////////////////////
    //  From IF
	//	if_pc: Program Counter from IF
	//////////////////////////////////////////////////////
    input  [`XLEN-1:0]  if_pc,  

    //To PHT
    //////////////////////////////////////////////////////
    //  To PHT
	//	bht_if: reading: which PHT to goto
	//	bht_ex: writing: which PHT to update
	//////////////////////////////////////////////////////
    output [`BHT_WIDTH-1:0] bht2pht_if,//3
    output [`BHT_WIDTH-1:0] bht2pht_ex //3
);

    logic [`BHT_WIDTH-1:0] bht [`BHT_SIZE-1:0];//3 - 0:31

    //////////////////////////////////////////////////////
	//	rptr: reading: decoded from if_pc, which entry to read
	//	wptr: writing: decoded from ex_pc, which entry to update
	//////////////////////////////////////////////////////
    logic [BHT_INDEX-1:0] rptr;
    logic [BHT_INDEX-1:0] wptr;
    // calculate the address
    always_comb begin
        wptr = ex_pc[2 +: BHT_INDEX];//[6:2]
        rptr = if_pc[2 +: BHT_INDEX];
    end

    //////////////////////////////////////////////////////
	//	bht: writing: update the new entry with addr = wptr, value = {bht[1:0],take_branch}
	//////////////////////////////////////////////////////
    always_ff @(posedge clock) begin
        if (reset) begin
            for (int i=0;i<`BHT_SIZE;i++) begin
                bht[i] <= 0;
            end
        end 
        else begin
            if (br_en) begin
                bht[wptr] <= {bht[wptr][`BHT_WIDTH-2:0],take_branch};
            end
        end
    end

    //////////////////////////////////////////////////////
	//	bht: reading: read the new entry with addr = rptr/wrtr
	//////////////////////////////////////////////////////

    assign bht2pht_if = bht[rptr];
    assign bht2pht_ex = bht[wptr];


endmodule



/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Modulename :  Prediction_History_Table.v                           //
//                                                                     //
//  Description :  Prediction History Table that stores the            //
//      Prediction History, according to the bht get by branch         //
//      history table, update/read the Prediction History Table        //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

typedef enum logic [1:0]{
	NT_STRONG  = 2'h0,    // assume branch taken strong
	NT_WEAK    = 2'h1,    // assume branch taken weak
	T_WEAK     = 2'h2,
	T_STRONG   = 2'h3    // assume branch no taken
}PHT_STATE;

module Prediction_History_Table #(
    parameter PHT_INDEX = $clog2(`PHT_SIZE)//5 32
) (
    input                           clock, reset,

    //From Excute Stage, update the prediction
    input                           br_en,
    input        [`XLEN-1:0]        ex_pc,
    input                           take_branch,

    //From IF, predict the npc
    input        [`XLEN-1:0]        if_pc,

    //From BHT, predict the npc
    input        [`BHT_WIDTH-1:0]   bht2pht_if,  
    input        [`BHT_WIDTH-1:0]   bht2pht_ex,//3

    //To multiplexer, selecting the npc, predict taken or not
    output logic                    predict_taken
);
    PHT_STATE  state   [`PHT_SIZE-1:0] [`H_SIZE-1:0];//2 32 7:0 Prediction History Table
    PHT_STATE  n_state [`PHT_SIZE-1:0] [`H_SIZE-1:0];

    logic [PHT_INDEX-1:0] wptr;    // write pointer for updating
    logic [PHT_INDEX-1:0] rptr;    // read pointer for predict
    
    always_comb begin
        wptr = ex_pc[2 +: PHT_INDEX];//[6:2]
        rptr = if_pc[2 +: PHT_INDEX];
    end
    
    always_comb begin: Update
        for (int j=0;j<`PHT_SIZE;j++) begin
            for (int m=0;m<`H_SIZE;m++) begin
                n_state[j][m] = state[j][m];
            end
        end
        case (state[wptr][bht2pht_ex])
            NT_STRONG: n_state[wptr][bht2pht_ex] = take_branch ? NT_WEAK  : NT_STRONG;
            NT_WEAK:   n_state[wptr][bht2pht_ex] = take_branch ? T_STRONG : NT_STRONG;
            T_WEAK:    n_state[wptr][bht2pht_ex] = take_branch ? T_STRONG : NT_STRONG;
            T_STRONG:  n_state[wptr][bht2pht_ex] = take_branch ? T_STRONG : T_WEAK;
        endcase
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            for (int k=0;k<`PHT_SIZE;k++) begin
                for (int n=0;n<`H_SIZE;n++) begin
                    state[k][n] <=  NT_WEAK;
                end
            end
        end
        else if (br_en) begin
            state [wptr][bht2pht_ex] <= n_state[wptr][bht2pht_ex];
        end
    end

    
    always_comb begin: read
        predict_taken = ((state[rptr][bht2pht_if]== T_WEAK) | (state[rptr][bht2pht_if]== T_STRONG)) ? 1 : 0;
    end

endmodule

//todo
//BTB_SIZE 256 BTB_INDEX 8 TAG_SIZE 10 VAL_SIZE 12
//////////////////////////////////////////////////////
//  Branch Target Buffer
//  Description: Record the Branch Targets From Excute Stage
//      And judge if pc from IF Stage hit or not
//////////////////////////////////////////////////////


module Branch_Target_Buffer #(
    parameter BTB_INDEX = $clog2(`BTB_SIZE)//5
) (
    input  clock, reset,

    //From Execute, record branch target history
    input                           br_en,
    input        [`XLEN-1:0]        ex_pc,
    input        [`XLEN-1:0]        ex_tg_pc,
    input                           take_branch,

    //From IF Stage
    input        [`XLEN-1:0]        if_pc,

    //Output
    output logic                    hit,
    output logic [`XLEN-1:0]        predict_pc_out
);
    logic [`TAG_SIZE+`VAL_SIZE-1:0] mem [`BTB_SIZE-1:0];//22 32
    logic [`BTB_SIZE-1:0] valid;    // 1 if address store valid target PC
   
    always_ff @(posedge clock) begin
        if (reset) begin
            for (int i=0;i<`BTB_SIZE;i++) begin
                mem [i] <= 0;
            end
            valid <= 0;
        end
        else if (br_en) begin
            if (take_branch) begin
                mem[ex_pc[2 +: BTB_INDEX]]   <= {ex_pc[BTB_INDEX + 2 +: `TAG_SIZE], ex_tg_pc[2 +: `VAL_SIZE]};//mem[ex_pc[6:2]] <= 
                valid[ex_pc[2 +: BTB_INDEX]] <= 1'b1;
            end
        end
    end

    // assign predict_pc_out = {if_pc[`XLEN-1:`VAL_SIZE+2], mem[if_pc[BTB_INDEX+1-:BTB_INDEX]][`VAL_SIZE-1:0],{2{1'b0}}};    

    // assign hit = (if_pc[BTB_INDEX+2 +: `TAG_SIZE] == mem[if_pc[BTB_INDEX+1-:BTB_INDEX]][`VAL_SIZE +: `TAG_SIZE]) & valid[if_pc[BTB_INDEX+1-:BTB_INDEX]];

    assign predict_pc_out = {if_pc[`XLEN-1:`VAL_SIZE+2], mem[if_pc[2 +: BTB_INDEX]][`VAL_SIZE-1:0],{2{1'b0}}};    

    assign hit = (if_pc[BTB_INDEX+2 +: `TAG_SIZE] == mem[if_pc[2 +: BTB_INDEX]][`VAL_SIZE +: `TAG_SIZE]) & valid[if_pc[2 +: BTB_INDEX]];
endmodule

// typedef enum logic [1:0]{
// NT_STRONG  = 2'h0,    // assume branch taken strong
// NT_WEAK    = 2'h1,    // assume branch taken weak
// T_WEAK     = 2'h2,
// T_STRONG   = 2'h3    // assume branch no taken
// }PHT_STATE;
//PHT_SIZE 256 PHT_INDEX 8 H_SIZE 3 XLEN 32 BHT_WIDTH 3
