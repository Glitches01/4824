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

    input [`XLEN-1:0]               branch_pc,
    input                           take_branch,    // taken-branch signal
    input [`XLEN-1:0]               branch_target,  // target pc: use if take_branch is TRUE

	//From ICache
	input  ICACHE_IF_PACKET         Icache_IF_packet,

	//To ICache
	output IF_ICACHE_PACKET         IF_Icache_packet,

	//To Instruction Buffer
	output IF_IB_PACKET             if_ib_packet[0:1]
);


    //////////////////////////////////////////////////////
	//	Fetch Module
	//////////////////////////////////////////////////////
    logic [`XLEN-1:0] PC_reg, NPC_reg; // PC we are currently fetching
    logic enable;

    always_comb begin//todo
        NPC_reg = PC_reg + 8;
    end
    assign enable = (Icache_IF_packet.Icache_valid_out && if_valid && (Icache_IF_packet.Icache_data_out != 64'h0));//todo could controls

    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            PC_reg <= 0;             // initial PC value is 0 (the memory address where our program starts)
        end else if (take_branch) begin
            PC_reg <= branch_target;    // or transition to next PC if valid
        end else if (enable) begin
            PC_reg <= NPC_reg;
        end
    end

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
        if_ib_packet[0].valid      = Icache_IF_packet.Icache_valid_out;
        if_ib_packet[0].inst       = Icache_IF_packet.Icache_data_out[31:0];
        if_ib_packet[0].NPC        = PC_reg + 4;
        if_ib_packet[0].PC         = PC_reg;

        if_ib_packet[1].valid      = Icache_IF_packet.Icache_valid_out;
        if_ib_packet[1].inst       = Icache_IF_packet.Icache_data_out[63:32];
        if_ib_packet[1].NPC        = NPC_reg;
        if_ib_packet[1].PC         = PC_reg + 4;
    end

endmodule // stage_if




// module Branch_Predictor(
//     //system signal
//     input                           clock, reset,

//     //From IF Stage
//     input [1:0][`XLEN-1:0]          if_pc,
//     input  INST [1:0]               inst,
//     input [1:0]                     valid,
//     //To IF Stage
//     output logic [1:0] [`XLEN-1:0]  bp2if_pc,
//     output logic [1:0] [`XLEN-1:0]  bp2if_npc,
//     output logic                    bp_taken,

//     //From EX Stage
//     input EX_BP_PACKET [1:0]       ex_bp_packet_in
// );

//     // for multiplexer
//     logic [`XLEN-1:0] link_pc;
//     assign link_pc = jump[0] ?  if_pc[0] : if_pc[1];
//     logic push;    // 1 if instruction is JAL
//     logic pop;     // 1 if instruction is JALR

//     // pht output
//     logic [1:0] predict_taken;
    
//     // BTB output
//     logic [1:0] hit;    // 1 if pc hit buffer 
//     logic [1:0][`XLEN-1:0] predict_pc_out;

//     // RAS output
//     logic [`XLEN-1:0] return_addr;    // return pc only when  current insn is JALR

//     // pre_decoder output
//     logic [1:0] cond_branch, uncond_branch;
//     logic [1:0] jump,link;
    
//     assign bp_taken = link[0] | (jump[0] & hit[0]) | (cond_branch[0] & predict_taken[0] & hit[0]);
    
//     // control the RAS 
//     always_comb begin
//         if (jump[0]) begin
//             push = valid[0];
//             pop  = 0;
//         end else if (link[0]) begin
//             push = 0;
//             pop  = valid[0];
//         end else begin
//             if (jump[1]) begin
//                 push = valid[1];
//                 pop  = 0; 
//             end else if (link[1]) begin
//                 push = 0;
//                 pop  = valid[1];
//             end else begin
//                 push = 0;
//                 pop  = 0;
//             end
//         end
//     end
    

//     // output to if, next pc multiplexer
//     always_comb begin
//         if (link[0]) begin
//             bp2if_npc[0] = return_addr;
//             bp2if_npc[1] = return_addr + 4;
//             bp2if_pc[0] = return_addr;
//             bp2if_pc[1] = return_addr + 4;
//         end else if (jump[0] && hit[0]) begin
//             bp2if_npc[0] = predict_pc_out[0];
//             bp2if_npc[1] = predict_pc_out[0] + 4;
//             bp2if_pc[0] =  predict_pc_out[0];
//             bp2if_pc[1] =  predict_pc_out[0] + 4;
//         end else if (cond_branch[0] && predict_taken[0] && hit[0]) begin
//             bp2if_npc[0] = predict_pc_out[0];
//             bp2if_npc[1] = predict_pc_out[0] + 4;
//             bp2if_pc[0] =  predict_pc_out[0];
//             bp2if_pc[1] =  predict_pc_out[0] + 4;
//         end else if (link[1]) begin
//             bp2if_npc[0] = if_pc[0] + 4;
//             bp2if_npc[1] = return_addr;
//             bp2if_pc[0] =  return_addr;
//             bp2if_pc[1] =  return_addr+4;
//         end else if (jump[1] && hit[1]) begin
//             bp2if_npc[0] = if_pc[0] + 4;
//             bp2if_npc[1] = predict_pc_out[1];
//             bp2if_pc[0] =  predict_pc_out[1];
//             bp2if_pc[1] =  predict_pc_out[1]+4;
//         end else if (cond_branch[1] && predict_taken[1] && hit[1]) begin
//             bp2if_npc[0] = if_pc[0] + 4;
//             bp2if_npc[1] = predict_pc_out[1];
//             bp2if_pc[0] =  predict_pc_out[1];
//             bp2if_pc[1] =  predict_pc_out[1]+4;
//         end else begin
//             bp2if_npc[0] = if_pc[0] + 4;
//             bp2if_npc[1] = if_pc[1] + 4;
//             bp2if_pc[0] =  if_pc[0] + 8;
//             bp2if_pc[1] =  if_pc[1] + 8;
//         end
//     end

   

// /////////////////////////////////////////////////////////////////////////
// //                                                                     //
// //  Modulename :  pre_docode.v                                         //
// //                                                                     //
// //  Description :  Pre decode the branch instructions                  //
// //                                                                     //
// /////////////////////////////////////////////////////////////////////////
//     genvar i;
//     generate 
//     for (i=0;i<2;i++) begin
//         pre_docode u_pre_decode(
//             .inst           (inst[i]),
//             .valid          (valid[i]),
//             // output
//             .cond_branch    (cond_branch[i]), 
//             .uncond_branch  (uncond_branch[i]),
//             .jump           (jump[i]),    // JAL is jump insn 
//             .link           (link[i])     // JALR is link insn
//         );
//     end
//     endgenerate


// /////////////////////////////////////////////////////////////////////////
// //                                                                     //
// //  Modulename :  Branch_History_Table.v                               //
// //                                                                     //
// //  Description :  Branch History Table/Register.                      //
// //      This Moudle is used for indicating the Branch History.         //
// //      Get the Value of Branch History Register for given PC from IF  //
// //      Stage, and Update the Branch History Register for given PC     //
// //      from Excute Stage                                              //
// //                                                                     //
// /////////////////////////////////////////////////////////////////////////
//     //bht to pht
//     logic [1:0][`BHT_WIDTH-1:0] bht2pht_if;
//     logic [1:0][`BHT_WIDTH-1:0] bht2pht_ex;
//     Branch_History_Table u_Branch_History_Table(
//     .clock             (clock), 
//     .reset             (reset), 

//     //From Excute
//     .br_en             ({ex_bp_packet_in[1].con_br_en,      ex_bp_packet_in[0].con_br_en}),
//     .ex_pc             ({ex_bp_packet_in[1].PC,             ex_bp_packet_in[0].PC}),
//     .take_branch       ({ex_bp_packet_in[1].con_br_taken,   ex_bp_packet_in[0].con_br_taken}),

//     //From IF
//     .if_pc             (if_pc),

//     //To PHT
//     .bht2pht_if        (bht2pht_if),
//     .bht2pht_ex        (bht2pht_ex)
//     );



// /////////////////////////////////////////////////////////////////////////
// //                                                                     //
// //  Modulename :  Prediction_History_Table.v                           //
// //                                                                     //
// //  Description :  Prediction History Table that stores the            //
// //      Prediction History, according to the bht get by branch         //
// //      history table, update/read the Prediction History Table        //
// //                                                                     //
// /////////////////////////////////////////////////////////////////////////
//     Prediction_History_Table u_Prediction_History_Table(
//     .clock              (clock), 
//     .reset              (reset),

//     //From Excute
//     .br_en              ({ex_bp_packet_in[1].con_br_en,     ex_bp_packet_in[0].con_br_en}),
//     .ex_pc              ({ex_bp_packet_in[1].PC,            ex_bp_packet_in[0].PC}),
//     .take_branch        ({ex_bp_packet_in[1].con_br_taken,  ex_bp_packet_in[0].con_br_taken}),

//     //From IF
//     .if_pc              (if_pc),

//     //From BHT
//     .bht2pht_if         (bht2pht_if),   
//     .bht2pht_ex         (bht2pht_ex),

//     //output
//     .predict_taken      (predict_taken)
//     );

// //////////////////////////////////////////////////////
// //  Branch Target Buffer
// //  Description: Record the Branch Targets From Excute Stage
// //      And judge if pc from IF Stage hit or not
// //////////////////////////////////////////////////////
//     Branch_Target_Buffer u_Branch_Target_Buffer (
//     .clock              (clock), 
//     .reset              (reset),

//     //From Execute, record branch target history
//     .br_en              ({ex_bp_packet_in[1].br_en, ex_bp_packet_in[0].br_en}),    // 1 if insn is branch (con/uncon)
//     .ex_pc              ({ex_bp_packet_in[1].PC,    ex_bp_packet_in[0].PC}),  // pc from ex stage 
//     .ex_tg_pc           ({ex_bp_packet_in[1].tg_pc, ex_bp_packet_in[0].tg_pc}),    // target pc from ex stage in 

//     //From IF Stage
//     .if_pc              (if_pc),

//     //output   
//     .hit                (hit),
//     .predict_pc_out     (predict_pc_out)
//     );


// /////////////////////////////////////////////////////////////////////////
// //                                                                     //
// //  Modulename :  Return_Address_Stack.v                               //
// //                                                                     //
// //  Description :  Reutrn Address Stack for handling and accelerating  //
// //        Function call -> jal/jalr                                    //
// //                                                                     //
// /////////////////////////////////////////////////////////////////////////
//     Return_Address_Stack u_Return_Address_Stack(
//         .clock          (clock),
//         .reset          (reset),
//         .push           (push),
//         .pop            (pop),
//         .pc             (link_pc),
//         .return_addr    (return_addr)
//     );

    
// endmodule


// //BHT_SIZE 256 BHT_INDEX 8 BHT_WIDTH 3 SD #1
// /////////////////////////////////////////////////////////////////////////
// //                                                                     //
// //  Modulename :  Branch_History_Table.v                               //
// //                                                                     //
// //  Description :  Branch History Table/Register.                      //
// //      This Moudle is used for indicating the Branch History.         //
// //      Get the Value of Branch History Register for given PC from IF  //
// //      Stage, and Update the Branch History Register for given PC     //
// //      from Excute Stage                                              //
// //                                                                     //
// /////////////////////////////////////////////////////////////////////////

// module Branch_History_Table#(
//     parameter BHT_INDEX = $clog2(`BHT_SIZE)
// )(
//     //system signal
//     input  clock, reset,

//     //From Excute
//     //////////////////////////////////////////////////////
// 	//  From Excute
// 	//	br_en: branch enable: the instruction is a branch from excute
// 	//	ex_pc: Program Counter: bypassing program counter, it is the original one from dump file
//     //  take_branch: Whether or not take branch
// 	//////////////////////////////////////////////////////
//     input  [1:0] br_en,
//     input  [1:0] [`XLEN-1:0] ex_pc,
//     input  [1:0] take_branch,

//     //From IF
//     //////////////////////////////////////////////////////
//     //  From IF
// 	//	if_pc: Program Counter from IF
// 	//////////////////////////////////////////////////////
//     input  [1:0][`XLEN-1:0] if_pc,  

//     //To PHT
//     //////////////////////////////////////////////////////
//     //  To PHT
// 	//	bht_if: reading: which PHT to goto
// 	//	bht_ex: writing: which PHT to update
// 	//////////////////////////////////////////////////////
//     output [1:0][`BHT_WIDTH-1:0] bht2pht_if,
//     output [1:0][`BHT_WIDTH-1:0] bht2pht_ex
// );

//     logic [`BHT_WIDTH-1:0] bht [`BHT_SIZE-1:0];//3 - 0:255

//     //////////////////////////////////////////////////////
// 	//	rptr: reading: decoded from if_pc, which entry to read
// 	//	wptr: writing: decoded from ex_pc, which entry to update
// 	//////////////////////////////////////////////////////
//     logic [BHT_INDEX-1:0] rptr [1:0];
//     logic [BHT_INDEX-1:0] wptr [1:0];
//     // calculate the address
//     always_comb begin
//         for (int i=0;i<2;i++) begin
//             wptr[i] = ex_pc[i][2 +: BHT_INDEX];//[2:9]
//             rptr[i] = if_pc[i][2 +: BHT_INDEX];
//         end
//     end

//     //////////////////////////////////////////////////////
// 	//	bht: writing: update the new entry with addr = wptr, value = {bht[1:0],take_branch}
// 	//////////////////////////////////////////////////////
//     always_ff @(posedge clock) begin
//         if (reset) begin
//             for (int i=0;i<`BHT_SIZE;i++) begin
//                 bht[i] <= `DELAY 0;
//             end
//         end 
//         else begin
//             if (br_en[0] && br_en[1] && (wptr[0] == wptr[1])) begin
//                 bht[wptr[0]] <= `DELAY {bht[wptr[0]][`BHT_WIDTH-2:0],take_branch[0]};
//             end
//             else begin
//                 if (br_en[0]) begin
//                     bht[wptr[0]] <= `DELAY {bht[wptr[0]][`BHT_WIDTH-2:0],take_branch[0]};
//                 end
//                 if (br_en[1])begin
//                     bht[wptr[1]] <= `DELAY {bht[wptr[1]][`BHT_WIDTH-2:0],take_branch[1]};
//                 end
//             end
//         end
//     end 

//     //////////////////////////////////////////////////////
// 	//	bht: reading: read the new entry with addr = rptr/wrtr
// 	//////////////////////////////////////////////////////
//     genvar j;
//     for (j=0;j<2;j++) begin
//         assign bht2pht_if[j] = bht[rptr[j]];
//         assign bht2pht_ex[j] = bht[wptr[j]];
//     end

// endmodule

// //BTB_SIZE 256 BTB_INDEX 8 TAG_SIZE 10 VAL_SIZE 12
// //////////////////////////////////////////////////////
// //  Branch Target Buffer
// //  Description: Record the Branch Targets From Excute Stage
// //      And judge if pc from IF Stage hit or not
// //////////////////////////////////////////////////////
// module Branch_Target_Buffer #(
//     parameter BTB_INDEX = $clog2(`BTB_SIZE)
// ) (
//     input  clock, reset,

//     //From Execute, record branch target history
//     input  [1:0] br_en,
//     input  [1:0] [`XLEN-1:0] ex_pc,
//     input  [1:0] [`XLEN-1:0] ex_tg_pc,

//     //From IF Stage
//     input  [1:0][`XLEN-1:0] if_pc,

//     //Output
//     output logic [1:0] hit,
//     output logic [1:0][`XLEN-1:0] predict_pc_out
// );
//     logic [`TAG_SIZE+`VAL_SIZE-1:0] mem [`BTB_SIZE-1:0];//22 256
//     logic [`BTB_SIZE-1:0] valid;    // 1 if address store valid target PC
   
//     always_ff @(posedge clock) begin
//         if (reset) begin
//             for (int i=0;i<`BTB_SIZE;i++) begin
//             mem [i] <= `DELAY 0;
//             end
//             valid <= `DELAY 0;
//         end
//         else begin
//             // if two pc return from ex stage is same, BTB will record the ex_tg_pc_in[0]
//             // doesn't matter because it is just a guss
//             if (br_en[0] && br_en[1] && (ex_pc[0][2 +: BTB_INDEX]==ex_pc[1][2 +: BTB_INDEX])) begin
//                 mem[ex_pc[0][2 +: BTB_INDEX]] <= `DELAY {ex_pc[0][BTB_INDEX+2 +: `TAG_SIZE], ex_tg_pc[0][2 +: `VAL_SIZE]};
//                 valid[ex_pc[0][2 +: BTB_INDEX]] <= `DELAY 1'b1;
//             end 
//             else begin
//                 if (br_en[0]) begin
//                     mem [ex_pc[0][2 +: BTB_INDEX]] <= `DELAY {ex_pc[0][BTB_INDEX+2 +: `TAG_SIZE], ex_tg_pc[0][2 +: `VAL_SIZE]};
//                     valid[ex_pc[0][2 +: BTB_INDEX]] <= `DELAY 1'b1;
//                 end
//                 if (br_en[1]) begin
//                     mem [ex_pc[1][2 +: BTB_INDEX]] <= `DELAY {ex_pc[1][BTB_INDEX+2 +: `TAG_SIZE], ex_tg_pc[1][2 +: `VAL_SIZE]};
//                     valid[ex_pc[1][2 +: BTB_INDEX]] <= `DELAY 1'b1;
//                 end
//             end
//         end
//     end

//     genvar j,k;
//     for (j=0;j<2;j++) begin
//         assign predict_pc_out[j] = {if_pc[j][`XLEN-1:`VAL_SIZE+2], mem[if_pc[j][BTB_INDEX+1-:BTB_INDEX]][`VAL_SIZE-1:0],{2{1'b0}}};    
//     end



//     for (k=0;k<2;k++) begin
//         assign hit[k] = (if_pc[k][BTB_INDEX+2 +: `TAG_SIZE] == mem[if_pc[k][BTB_INDEX+1-:BTB_INDEX]][`VAL_SIZE +: `TAG_SIZE]) & valid[if_pc[k][BTB_INDEX+1-:BTB_INDEX]];
//     end

// endmodule

// // typedef enum logic [1:0]{
// // NT_STRONG  = 2'h0,    // assume branch taken strong
// // NT_WEAK    = 2'h1,    // assume branch taken weak
// // T_WEAK     = 2'h2,
// // T_STRONG   = 2'h3    // assume branch no taken
// // }PHT_STATE;
// //PHT_SIZE 256 PHT_INDEX 8 H_SIZE 3 XLEN 32 BHT_WIDTH 3
// /////////////////////////////////////////////////////////////////////////
// //                                                                     //
// //  Modulename :  Prediction_History_Table.v                           //
// //                                                                     //
// //  Description :  Prediction History Table that stores the            //
// //      Prediction History, according to the bht get by branch         //
// //      history table, update/read the Prediction History Table        //
// //                                                                     //
// /////////////////////////////////////////////////////////////////////////

// module Prediction_History_Table #(
//     parameter PHT_INDEX = $clog2(`PHT_SIZE)
// ) (
//     input                           clock, reset,

//     //From Excute Stage, update the prediction
//     input  [1:0]                    br_en,
//     input  [1:0] [`XLEN-1:0]        ex_pc,
//     input  [1:0]                    take_branch,

//     //From IF, predict the npc
//     input  [1:0] [`XLEN-1:0]        if_pc,

//     //From BHT, predict the npc
//     input  [1:0] [`BHT_WIDTH-1:0]   bht2pht_if,  
//     input  [1:0] [`BHT_WIDTH-1:0]   bht2pht_ex,//3

//     //To multiplexer, selecting the npc, predict taken or not
//     output logic [1:0]              predict_taken
// );
//     PHT_STATE  state [`PHT_SIZE-1:0] [`H_SIZE-1:0];//2 256 3
//     PHT_STATE  n_state [`PHT_SIZE-1:0] [`H_SIZE-1:0];

//     logic [PHT_INDEX-1:0] wptr [1:0];    // write pointer for updating
//     logic [PHT_INDEX-1:0] rptr [1:0];    // read pointer for predict
    
//     always_comb begin
//         for (int i=0;i<2;i++) begin
//             wptr[i] = ex_pc[i][2 +: PHT_INDEX];//[2:9]
//             rptr[i] = if_pc[i][2 +: PHT_INDEX];
//         end
//     end
    
//     always_comb begin
//         for (int j=0;j<`PHT_SIZE;j++) begin
//             for (int m=0;m<`H_SIZE;m++) begin
//                 n_state[j][m] = state[j][m];
//             end
//         end
//         case (state[wptr[1]][bht2pht_ex[1]])
//             NT_STRONG: n_state[wptr[1]][bht2pht_ex[1]] = take_branch[1] ? NT_WEAK : NT_STRONG;
//             NT_WEAK:   n_state[wptr[1]][bht2pht_ex[1]] = take_branch[1] ? T_STRONG  : NT_STRONG;
//             T_WEAK:    n_state[wptr[1]][bht2pht_ex[1]] = take_branch[1] ? T_STRONG : NT_STRONG;
//             T_STRONG:  n_state[wptr[1]][bht2pht_ex[1]] = take_branch[1] ? T_STRONG : T_WEAK;
//         endcase
//         //if wptr[0] == wptr[1], we will record pc[0](taken/notake)
//         case (state[wptr[0]][bht2pht_ex[0]])
//             NT_STRONG: n_state[wptr[0]][bht2pht_ex[0]] = take_branch[0] ? NT_WEAK : NT_STRONG;
//             NT_WEAK:   n_state[wptr[0]][bht2pht_ex[0]] = take_branch[0] ? T_STRONG  : NT_STRONG;
//             T_WEAK:    n_state[wptr[0]][bht2pht_ex[0]] = take_branch[0] ? T_STRONG : NT_STRONG;
//             T_STRONG:  n_state[wptr[0]][bht2pht_ex[0]] = take_branch[0] ? T_STRONG : T_WEAK;
//         endcase
//     end

//     // synopsys sync_set_reset "reset"
//     always_ff @(posedge clock) begin
//         if (reset) begin
//             for (int k=0;k<`PHT_SIZE;k++) begin
//                 for (int n=0;n<`H_SIZE;n++) begin
//                     state[k][n] <= `SD  NT_WEAK;
//                 end
//             end
//         end
//         else if (br_en[0] | br_en[1]) begin
//             if (br_en[0] && br_en[1] && (wptr[0] == wptr[1])) begin
//                 state [wptr[0]][bht2pht_ex[0]] <= `SD n_state[wptr[0]][bht2pht_ex[0]];
//             end else begin
//                 if (br_en[0]) begin
//                     state [ wptr[0]][bht2pht_ex[0]] <= `SD n_state[wptr[0]][bht2pht_ex[0]];
//                 end
//                 if (br_en[1])begin
//                     state [ wptr[1]][bht2pht_ex[1]] <= `SD n_state[wptr[1]][bht2pht_ex[1]];
//                 end
//             end
//         end
//     end

    
//     always_comb begin
//         for (int n=0;n<2;n++) begin
//             predict_taken[n] = ((state[rptr[n]][bht2pht_if[n]]== T_WEAK) | (state[rptr[n]][bht2pht_if[n]]== T_STRONG)) ? 1 : 0;
//         end 
//     end

// endmodule
