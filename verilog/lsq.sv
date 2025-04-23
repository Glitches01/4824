`include "verilog/sys_defs.svh"

module lsq
#(
    parameter DEPTH = 8
)
(
    input  logic                        clock,
    input  logic                        reset,

    input  DP_LSQ_PACKET                dp_lsq_packet,
    input  logic                        enable,
    output logic                        lsq_available,
    output [$clog2(DEPTH)-1:0]          lsq_idx,

//from RS/Dispatch
    input  EX_LSQ_PACKET                ex_lsq_packet,
    
//to CDB
    output CDB_PACKET                   lsq2cdb,
//with dcache
    output DCACHE_IN_PACKET             lsq2dcache_packet,
    input  DCACHE_OUT_PACKET            dcache2lsq_packet
);

  //localparam int RBTAG_W = $clog2(`ROB_SIZE);
  LSQ_ENTRY lsq[DEPTH], lsq_n[DEPTH];
  logic [$clog2(DEPTH):0]     lsq_head, lsq_head_n;
  logic [$clog2(DEPTH):0]     lsq_tail, lsq_tail_n;

  wire [$clog2(DEPTH)-1:0] lsq_head_idx = lsq_head[$clog2(DEPTH)-1:0];
  wire [$clog2(DEPTH)-1:0] lsq_tail_idx = lsq_tail[$clog2(DEPTH)-1:0];

  assign lsq_idx = lsq_tail_idx;

  always_comb begin
    for (int i = 0; i < DEPTH; i++) begin
      lsq_n[i] = lsq[i];
    end
    lsq_tail_n = lsq_tail;
    lsq_head_n = lsq_head;

    if (enable) begin
      lsq_n[lsq_tail_idx].valid        = 1'b0;
      lsq_n[lsq_tail_idx].PC           = dp_lsq_packet.PC;
      lsq_n[lsq_tail_idx].NPC          = dp_lsq_packet.NPC;
      lsq_n[lsq_tail_idx].addr         = 32'h0;
      lsq_n[lsq_tail_idx].data         = 32'h0;
      lsq_n[lsq_tail_idx].mem_size     = 0;
      lsq_n[lsq_tail_idx].is_store     = dp_lsq_packet.wr_mem;
      lsq_n[lsq_tail_idx].Tag          = dp_lsq_packet.Tag;
      lsq_n[lsq_tail_idx].inst         = dp_lsq_packet.inst;
      lsq_n[lsq_tail_idx].dest_reg_idx = dp_lsq_packet.dest_reg_idx;

      lsq_tail_n  = lsq_tail + 1;
    end

    if (ex_lsq_packet.valid && (ex_lsq_packet.PC == lsq_n[ex_lsq_packet.lsq_idx].PC)) begin
      lsq_n[ex_lsq_packet.lsq_idx].valid        = 1'b1;
      lsq_n[ex_lsq_packet.lsq_idx].addr         = ex_lsq_packet.addr;
      lsq_n[ex_lsq_packet.lsq_idx].data         = ex_lsq_packet.data;
      lsq_n[ex_lsq_packet.lsq_idx].mem_size     = ex_lsq_packet.mem_size;
    end

    if (lsq[lsq_head_idx].valid && dcache2lsq_packet.completed) begin
      lsq_n[lsq_head_idx].valid = 1'b0;
      lsq_head_n  = lsq_head + 1;
    end
  end

  logic [$clog2(DEPTH)-1:0] lsq_next_tail;
  assign lsq_next_tail = (lsq_tail_idx == DEPTH-1) ? 0 : lsq_tail_idx + 1;
  always_comb begin
      if (lsq_next_tail == lsq_head_idx)
          lsq_available = 1'b0;
      else
          lsq_available = 1'b1;
  end


  always_ff @(posedge clock) begin
    if (reset) begin
      lsq_head  <= 0;
      lsq_tail  <= 0;
      for (int i = 0; i < DEPTH; i++)
        lsq[i] <= 1'b0;
        
    end else begin
      lsq_head  <= lsq_head_n;
      lsq_tail  <= lsq_tail_n;
      lsq       <= lsq_n;
    end
  end



  assign lsq2dcache_packet.address           = lsq[lsq_head_idx].addr;
  assign lsq2dcache_packet.value             = lsq[lsq_head_idx].data;
  assign lsq2dcache_packet.mem_size          = lsq[lsq_head_idx].mem_size;
  assign lsq2dcache_packet.is_store          = lsq[lsq_head_idx].is_store;
  assign lsq2dcache_packet.lsq_is_requesting = lsq[lsq_head_idx].valid;//when 1, send load/store to dcache

always_ff @(posedge clock) begin
  if (reset) begin
    lsq2cdb <= '{valid: 0, Tag: 0, Value: 0, alu_result:0, inst: '0, PC:0, NPC:0, take_branch:0, dest_reg_idx:0, halt:0, illegal:0, done:0};
  end else begin
    //lsq2cdb <= '{valid: 0, Tag: 0, Value: 0, alu_result:0, inst: '0, PC:0, NPC:0, take_branch:0, dest_reg_idx:0, halt:0, illegal:0, done:0};
    if (lsq[lsq_head_idx].valid && dcache2lsq_packet.completed) begin
      lsq2cdb.valid        <= 1'b1;//TODO
      lsq2cdb.PC           <= lsq[lsq_head_idx].PC;
      lsq2cdb.NPC          <= lsq[lsq_head_idx].NPC;

      lsq2cdb.Value        <= dcache2lsq_packet.value;
      lsq2cdb.Tag          <= lsq[lsq_head_idx].Tag;
      lsq2cdb.inst         <= lsq[lsq_head_idx].inst;
      lsq2cdb.dest_reg_idx <= lsq[lsq_head_idx].dest_reg_idx;
    end else begin
      lsq2cdb.valid        <= 1'b0;//TODO
    end
  end
end
endmodule

