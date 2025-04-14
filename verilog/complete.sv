`include "verilog/sys_defs.svh"
module complete (

    input EX_PACKET ex_reg,

    output CDB_PACKET cdb_packet,
    output logic             wb_regfile_en,  // register write enable
    output logic [4:0]       wb_regfile_idx, // register write index
    output logic [`XLEN-1:0] wb_regfile_data // register write data
);
    logic [1:0] select;

    always_comb begin
        case (select)
            default: begin
                cdb_packet.Value        = wb_regfile_data; 
	            cdb_packet.NPC          = ex_reg.NPC;     
	            cdb_packet.take_branch  = ex_reg.take_branch;
                //cdb_packet.inst = ex_reg.inst;		
	            cdb_packet.dest_reg_idx = ex_reg.dest_reg_idx;
	            cdb_packet.halt         = 0;
                cdb_packet.illegal      = 0;; 
	            cdb_packet.done         = 0;
	            cdb_packet.valid        = wb_regfile_en;
	            cdb_packet.Tag          = ex_reg.Tag;
            end
        endcase
        
    end
    // This enable computation is sort of overkill since the reg file
    // also handles the `ZERO_REG case, but there's no harm in putting this here
    // the valid check is also somewhat redundant
    assign wb_regfile_en = ex_reg.valid && (ex_reg.dest_reg_idx != `ZERO_REG);

    assign wb_regfile_idx = ex_reg.dest_reg_idx;

    // Select register writeback data:
    // ALU/MEM result, unless taken branch, in which case we write
    // back the old NPC as the return address. Note that ALL branches
    // and jumps write back the 'link' value, but those that don't
    // use it specify ZERO_REG as the destination.
    assign wb_regfile_data = (ex_reg.take_branch) ? ex_reg.NPC : ex_reg.alu_result;

endmodule // stage_wb
