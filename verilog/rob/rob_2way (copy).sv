module ROB(
input clock,
input reset,
input CDB_PACKET [1:0] CDB_packet_in,
input DP_PACKET [1:0] DP_packet_in,
input MT_ROB_PACKET [1:0] MT_ROB_packet_in,

output CP_RT_PACKET [1:0] CP_RT_packet_out,
output ROB_RS_PACKET [1:0] ROB_RS packet_out,
output ROB_MT_PACKET [1:0] ROB_MT_packet_out,

);

logic [$clog2(`ROB_SIZE):0] head, head_plus1, tail, tail_plus1, head_nc, tail,nc;
assign head_plus1 = head + 1;
assign tail_plus1 = tail + 1;

ROB_ENTRY ROB_content [`ROB_SIZE-1:0];
ROB_ENTRY ROB_content_n [`ROB_SIZE-1:0];

logic [$clog2(`ROB_SIZE)-1:0] space_available;

always_comb begin
	for (integer unsigned j = 0; j < `ROB_SIZE; j++) begin
		ROB_content_n[j] = ROB_content[j];
	end
	case ({DP_packet_in[1], DP_packet_in[0]}) begin
	2'b00: begin
		ROB_content_n.reg_idx = RoB_content.reg_idx;
		RoB_content_n.valid = RoB_content.valid;


	       end
	2'b01:
	2'b11:
	endcase
