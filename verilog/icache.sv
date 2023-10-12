//////////////////////////////////////////////////////////////////////////
//                                                                      //
//   Modulename :  icache.sv                                            //
//                                                                      //
//  Description :  the instruction cache module that reroutes memory    //
//                 accesses to decrease misses                          //
//                                                                      //
//////////////////////////////////////////////////////////////////////////

`ifndef __ICACHE_SV__
`define __ICACHE_SV__

`include "sys_defs.svh"

// internal macros, no other file should need these
`define CACHE_LINES 32
`define CACHE_LINE_BITS $clog2(`CACHE_LINES)

typedef struct packed {
	logic [63:0]                  data;
	// 12:0 (13 bits) since only 16 bits of address exist in mem - and 3 are the block offset
	logic [12-`CACHE_LINE_BITS:0] tags;
	logic                         valid;
} ICACHE_PACKET;

// a quick README copied from mem.sv:
// We've increased the memory latency from 1 cycle to 100ns. which will be multiple cycles for any
// reasonable processor. Thus, memory can have multiple transactions pending, and coordinates them
// via memory tags (different meaning than cache tags) that represent a transaction it's working on.
// Memory tags are 4 bits long since 15 mem accesses can be live at one time, and only one access happens per cycle.
// the 0 tag is a sentinel value and unused
// it would be very difficult to push your clock period past 100ns/15=6.66ns, so 15 is sufficient
// Upon a request, memory *responds* with the tag it will use for that request
// then ceiling(100ns/clock period) cycles later, it will return the data with the corresponding tag

// this cache's job is to coordinate those tags and speed up this process when data is reused

// note that this cache is blocking, and will wait on one memory request before sending another
// (unless the input address changes, in which case it abandons that request)
// implementing a non-blocking cache can count towards advanced feature points
// but will require careful management of memory tags

module icache (
	input clock,
	input reset,

	// from memory
	input [3:0]  Imem2proc_response, // this should be zero unless we got a response
	input [63:0] Imem2proc_data,
	input [3:0]  Imem2proc_tag,

	// from fetch stage
	input [`XLEN-1:0] proc2Icache_addr,

	// to memory
	output logic [1:0]       proc2Imem_command,
	output logic [`XLEN-1:0] proc2Imem_addr,

	// to fetch stage
	output logic [63:0] Icache_data_out, // value is memory[proc2Icache_addr]
	output logic        Icache_valid_out // when this is high
);

	ICACHE_PACKET [`CACHE_LINES-1:0] icache_data;

	// note: cache tags, not memory tags
	logic [`CACHE_LINE_BITS - 1:0] current_index, last_index;
	logic [12-`CACHE_LINE_BITS:0] current_tag, last_tag;

	assign {current_tag, current_index} = proc2Icache_addr[15:3];

	assign Icache_data_out = icache_data[current_index].data;
	assign Icache_valid_out = icache_data[current_index].valid && (icache_data[current_index].tags == current_tag);

	logic [3:0] current_mem_tag; // the current memory tag we might be waiting on
	logic miss_outstanding; // whether a miss has received its response tag to wait on

	logic got_mem_data;
	assign got_mem_data = (current_mem_tag == Imem2proc_tag) && (current_mem_tag != 0);

	logic changed_addr;
	assign changed_addr = (current_index != last_index) || (current_tag != last_tag);

	// should set to zero if we changed_addr, but will keep resetting while we have a miss_outstanding
	// and will set to zero when we got_mem_data
	// (this is since Imem2proc_response should be zero if no request)
	logic update_mem_tag;
	assign update_mem_tag = changed_addr || miss_outstanding || got_mem_data;

	logic unanswered_miss; // if we have a new miss or still waiting for the response tag
	// we might need to wait for the response tag because dcache has priority over icache
	assign unanswered_miss = changed_addr ? !Icache_valid_out
	                                      : miss_outstanding && (Imem2proc_response == 0);

	// keep sending memory requests until we receive a response tag or change addresses
	assign proc2Imem_command = (miss_outstanding && !changed_addr) ? BUS_LOAD : BUS_NONE;
	assign proc2Imem_addr    = {proc2Icache_addr[31:3],3'b0};

	// synopsys sync_set_reset "reset"
	always_ff @(posedge clock) begin
		if (reset) begin
			last_index       <= `SD -1; // These are -1 to get ball rolling when
			last_tag         <= `SD -1; // reset goes low because addr "changes"
			current_mem_tag  <= `SD 0;
			miss_outstanding <= `SD 0;
			icache_data      <= `SD 0; // set all cache data to 0 (including valid bits)
		end else begin
			last_index       <= `SD current_index;
			last_tag         <= `SD current_tag;
			miss_outstanding <= `SD unanswered_miss;
			if (update_mem_tag) begin
				current_mem_tag <= `SD Imem2proc_response;
			end
			if (got_mem_data) begin // If data came from memory, meaning tag matches
				icache_data[current_index].data   <= `SD Imem2proc_data;
				icache_data[current_index].tags   <= `SD current_tag;
				icache_data[current_index].valid <= `SD 1;
			end
		end
	end

endmodule // module icache

`endif // __ICACHE_SV__
