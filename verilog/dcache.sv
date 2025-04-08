
// 2-way, write-back + write-allocate
// BLOCK_SIZE = 64 bits (8 bytes)
// CACHE_SIZE = 256 bytes

`timescale 1ns/100ps


module dcache(
    //system signal
    input                       clock, reset,

    //from LSQ
    input LSQ2DCACHE_PACKET     lsq2dcache_packet,

    //from MEMORY
    input [3:0]                 Dmem2proc_response,
    input [63:0]                Dmem2proc_data,
    input [3:0]                 Dmem2proc_tag,

    //to LSQ (and ROB)
    output DCACHE2LSQ_PACKET    dcache2lsq_packet,

    //to testbench and cache set
    output DCACHE_SET [15:0]    cache_data,

    //to MEMORY
    output logic [`XLEN-1:0]    proc2Dmem_addr,
    output logic [63:0]         proc2Dmem_data,
    output BUS_COMMAND          proc2Dmem_command
);

    // typedef enum logic [2:0] { 
    //     DCACHE_IDLE_HIT = 3'h0,
    //     DCACHE_ST_EVICT = 3'h1,
    //     DCACHE_LD_EVICT = 3'h2,
    //     DCACHE_ST_WAIT = 3'h3,
    //     DCACHE_LD_WAIT = 3'h4
    // } DCACHE_STATE;
    ///////////////////////////////////////////////////////////////////
    //  state: fsm: DCACHE_IDLE_HIT -> DCACHE_ST_EVICT/DCACHE_LD_EVICT -> DCACHE_ST_WAIT/DCACHE_LD_WAIT
    //  current_set: current cache set lsq is accessing
    //  n_cache_data: cache set that will be updated
    ///////////////////////////////////////////////////////////////////
    DCACHE_STATE        state;
    DCACHE_SET          current_set;
    DCACHE_SET [15:0]   n_cache_data;

    ///////////////////////////////////////////////////////////////////
    //  address_valid: the accessed address by lsq is valid
    //  request_valid: valid request
    //  waiting_mem_tag: the tag dcache should be expecting.
    ///////////////////////////////////////////////////////////////////
    logic               address_valid;
    logic               request_valid;
    logic [3:0]         waiting_mem_tag;

    assign address_valid        = (lsq2dcache_packet.address < `MEM_SIZE_IN_BYTES);
    assign request_valid        = (lsq2dcache_packet.lsq_is_requesting) && address_valid;


    ///////////////////////////////////////////////////////////////////
    //  current_addr_tag: decoded tag sent by lsq
    //  current_addr_index: decoded index sent by lsq
    //  current_addr_offset: decoded offset sent by lsq
    ///////////////////////////////////////////////////////////////////
    logic [24:0]    current_addr_tag;
    logic [3:0]     current_addr_index;
    logic [2:0]     current_addr_offset;
    assign current_addr_tag     = lsq2dcache_packet.address[31:7];
    assign current_addr_index   = lsq2dcache_packet.address[6:3];
    assign current_addr_offset  = lsq2dcache_packet.address[2:0];

    ///////////////////////////////////////////////////////////////////
    //  
    //  current_set: current cache set lsq is accessing
    //  
    ///////////////////////////////////////////////////////////////////

    assign current_set = cache_data[current_addr_index];

    ///////////////////////////////////////////////////////////////////
    //  line0_hit: hit line 0 by comparing tag
    //  line1_hit: hit line 1
    //  miss: miss at line0 & line 1
    ///////////////////////////////////////////////////////////////////
    logic line0_hit, line1_hit;
    logic miss;
    assign line0_hit = (current_set.line[0].tag == current_addr_tag) & current_set.line[0].valid;
    assign line1_hit = (current_set.line[1].tag == current_addr_tag) & current_set.line[1].valid;
    assign miss = (~line0_hit & ~line1_hit) & request_valid;

    assign dcache2lsq_packet.valid = (!address_valid) || (state == DCACHE_IDLE_HIT) && (request_valid) && (~miss);

    logic [31:0]    lsq2dcache_word;
    logic [15:0]    lsq2dcache_halfword;
    logic [7:0]     lsq2dcache_byte;
    assign lsq2dcache_word      = lsq2dcache_packet.value;
    assign lsq2dcache_halfword  = lsq2dcache_packet.value[15:0];
    assign lsq2dcache_byte      = lsq2dcache_packet.value[7:0];

    logic evict_line0, evict_line1;
    assign evict_line0 = miss & (current_set.last_accessed) & current_set.line[0].valid;
    assign evict_line1 = miss & (~current_set.last_accessed) & current_set.line[0].valid;


    logic current_line_idx;

    always_comb begin
        if (line0_hit)
            current_line_idx = 1'b0;
        else if (line1_hit)
            current_line_idx = 1'b1;
        else begin // miss case, find a new line
            current_line_idx = ~current_set.last_accessed;
        end
    end

    ///////////////////////////////////////////////////////////////////
    //  send request to Data MEM
    ///////////////////////////////////////////////////////////////////
    always_comb begin
        proc2Dmem_command = BUS_NONE;
        proc2Dmem_addr = 0;
        proc2Dmem_data = 32'hB16B00B5;
        if ((state == DCACHE_IDLE_HIT) && miss) begin
            proc2Dmem_command = BUS_LOAD;
            proc2Dmem_addr = {current_addr_tag, current_addr_index, 3'b0};
        end else if ((state == DCACHE_LD_EVICT) || (state == DCACHE_ST_EVICT)) begin
            if (evict_line0 || evict_line1) begin
                proc2Dmem_command = BUS_STORE;
                proc2Dmem_addr = {current_set.line[current_line_idx].tag, current_addr_index, 3'b0};
                proc2Dmem_data = current_set.line[current_line_idx].data;
            end
        end
    end

    ///////////////////////////////////////////////////////////////////
    //  Update the cache line during DCACHE_IDLE_HIT. DCACHE_LD_WAIT, DCACHE_ST_WAIT
    ///////////////////////////////////////////////////////////////////
    always_comb begin
        n_cache_data = cache_data;
        if (((state == DCACHE_LD_WAIT) || (state == DCACHE_ST_WAIT)) && request_valid) begin
            if ((waiting_mem_tag == Dmem2proc_tag) && (waiting_mem_tag != 3'b0)) begin
                n_cache_data[current_addr_index].line[current_line_idx].data = Dmem2proc_data;
                n_cache_data[current_addr_index].line[current_line_idx].valid = 1'b1;
                n_cache_data[current_addr_index].line[current_line_idx].tag = current_addr_tag;
            end
        end
        if (((state == DCACHE_ST_WAIT) && ((waiting_mem_tag == Dmem2proc_tag) && (waiting_mem_tag != 3'b0)))
        || ((state == DCACHE_IDLE_HIT) && ~miss && lsq2dcache_packet.is_store)) begin
            if (lsq2dcache_packet.mem_size[1:0] == 2'b10) begin 
                case (current_addr_offset[2])
                    1'b1: n_cache_data[current_addr_index].line[current_line_idx].data[63:32] = lsq2dcache_packet.value;
                    1'b0: n_cache_data[current_addr_index].line[current_line_idx].data[31:0] = lsq2dcache_packet.value;
                endcase
            end else if (lsq2dcache_packet.mem_size[1:0] == 2'b01) begin
                case (current_addr_offset[2:1])
                    2'b11: n_cache_data[current_addr_index].line[current_line_idx].data[63:48] = lsq2dcache_halfword;
                    2'b10: n_cache_data[current_addr_index].line[current_line_idx].data[47:32] = lsq2dcache_halfword;
                    2'b01: n_cache_data[current_addr_index].line[current_line_idx].data[31:16] = lsq2dcache_halfword;
                    2'b00: n_cache_data[current_addr_index].line[current_line_idx].data[15:0] = lsq2dcache_halfword;
                endcase
            end else if (lsq2dcache_packet.mem_size[1:0] == 2'b00) begin 
                case (current_addr_offset)
                    3'b111: n_cache_data[current_addr_index].line[current_line_idx].data[63:56] = lsq2dcache_byte;
                    3'b110: n_cache_data[current_addr_index].line[current_line_idx].data[55:48] = lsq2dcache_byte;
                    3'b101: n_cache_data[current_addr_index].line[current_line_idx].data[47:40] = lsq2dcache_byte;
                    3'b100: n_cache_data[current_addr_index].line[current_line_idx].data[39:32] = lsq2dcache_byte;
                    3'b011: n_cache_data[current_addr_index].line[current_line_idx].data[31:24] = lsq2dcache_byte;
                    3'b010: n_cache_data[current_addr_index].line[current_line_idx].data[23:16] = lsq2dcache_byte;
                    3'b001: n_cache_data[current_addr_index].line[current_line_idx].data[15:8] = lsq2dcache_byte;
                    3'b000: n_cache_data[current_addr_index].line[current_line_idx].data[7:0] = lsq2dcache_byte;
                endcase
            end
        end
        if ((state == DCACHE_IDLE_HIT) && ~miss && request_valid) begin
            n_cache_data[current_addr_index].last_accessed = current_line_idx;
        end
    end
    
    ////////////////////////////////////////////////////////////////////////
    //  send data to lsq
    //  Controls the line and exact byte it is accessing based on lsq2dcache_packet.mem_size and current_addr_offset
    ////////////////////////////////////////////////////////////////////////
    always_comb begin
        if (current_addr_offset[2] == 1'b1) begin
            dcache2lsq_packet.value = current_set.line[current_line_idx].data[63:32];
        end else begin
            dcache2lsq_packet.value = current_set.line[current_line_idx].data[31:0];
        end
        case (lsq2dcache_packet.mem_size[1:0])
            2'b01 : begin
                dcache2lsq_packet.value >>= current_addr_offset[1] * 16;
                dcache2lsq_packet.value[31:16] = 16'b0;
            end
            2'b00 : begin
                dcache2lsq_packet.value >>= current_addr_offset[1:0] * 8;
                dcache2lsq_packet.value[31:8] = 24'b0;
            end
            default : dcache2lsq_packet.value = dcache2lsq_packet.value;
        endcase
    end

    ////////////////////////////////////////////////////////////////////////
    //  FSM:
    //  DCACHE_IDLE_HIT if(miss) then-> DCACHE_ST_EVICT/DCACHE_LD_EVICT if(tag = tag) then -> DCACHE_ST_WAIT/DCACHE_LD_WAIT
    ////////////////////////////////////////////////////////////////////////
    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            for (int i = 0; i < 16; i++) begin
                cache_data[i].line[0].valid <= `SD 1'b0;
                cache_data[i].line[1].valid <= `SD 1'b0;
                cache_data[i].last_accessed <= `SD 1'b0;
            end
            state <= `SD DCACHE_IDLE_HIT;
            waiting_mem_tag <= `SD 4'b0;
        end else begin
            cache_data <= `SD n_cache_data;
            // State transitions
            if ((state == DCACHE_IDLE_HIT) && miss) begin
                waiting_mem_tag <= `SD Dmem2proc_response;
                if (lsq2dcache_packet.is_store) begin
                    state <= `SD DCACHE_ST_EVICT;
                end else begin
                    state <= `SD DCACHE_LD_EVICT;
                end
            end else if (state == DCACHE_LD_EVICT) begin
                if (request_valid)
                    state <= `SD DCACHE_LD_WAIT;
                else begin
                    state <= `SD DCACHE_IDLE_HIT;
                    waiting_mem_tag <= `SD 4'b0;
                end
            end else if (state == DCACHE_ST_EVICT) begin
                if (request_valid)
                    state <= `SD DCACHE_ST_WAIT;
                else begin
                    state <= `SD DCACHE_IDLE_HIT;
                    waiting_mem_tag <= `SD 4'b0;
                end
            end else if (state == DCACHE_LD_WAIT) begin
                if (~request_valid || ((waiting_mem_tag == Dmem2proc_tag) && (waiting_mem_tag != 3'b0))) begin
                    state <= `SD DCACHE_IDLE_HIT;
                    waiting_mem_tag <= `SD 4'b0;
                end
            end else if (state == DCACHE_ST_WAIT) begin
                if (~request_valid || ((waiting_mem_tag == Dmem2proc_tag) && (waiting_mem_tag != 3'b0))) begin
                    state <= `SD DCACHE_IDLE_HIT;
                    waiting_mem_tag <= `SD 4'b0;
                end
            end
        end
    end

endmodule
