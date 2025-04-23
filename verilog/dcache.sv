`timescale 1ns/1ps

module dcache(
    input clock, reset,

    input DCACHE_IN_PACKET              dcache_in,                    // input from LSQ
    
    // inputs from mem
    input [3:0]                         Dmem2proc_resp,                           // memory response status/ack                
    input [63:0]                        Dmem2proc_data,                          // memory response data
    input [3:0]                         Dmem2proc_tag,                            // response tag used for tracking requests

    output DCACHE_OUT_PACKET            dcache2lsq_packet,           // output to LSQ (and ROB)
    
    output DCACHE_DATASET [15:0]        dcache,                      // dcache
    
    // outputs to mem
    output logic [`XLEN-1:0]            proc2Dmem_addr,               // address of memory request
    output logic [63:0]                 proc2Dmem_data,                    // data being written to memory
    output BUS_COMMAND                  proc2Dmem_cmd                       // command to memory: LOAD / STORE / NONE
);

    DCACHE_STATE state;
    DCACHE_DATASET [15:0] n_cache_data;
    DCACHE_DATASET current_set;
   
    
    ///////////////////////////////////////////////////////////////////
    //  address_valid: the accessed address by lsq is valid
    //  request_valid: valid request
    //  waiting_mem_tag: the tag dcache should be expecting.
    ///////////////////////////////////////////////////////////////////
    logic address_valid;
    logic real_request; // 1 if dcache_in is really an effictive request (not)
    logic [3:0] waiting_mem_tag; // the tag dcache should be expecting.
    
    assign address_valid = (dcache_in.address < `MEM_SIZE_IN_BYTES);
    assign real_request  = (dcache_in.lsq_is_requesting) && address_valid;

    
    ///////////////////////////////////////////////////////////////////
    //  current_addr_tag: decoded tag sent by lsq
    //  current_addr_index: decoded index sent by lsq
    //  current_addr_offset: decoded offset sent by lsq
    ///////////////////////////////////////////////////////////////////
    logic [24:0] current_addr_tag;
    logic [3:0]  current_addr_index;
    logic [2:0]  current_addr_offset;
    
    assign current_addr_tag    = dcache_in.address[31:7];
    assign current_addr_index  = dcache_in.address[6:3];
    assign current_addr_offset = dcache_in.address[2:0];

    ///////////////////////////////////////////////////////////////////
    //  
    //  current_set: current cache set lsq is accessing
    //  
    ///////////////////////////////////////////////////////////////////
    assign current_set = dcache[current_addr_index];
    
    ///////////////////////////////////////////////////////////////////
    //  line0_hit: hit line 0 by comparing tag
    //  line1_hit: hit line 1
    //  miss: miss at line0 & line 1
    ///////////////////////////////////////////////////////////////////
    logic line_0_hit, line_1_hit;
    logic miss;
    assign line_0_hit = (current_set.line[0].tag == current_addr_tag) & current_set.line[0].valid;
    assign line_1_hit = (current_set.line[1].tag == current_addr_tag) & current_set.line[1].valid;
    assign miss = (~line_0_hit & ~line_1_hit) & real_request;

    assign dcache2lsq_packet.completed = (!address_valid) || (state == DCACHE_IDLE_HIT) && (real_request) && (~miss);

    logic [31:0] dcache_in_value_word;
    logic [15:0] dcache_in_value_half;
    logic [7:0] dcache_in_value_byte;
    assign dcache_in_value_word = dcache_in.value;
    assign dcache_in_value_half = dcache_in.value[15:0];
    assign dcache_in_value_byte = dcache_in.value[7:0];

    logic evict_line0, evict_line1;
    assign evict_line0 = miss & (current_set.last_accessed)  & current_set.line[0].valid;
    assign evict_line1 = miss & (~current_set.last_accessed) & current_set.line[0].valid;


    logic current_line_idx;

    always_comb begin
        if (line_0_hit)
            current_line_idx = 1'b0;
        else if (line_1_hit)
            current_line_idx = 1'b1;
        else begin // miss case, find a new line
            current_line_idx = ~current_set.last_accessed;
        end
    end

    ///////////////////////////////////////////////////////////////////
    //  send request to Data MEM
    ///////////////////////////////////////////////////////////////////
    always_comb begin
        proc2Dmem_cmd = BUS_NONE;
        proc2Dmem_addr = 0;
        proc2Dmem_data = 32'hB16B00B5;
        if ((state == DCACHE_IDLE_HIT) && miss) begin
            proc2Dmem_cmd = BUS_LOAD;
            proc2Dmem_addr = {current_addr_tag, current_addr_index, 3'b0};
        end else if ((state == DCACHE_LD_EVICT) || (state == DCACHE_ST_EVICT)) begin
            if (evict_line0 || evict_line1) begin
                proc2Dmem_cmd = BUS_STORE;
                proc2Dmem_addr = {current_set.line[current_line_idx].tag, current_addr_index, 3'b0};
                proc2Dmem_data = current_set.line[current_line_idx].data;
            end
        end
    end

    ///////////////////////////////////////////////////////////////////
    //  Update the cache line during DCACHE_IDLE_HIT. DCACHE_LD_WAIT, DCACHE_ST_WAIT
    ///////////////////////////////////////////////////////////////////
    always_comb begin
        n_cache_data = dcache;
        if (((state == DCACHE_LD_WAIT) || (state == DCACHE_ST_WAIT)) && real_request) begin
            if ((waiting_mem_tag == Dmem2proc_tag) && (waiting_mem_tag != 3'b0)) begin
                n_cache_data[current_addr_index].line[current_line_idx].data  = Dmem2proc_data;
                n_cache_data[current_addr_index].line[current_line_idx].valid = 1'b1;
                n_cache_data[current_addr_index].line[current_line_idx].tag   = current_addr_tag;
            end
        end
        if (((state == DCACHE_ST_WAIT) && ((waiting_mem_tag == Dmem2proc_tag) && (waiting_mem_tag != 3'b0)))
        || ((state == DCACHE_IDLE_HIT) && ~miss && dcache_in.is_store)) begin
            if (dcache_in.mem_size[1:0] == 2'b10) begin 
                case (current_addr_offset[2])
                    1'b1: n_cache_data[current_addr_index].line[current_line_idx].data[63:32] = dcache_in.value;
                    1'b0: n_cache_data[current_addr_index].line[current_line_idx].data[31:0]  = dcache_in.value;
                endcase
            end else if (dcache_in.mem_size[1:0] == 2'b01) begin
                case (current_addr_offset[2:1])
                    2'b11: n_cache_data[current_addr_index].line[current_line_idx].data[63:48] = dcache_in_value_half;
                    2'b10: n_cache_data[current_addr_index].line[current_line_idx].data[47:32] = dcache_in_value_half;
                    2'b01: n_cache_data[current_addr_index].line[current_line_idx].data[31:16] = dcache_in_value_half;
                    2'b00: n_cache_data[current_addr_index].line[current_line_idx].data[15:0]  = dcache_in_value_half;
                endcase
            end else if (dcache_in.mem_size[1:0] == 2'b00) begin 
                case (current_addr_offset)
                    3'b111: n_cache_data[current_addr_index].line[current_line_idx].data[63:56] = dcache_in_value_byte;
                    3'b110: n_cache_data[current_addr_index].line[current_line_idx].data[55:48] = dcache_in_value_byte;
                    3'b101: n_cache_data[current_addr_index].line[current_line_idx].data[47:40] = dcache_in_value_byte;
                    3'b100: n_cache_data[current_addr_index].line[current_line_idx].data[39:32] = dcache_in_value_byte;
                    3'b011: n_cache_data[current_addr_index].line[current_line_idx].data[31:24] = dcache_in_value_byte;
                    3'b010: n_cache_data[current_addr_index].line[current_line_idx].data[23:16] = dcache_in_value_byte;
                    3'b001: n_cache_data[current_addr_index].line[current_line_idx].data[15:8]  = dcache_in_value_byte;
                    3'b000: n_cache_data[current_addr_index].line[current_line_idx].data[7:0]   = dcache_in_value_byte;
                endcase
            end
        end
        if ((state == DCACHE_IDLE_HIT) && ~miss && real_request) begin
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
        case (dcache_in.mem_size[1:0])
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
    always_ff @(posedge clock) begin
        if (reset) begin
            for (int i = 0; i < 16; i++) begin
                dcache[i].line[0].valid <= 1'b0;
                dcache[i].line[1].valid <= 1'b0;
                dcache[i].last_accessed <= 1'b0;
            end
            state <= DCACHE_IDLE_HIT;
            waiting_mem_tag <= 4'b0;
        end else begin
            dcache <= n_cache_data;
            // State transitions
            if ((state == DCACHE_IDLE_HIT) && miss) begin
                waiting_mem_tag <= Dmem2proc_resp;
                if (dcache_in.is_store) begin
                    state <= DCACHE_ST_EVICT;
                end else begin
                    state <= DCACHE_LD_EVICT;
                end
            end else if (state == DCACHE_LD_EVICT) begin
                if (real_request)
                    state <= DCACHE_LD_WAIT;
                else begin
                    state <= DCACHE_IDLE_HIT;
                    waiting_mem_tag <= 4'b0;
                end
            end else if (state == DCACHE_ST_EVICT) begin
                if (real_request)
                    state <= DCACHE_ST_WAIT;
                else begin
                    state <= DCACHE_IDLE_HIT;
                    waiting_mem_tag <= 4'b0;
                end
            end else if (state == DCACHE_LD_WAIT) begin
                if (~real_request || ((waiting_mem_tag == Dmem2proc_tag) && (waiting_mem_tag != 3'b0))) begin
                    state <= DCACHE_IDLE_HIT;
                    waiting_mem_tag <= 4'b0;
                end
            end else if (state == DCACHE_ST_WAIT) begin
                if (~real_request || ((waiting_mem_tag == Dmem2proc_tag) && (waiting_mem_tag != 3'b0))) begin
                    state <= DCACHE_IDLE_HIT;
                    waiting_mem_tag <= 4'b0;
                end
            end
        end
    end

endmodule