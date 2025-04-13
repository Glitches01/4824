module inst_buffer #(
    parameter DEPTH = 16,
    parameter WIDTH = 64,
    parameter ADDR  = $clog2(DEPTH)
) (
    input                   clock,
    input                   reset,

    input                   squash,

    input  IF_IB_PACKET     if_ib_packet[0:1],

    output IB_ID_PACKET     ib_id_packet[0:1]
);


    IF_IB_PACKET buffer[0:DEPTH-1][0:1];
    logic [ADDR:0] w_ptr;
    logic [ADDR:0] r_ptr;
    logic w_en, r_en;//todo
    IB_ID_PACKET r_data[0:1];
    logic full, empty;

    assign full  = (w_ptr[ADDR] == ~r_ptr[ADDR]) && (w_ptr[ADDR-1:0] == r_ptr[ADDR-1:0]);
    assign empty = (w_ptr == r_ptr)? 1'b1 : 1'b0;
    assign w_en  = if_ib_packet[0].valid;
    assign r_en  = 1;//todo

    assign ib_id_packet = r_data;

    always_ff @( posedge clock ) begin : update
        if(reset) begin
            for (integer i = 0; i < DEPTH; i = i + 1) begin
                buffer[i][0] <= 0;
                buffer[i][1] <= 0;
            end
            w_ptr <= 'd0;
        end else if(squash) begin
            w_ptr <= 0;
        end else if(w_en & !full) begin
            buffer[w_ptr[ADDR-1:0]] <= if_ib_packet;
            w_ptr <= w_ptr + 1;
        end
    end

    always_ff @( posedge clock ) begin : read
        if (reset) begin
            r_data[0] <= 0;
            r_data[1] <= 0;
            r_ptr <= 0;
        end else if(squash) begin
            r_ptr <= 0;
        end else if((r_en) && (!empty)) begin
            r_data[0].valid <= buffer[r_ptr[ADDR-1:0]][0].valid;
            r_data[0].inst  <= buffer[r_ptr[ADDR-1:0]][0].inst;
            r_data[0].NPC   <= buffer[r_ptr[ADDR-1:0]][0].NPC;
            r_data[0].PC    <= buffer[r_ptr[ADDR-1:0]][0].PC;

            r_data[1].valid <= buffer[r_ptr[ADDR-1:0]][1].valid;
            r_data[1].inst  <= buffer[r_ptr[ADDR-1:0]][1].inst;
            r_data[1].NPC   <= buffer[r_ptr[ADDR-1:0]][1].NPC;
            r_data[1].PC    <= buffer[r_ptr[ADDR-1:0]][1].PC;
            r_ptr <= r_ptr + 1;
        end
    end

    // IF_IB_PACKET buffer[0:DEPTH-1];
    // logic [$clog2(DEPTH):0] w_ptr;
    // logic [$clog2(DEPTH):0] r_ptr;
    // logic w_en, r_en;//todo
    // IB_ID_PACKET r_data;
    // logic full, empty;

    // assign full  = (w_ptr[$clog2(DEPTH)] != r_ptr[$clog2(DEPTH)]) && (w_ptr[$clog2(DEPTH)-1:0] == r_ptr[$clog2(DEPTH)-1:0]);
    // assign empty = (w_ptr == r_ptr)? 1'b1 : 1'b0;
    // assign w_en  = if_ib_packet.valid;
    // assign r_en  = 1;//todo

    // assign ib_id_packet = r_data;

    // always_ff @( posedge clock ) begin : update
    //     if(reset) begin
    //         for (integer i = 0; i < DEPTH; i = i + 1) begin
    //             buffer[i] <= 0;
    //         end
    //         w_ptr <= 'd0;
    //     end else if(squash) begin
    //         w_ptr <= 0;
    //     end else if(w_en & !full) begin
    //         buffer[w_ptr] <= if_ib_packet;
    //         w_ptr <= w_ptr + 1;
    //     end
    // end
    // logic read_phase;
    // always_ff @( posedge clock ) begin : read
    //     if (reset) begin
    //         r_data <= 0;
    //         r_ptr <= 'd0;
    //         read_phase <= 0;
    //     end else if(squash) begin
    //         r_ptr <= 0;
    //         read_phase <= 0;
    //     end else if((r_en) && (!empty) && (read_phase == 0)) begin
    //         r_data.valid <= buffer[r_ptr].valid;
    //         r_data.inst  <= buffer[r_ptr].inst[0];
    //         r_data.NPC   <= buffer[r_ptr].NPC;
    //         r_data.PC    <= buffer[r_ptr].PC;
    //         read_phase   <= 1;
    //     end else if((r_en) && (!empty) && (read_phase == 1)) begin
    //         r_data.valid <= buffer[r_ptr].valid;
    //         r_data.inst  <= buffer[r_ptr].inst[1];
    //         r_data.NPC   <= buffer[r_ptr].NPC;
    //         r_data.PC    <= buffer[r_ptr].PC;
    //         read_phase   <= 0;
    //         r_ptr        <= r_ptr + 1;
    //     end
    // end

    // logic [WIDTH-1:0] buffer[0:DEPTH-1];
    // logic [$clog2(DEPTH):0] w_ptr, r_ptr;
    // logic w_en, r_en;
    // logic [WIDTH-1:0] r_data;
    // logic full, empty;

    // assign full = (w_ptr[$clog2(DEPTH)] != r_ptr[$clog2(DEPTH)]) && (w_ptr[$clog2(DEPTH)-1:0] == r_ptr[$clog2(DEPTH)-1:0]);
    // assign empty = (w_ptr == r_ptr)? 1'b1 : 1'b0;

    // assign 

    // always_ff @( clock ) begin : update
    //     if(reset) begin
    //         for (integer i = 0; i < DEPTH; i = i + 1) begin
    //             buffer[i] <= 'd0;
    //         end
    //         w_ptr <= 'd0;
    //     end else if(w_en & !full) begin
    //         buffer[w_ptr] <= {if_ib_packet.inst[1],if_ib_packet.inst[0]};
    //         w_ptr <= w_ptr + 1;
    //     end
    // end

    // always_ff @( clock ) begin : read
    //     if (reset) begin
    //         r_data <= 'd0;
    //         r_ptr <= 'd0;
    //     end else if(r_en & !empty) begin
    //         r_data <= buffer[r_ptr];
    //         r_ptr <= r_ptr + 1;
    //     end
    // end

    
endmodule